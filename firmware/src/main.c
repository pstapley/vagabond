#include <stdio.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_assert.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"


#include "common/mavlink.h"

#define UART_TELEM        UART_NUM_2
#define UART_TX_PIN       GPIO_NUM_17
#define UART_RX_PIN       GPIO_NUM_16
#define UART_BAUD         57600

#define PIN_NUM_MISO        19
#define PIN_NUM_MOSI        23
#define PIN_NUM_CLK         18
#define FLASH_PIN_NUM_CS    5
#define IMU_PIN_NUM_CS      21

// --- ICM-42688-P registers (Bank 0) ---
#define REG_WHO_AM_I      0x75
#define REG_DEVICE_CONFIG 0x11
#define REG_PWR_MGMT0     0x4E
#define REG_GYRO_CONFIG0  0x4F
#define REG_ACCEL_CONFIG0 0x50

// WHO_AM_I expected for ICM-42688-P
#define ICM42688_WHO_AM_I  0x75
#define ICM42688_ID        0x47

#define IMU_SPI_HZ       (2 * 1000 * 1000)   // start slow for bring-up
static spi_device_handle_t imu_dev;          // keep as a global/static so other code can use it

static const char *TAG = "extflash";

#define MAV_POOL_SIZE 32

typedef struct {
    mavlink_message_t msg;
    int64_t t_us;
} mavlink_message_entry_t;

#define MAV_TX_Q_DEPTH 32

typedef struct {
    uint16_t len;
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
} mav_tx_frame_t;

static QueueHandle_t s_mav_tx_q;

static mavlink_message_entry_t s_pool[MAV_POOL_SIZE];
static QueueHandle_t s_free_q;
static QueueHandle_t s_rx_q;

static bool g_armed = false;

void spi_bus_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
}

void extflash_init(void) {
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,   // start at 10 MHz; you can go higher later
        .mode = 0,                            // SPI mode 0 is typical for W25Q
        .spics_io_num = FLASH_PIN_NUM_CS,
        .queue_size = 4,
    };

    spi_device_handle_t flash_dev;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &flash_dev));

    ESP_LOGI(TAG, "SPI external flash device added");

}

void imu_init(void)
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = IMU_SPI_HZ,
        .mode = 0,                 // ICM-42688-P uses SPI mode 0
        .spics_io_num = IMU_PIN_NUM_CS,
        .queue_size = 4,
        // .cs_ena_pretrans = 0,
        // .cs_ena_posttrans = 0,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &imu_dev));
    ESP_LOGI(TAG, "SPI IMU device added");

    // Next step: do WHO_AM_I + basic config using imu_dev handle
}

esp_err_t imu_read_reg(uint8_t reg, uint8_t *data)
{
    uint8_t tx[2] = { reg | 0x80, 0x00 }; // read command + dummy
    uint8_t rx[2] = { 0 };

    spi_transaction_t t = {
        .length = 16,        // 2 bytes
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_transmit(imu_dev, &t);
    if (err != ESP_OK) {
        return err;
    }

    *data = rx[1]; // second byte is the register value
    return ESP_OK;
}

void imu_log_whoami(void)
{
    uint8_t whoami = 0;

    ESP_ERROR_CHECK(imu_read_reg(ICM42688_WHO_AM_I, &whoami));

    if (whoami == ICM42688_ID) {
        ESP_LOGI("IMU", "WHO_AM_I = 0x%02X ✅ (ICM-42688 detected)", whoami);
    }
    else {
        ESP_LOGE("IMU",
                 "Unexpected WHO_AM_I = 0x%02X (expected 0x%02X)",
                 whoami,
                 ICM42688_ID);
    }
}


static void mav_init_queues(void) {
    s_free_q = xQueueCreate(MAV_POOL_SIZE, sizeof(uint16_t));
    s_rx_q = xQueueCreate(MAV_POOL_SIZE, sizeof(uint16_t));

    configASSERT(s_free_q);
    configASSERT(s_rx_q);

    for (uint16_t i = 0; i < MAV_POOL_SIZE; i++) {
        xQueueSend(s_free_q, &i, 0);
    }
}

static void mav_tx_init(void) {
    s_mav_tx_q = xQueueCreate(MAV_TX_Q_DEPTH, sizeof(mav_tx_frame_t));
    configASSERT(s_mav_tx_q);
}

static bool mav_send_msg(const mavlink_message_t *msg, TickType_t wait) {
    mav_tx_frame_t frame;
    frame.len = mavlink_msg_to_send_buffer(frame.data, msg);

    return(xQueueSend(s_mav_tx_q, &frame, wait) == pdTRUE);
}

static void telem_uart_init(void) {
    const uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    // RX buffer big enough for bursts; TX buffered too
    ESP_ERROR_CHECK(uart_param_config(UART_TELEM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_TELEM, UART_TX_PIN, UART_RX_PIN,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_TELEM, 4096, 2048, 0, NULL, 0));

}

static void send_statustext(uint8_t severity, const char *text)
{
    mavlink_message_t msg;

    mavlink_msg_statustext_pack(
        1,                          // sysid
        MAV_COMP_ID_AUTOPILOT1,
        &msg,
        severity,
        text,
        0,      // id (chunking, ignore for now)
        0       // chunk_seq
    );

    mav_send_msg(&msg, pdMS_TO_TICKS(10));
}

static void heartbeat_task(void *arg) {
    (void)arg;

    const uint8_t sysid  = 1;
    const uint8_t compid = MAV_COMP_ID_AUTOPILOT1;

    while (1) {
        mavlink_message_t msg;

        uint8_t base_mode =
            MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
            MAV_MODE_FLAG_STABILIZE_ENABLED;

        if (g_armed) {
            base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
        }

        mavlink_msg_heartbeat_pack(
            sysid,
            compid,
            &msg,
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_GENERIC,
            base_mode,     // base mode
            0,     // custom_mode
            MAV_STATE_STANDBY
        );

        (void)mav_send_msg(&msg, 0);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz
    }
}

static void sys_status_task(void *arg) {
    (void)arg;

    const uint8_t sysid  = 1;
    const uint8_t compid = MAV_COMP_ID_AUTOPILOT1;

    while (1) {
        mavlink_message_t msg;

        mavlink_msg_sys_status_pack(
            sysid,
            compid,
            &msg,

            MAV_SYS_STATUS_SENSOR_3D_GYRO |
            MAV_SYS_STATUS_SENSOR_3D_ACCEL |
            MAV_SYS_STATUS_SENSOR_3D_MAG,

            MAV_SYS_STATUS_SENSOR_3D_GYRO |
            MAV_SYS_STATUS_SENSOR_3D_ACCEL |
            MAV_SYS_STATUS_SENSOR_3D_MAG,

            MAV_SYS_STATUS_SENSOR_3D_GYRO |
            MAV_SYS_STATUS_SENSOR_3D_ACCEL |
            MAV_SYS_STATUS_SENSOR_3D_MAG,

            500,    // load (50%)
            12000,  // battery mV
            -1,     // battery current (unknown)
            -1,     // remaining %
            0,0,0,0,0,0,0,0,0
        );

        (void)mav_send_msg(&msg, 0);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz
    }
}

static void gps_task(void *arg) {
    (void)arg;

    const uint8_t sysid  = 1;
    const uint8_t compid = MAV_COMP_ID_AUTOPILOT1;

    while (1) {
        mavlink_message_t msg;

        mavlink_msg_gps_raw_int_pack(
            sysid,
            compid,
            &msg,
            esp_timer_get_time() / 1000, // time_usec (doesn't matter much)

            3,                          // FIX_TYPE_3D

            (int32_t)(42.97554037704246 * 1e7),   // lat
            (int32_t)(-85.64667028223883 * 1e7),  // lon
            250000,                     // altitude mm (250m)

            65535, // eph (unknown)
            65535, // epv (unknown)

            0,     // velocity cm/s
            0,     // cog
            10,    // satellites visible

            0,     // alt_ellipsoid
            0,     // h_acc
            0,     // v_acc
            0,     // vel_acc
            0,     // hdg_acc
            0      // yaw
        );

        (void)mav_send_msg(&msg, 0);
        vTaskDelay(pdMS_TO_TICKS(200)); // 1 Hz
    }
}

static void mav_rx_task(void *arg) {
    (void)arg;

    uint8_t data[128];
    mavlink_message_t msg;
    mavlink_status_t status = {0};

    while (1) {
        int length = uart_read_bytes(UART_TELEM, data, sizeof(data), pdMS_TO_TICKS(100));
        if (length > 0) {
            for (int i = 0; i < length; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
                    uint16_t idx;
                    if (xQueueReceive(s_free_q, &idx, 0) == pdTRUE) {
                        s_pool[idx].msg = msg;
                        s_pool[idx].t_us = esp_timer_get_time();
                        
                        if(xQueueSend(s_rx_q, &idx, 0) != pdTRUE) {
                            xQueueSend(s_free_q, &idx, 0);
                        }
                    } else {
                        printf("Dropping MAVLink message with ID %d\n", msg.msgid);
                    }
                }
            }
        }
    }

}

static void mav_tx_task(void *arg) {
    (void)arg;

    mav_tx_frame_t frame;
    while (1) {
        if (xQueueReceive(s_mav_tx_q, &frame, portMAX_DELAY) == pdTRUE) {
            uart_write_bytes(UART_TELEM, (const char*)frame.data, frame.len);
        }
    }
}

static bool send_command_ack(const mavlink_message_t *cmd_msg, uint16_t command, uint8_t result)
{
    mavlink_message_t ack;

    // Reply to the sender (QGC): cmd_msg->sysid/compid
    mavlink_msg_command_ack_pack(
        1,                         // your sysid
        MAV_COMP_ID_AUTOPILOT1,     // your compid
        &ack,
        command,                   // MAV_CMD_*
        result,                    // MAV_RESULT_*
        0,                         // progress (0-100) - unused for now
        0,                         // result_param2 - unused for now
        cmd_msg->sysid,            // target_system = sender system (QGC = 255)
        cmd_msg->compid            // target_component = sender component (often 190)
    );

    // Queue it for TX. Small wait because ACKs are important.
    return mav_send_msg(&ack, pdMS_TO_TICKS(10));
}

static void router_task(void *arg) {
    uint16_t idx;

    while (1) {
        if (xQueueReceive(s_rx_q, &idx, portMAX_DELAY) == pdTRUE) {
            mavlink_message_t *m = &s_pool[idx].msg;
            printf("Received MAVLink message with ID %d\n", m->msgid);

            switch (m->msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    break;
                case MAVLINK_MSG_ID_COMMAND_LONG: {
                    mavlink_command_long_t cmd;
                    mavlink_msg_command_long_decode(m, &cmd);

                    printf("COMMAND_LONG: cmd=%u p1=%f p2=%f p3=%f p4=%f p5=%f p6=%f p7=%f from sys=%u comp=%u\n",
                        (unsigned)cmd.command,
                        cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5, cmd.param6, cmd.param7,
                        (unsigned)m->sysid, (unsigned)m->compid);

                    if (cmd.command == MAV_CMD_REQUEST_MESSAGE) {
                        uint32_t req_msgid = (uint32_t)cmd.param1;
                        printf("  -> REQUEST_MESSAGE: msgid=%u\n", (unsigned)req_msgid);

                        if (req_msgid == MAVLINK_MSG_ID_CAMERA_INFORMATION || req_msgid == MAVLINK_MSG_ID_CAMERA_SETTINGS || req_msgid == MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS) {
                            send_command_ack(m, cmd.command, MAV_RESULT_UNSUPPORTED);
                        } else {
                            send_command_ack(m, cmd.command, MAV_RESULT_UNSUPPORTED);
                        }
                    } else if(cmd.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                        g_armed = cmd.param1 > 0.5;
                        printf("ARM_DISARM command received: %f, armed=%d\n", cmd.param1, g_armed);
                        send_command_ack(m, cmd.command, MAV_RESULT_ACCEPTED);
                    } else {
                        // For now: tell QGC you heard it but don't support it yet
                        send_command_ack(m, cmd.command, MAV_RESULT_UNSUPPORTED);
                    }

                    break;
                }
                case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
                    printf("Log request\n");
                    break;
                default:
                    break;
            }

            xQueueSend(s_free_q, &idx, 0);
        }
    }
}

void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // let UART init messages finish
    printf("Boot: sending MAVLink heartbeat on UART2 (TX=%d RX=%d) @ %d\n",
           UART_TX_PIN, UART_RX_PIN, UART_BAUD);

    mav_init_queues();

    telem_uart_init();

    mav_tx_init();

    spi_bus_init();
    extflash_init();
    imu_init();
    imu_log_whoami();

    xTaskCreatePinnedToCore(heartbeat_task, "hb", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(sys_status_task, "status", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(gps_task, "gps", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(mav_rx_task, "rx", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(mav_tx_task, "tx", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(router_task, "router", 4096, NULL, 3, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(2000));
    send_statustext(MAV_SEVERITY_INFO, "ESP32 FC Online");
}