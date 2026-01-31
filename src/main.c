#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "common/mavlink.h"

#define UART_TELEM        UART_NUM_2
#define UART_TX_PIN       GPIO_NUM_17
#define UART_RX_PIN       GPIO_NUM_16
#define UART_BAUD         57600

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
    uart_driver_install(UART_TELEM, 4096, 2048, 0, NULL, 0);
    uart_param_config(UART_TELEM, &cfg);
    uart_set_pin(UART_TELEM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void mav_send_msg(const mavlink_message_t *msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    uart_write_bytes(UART_TELEM, (const char*)buf, len);
}

static void heartbeat_task(void *arg) {
    (void)arg;

    const uint8_t sysid  = 1;
    const uint8_t compid = MAV_COMP_ID_AUTOPILOT1;

    while (1) {
        mavlink_message_t msg;

        mavlink_msg_heartbeat_pack(
            sysid,
            compid,
            &msg,
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_GENERIC,                 // you can swap to MAV_AUTOPILOT_PX4 later if you want
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,     // minimal base mode
            0,                                     // custom_mode
            MAV_STATE_STANDBY
        );

        mav_send_msg(&msg);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz
    }
}

void app_main(void) {
    printf("Boot: sending MAVLink heartbeat on UART2 (TX=%d RX=%d) @ %d\n",
           UART_TX_PIN, UART_RX_PIN, UART_BAUD);

    telem_uart_init();
    xTaskCreate(heartbeat_task, "hb", 4096, NULL, 2, NULL);
}