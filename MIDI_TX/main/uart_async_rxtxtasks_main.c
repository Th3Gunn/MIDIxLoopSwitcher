#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdio.h>

#define MIDI_UART      UART_NUM_1
#define MIDI_TX_PIN    CONFIG_EXAMPLE_UART_TXD
#define MIDI_BAUD      31250

#define MIDI_CHANNEL   0   // 0 = channel 1
#define CC_NUMBER      11  // CC #11
#define CC_VALUE       0   // Value 0

static const char *TAG = "MIDI_TX";

// Initialize UART for MIDI
void midi_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = MIDI_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(MIDI_UART, 256, 0, 0, NULL, 0);
    uart_param_config(MIDI_UART, &uart_config);
    uart_set_pin(MIDI_UART,
                 MIDI_TX_PIN,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
}

// Send a single Control Change message
static void midi_send_cc(uint8_t channel, uint8_t cc, uint8_t value)
{
    uint8_t status = 0xB0 | (channel & 0x0F); // Ensure correct first byte
    uint8_t msg[3] = { status, cc & 0x7F, value & 0x7F };

    // Send bytes via UART
    uart_write_bytes(MIDI_UART, (const char *)msg, 3);

    // Log the decimal values of the MIDI message
    printf("Sent MIDI CC: %d, %d, %d\n", msg[0], msg[1], msg[2]);
}

static void tx_task(void *arg)
{
    ESP_LOGI(TAG, "MIDI TX task started");

    while (1) {
        midi_send_cc(MIDI_CHANNEL, CC_NUMBER, CC_VALUE);
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5-second delay
    }
}

void app_main(void)
{
    midi_uart_init();
    xTaskCreate(tx_task,
                "midi_tx_task",
                CONFIG_EXAMPLE_TASK_STACK_SIZE,
                NULL,
                configMAX_PRIORITIES - 2,
                NULL);
}
