/* UART MIDI RX example for ESP32-S3
 *
 * Receives raw UART data, dumps it in HEX,
 * and recognizes basic MIDI messages.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdlib.h>

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (CONFIG_EXAMPLE_UART_TXD)
#define RXD_PIN (CONFIG_EXAMPLE_UART_RXD)

static const char *TAG_UART = "UART";
static const char *TAG_MIDI = "MIDI";

/* ---------------- MIDI TYPES ---------------- */

typedef enum {
    MIDI_UNKNOWN,
    MIDI_NOTE_OFF,
    MIDI_NOTE_ON,
    MIDI_CC,
    MIDI_PROGRAM_CHANGE,
    MIDI_PITCH_BEND
} midi_type_t;

/* ---------------- MIDI HELPERS ---------------- */

static midi_type_t midi_get_type(uint8_t status)
{
    switch (status & 0xF0) {
        case 0x80: return MIDI_NOTE_OFF;
        case 0x90: return MIDI_NOTE_ON;
        case 0xB0: return MIDI_CC;
        case 0xC0: return MIDI_PROGRAM_CHANGE;
        case 0xE0: return MIDI_PITCH_BEND;
        default:   return MIDI_UNKNOWN;
    }
}

static const char *midi_type_to_string(midi_type_t type)
{
    switch (type) {
        case MIDI_NOTE_OFF:       return "NOTE_OFF";
        case MIDI_NOTE_ON:        return "NOTE_ON";
        case MIDI_CC:             return "CONTROL_CHANGE";
        case MIDI_PROGRAM_CHANGE: return "PROGRAM_CHANGE";
        case MIDI_PITCH_BEND:     return "PITCH_BEND";
        default:                  return "UNKNOWN";
    }
}

/* ---------------- MIDI PARSER ---------------- */

static void parse_and_print_midi(uint8_t *data, int len)
{
    int i = 0;

    while (i < len) {
        uint8_t status = data[i];

        // MIDI status byte must have MSB = 1
        if ((status & 0x80) == 0) {
            i++;
            continue;
        }

        midi_type_t type = midi_get_type(status);
        uint8_t channel = (status & 0x0F) + 1;

        int msg_len = 0;

        if (type == MIDI_PROGRAM_CHANGE) {
            msg_len = 2;
        } else if (type != MIDI_UNKNOWN) {
            msg_len = 3;
        }

        if (msg_len == 0 || (i + msg_len) > len) {
            i++;
            continue;
        }

        uint8_t code1 = data[i + 1];
        uint8_t code2 = (msg_len == 3) ? data[i + 2] : 0;

        ESP_LOGI(TAG_MIDI,
                 "[ \"%d\" \"%s\" \"%d\" \"%d\" ]",
                 channel,
                 midi_type_to_string(type),
                 code1,
                 code2);

        i += msg_len;
    }
}

/* ---------------- UART INIT ---------------- */

static void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 31250,               // MIDI baud rate
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG_UART, "UART initialized for MIDI (31250 baud)");
}

/* ---------------- RX TASK ---------------- */

static void rx_task(void *arg)
{
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE);

    while (1) {
        int rxBytes = uart_read_bytes(
            UART_NUM_1,
            data,
            RX_BUF_SIZE,
            pdMS_TO_TICKS(1000)
        );

        if (rxBytes > 0) {
            ESP_LOGI(TAG_UART, "Received %d bytes", rxBytes);

            // Dump ALL received data
            ESP_LOG_BUFFER_HEXDUMP(TAG_UART, data, rxBytes, ESP_LOG_INFO);

            // Try to decode MIDI messages
            parse_and_print_midi(data, rxBytes);
        }
    }

    free(data);
}

/* ---------------- MAIN ---------------- */

void app_main(void)
{
    uart_init();

    xTaskCreate(
        rx_task,
        "uart_rx_task",
        CONFIG_EXAMPLE_TASK_STACK_SIZE,
        NULL,
        configMAX_PRIORITIES - 1,
        NULL
    );
}
