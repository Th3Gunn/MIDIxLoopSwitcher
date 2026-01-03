#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

// GPIO pins
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

// MAX7219 Registers
#define MAX7219_REG_NOOP        0x00
#define MAX7219_REG_DIGIT0      0x01
#define MAX7219_REG_DIGIT1      0x02
#define MAX7219_REG_DIGIT2      0x03
#define MAX7219_REG_DIGIT3      0x04
#define MAX7219_REG_DIGIT4      0x05
#define MAX7219_REG_DIGIT5      0x06
#define MAX7219_REG_DIGIT6      0x07
#define MAX7219_REG_DIGIT7      0x08
#define MAX7219_REG_DECODEMODE  0x09
#define MAX7219_REG_INTENSITY   0x0A
#define MAX7219_REG_SCANLIMIT   0x0B
#define MAX7219_REG_SHUTDOWN    0x0C
#define MAX7219_REG_DISPLAYTEST 0x0F

static spi_device_handle_t spi;

// SPI init
void spi_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1*1000*1000, // 1 MHz
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if(ret != ESP_OK) ESP_LOGE("SPI", "Bus init failed");

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if(ret != ESP_OK) ESP_LOGE("SPI", "Add device failed");
}

// Send one register/data pair to MAX7219
void max7219_send(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2] = { reg, data };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
    };
    esp_err_t ret = spi_device_transmit(spi, &t);
    if(ret != ESP_OK) ESP_LOGE("MAX7219", "Transmit failed");
}

// Initialize MAX7219
void max7219_init(void)
{
    max7219_send(MAX7219_REG_SHUTDOWN, 0x00); // shutdown
    max7219_send(MAX7219_REG_DISPLAYTEST, 0x00); // normal operation
    max7219_send(MAX7219_REG_SCANLIMIT, 0x07);   // digits 0-7
    max7219_send(MAX7219_REG_DECODEMODE, 0xFF);  // Code B decode for digits
    max7219_send(MAX7219_REG_INTENSITY, 0x0F);   // max brightness
    max7219_send(MAX7219_REG_SHUTDOWN, 0x01);   // normal operation

    // Clear display
    for(int i = 1; i <= 8; i++)
        max7219_send(i, 0x0F); // blank
}

// Display a number on all digits
void max7219_display_number(uint8_t num)
{
    for(int i = 1; i <= 8; i++)
        max7219_send(i, num);
}

// Main task
void app_main(void)
{
    spi_init();
    max7219_init();

    uint8_t sequence[] = {1, 2, 3};
    size_t seq_len = sizeof(sequence)/sizeof(sequence[0]);
    size_t idx = 0;

    while(1) {
        max7219_display_number(sequence[idx]);
        idx = (idx + 1) % seq_len;
        vTaskDelay(pdMS_TO_TICKS(500)); // 500 ms delay
    }
}
