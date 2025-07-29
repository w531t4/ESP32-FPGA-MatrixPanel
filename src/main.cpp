// #include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "matrix_panel_fpga_config.hpp"
#include "matrix_panel_fpga.hpp"
#include "string.h"


// #define PIN_NUM_MOSI 2
// #define PIN_NUM_CLK  14
// #define PIN_NUM_CS   15
// USE HSPI
//  - MISO  - IO12 - 14 - SD_D2
//  - MOSI  - IO13 - 16 - SD_D3
//  - SCK   - IO14 - 13 - SD_CLK
//  - SS    - IO15 - 23 - SD_CMD
spi_device_handle_t spi;

// void app_main(void)
// {
//     int i = 0;
//     while (1) {
//         printf("[%d] Hello world!\n", i);
//         i++;
//         vTaskDelay(5000 / portTICK_PERIOD_MS);
//     }
void setBrightness(spi_device_handle_t &d, uint8_t level) {
    spi_transaction_t t = {
        .length = 8, // bits
        .tx_buffer = &level,
    };
    spi_device_transmit(d, &t); // blocking
}

extern "C" void app_main(void) {
    FPGA_SPI_CFG mxconfig_;

    mxconfig_.spispeed = FPGA_SPI_CFG::clk_speed::HZ_20M;
    MatrixPanel_FPGA_SPI *dma_display_ = new MatrixPanel_FPGA_SPI(mxconfig_);
    dma_display_->begin();
    while (true) {
        dma_display_->fillScreenRGB888(static_cast<uint8_t>(3), static_cast<uint8_t>(30), static_cast<uint8_t>(255));
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 3 seconds
        dma_display_->setBrightness8(static_cast<uint8_t>(25));
        vTaskDelay(pdMS_TO_TICKS(2000));  // Delay for 3 seconds
        dma_display_->clearScreen();
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 3 seconds
        dma_display_->setBrightness8(static_cast<uint8_t>(175));
    }

    // spi_bus_config_t buscfg = {
    //     .mosi_io_num = (gpio_num_t)mxconfig_.gpio.mosi,
    //     .miso_io_num = -1, // Not used
    //     .sclk_io_num = (gpio_num_t)mxconfig_.gpio.clk,
    //     .quadwp_io_num = -1,
    //     .quadhd_io_num = -1,
    // };

    // spi_device_interface_config_t devcfg = {
    //     .mode = 0,                          // SPI mode 0
    //     .clock_speed_hz = mxconfig_.spispeed, // 10 MHz

    //     .spics_io_num = (gpio_num_t)mxconfig_.gpio.ce,
    //     .queue_size = 1,
    // };

    // // Initialize SPI bus and device
    // gpio_reset_pin((gpio_num_t)mxconfig_.gpio.mosi);
    // gpio_reset_pin((gpio_num_t)mxconfig_.gpio.clk);
    // gpio_reset_pin((gpio_num_t)mxconfig_.gpio.ce);
    // spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    // spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    // // gpio_set_direction(PIN_NUM_MOSI, GPIO_MODE_OUTPUT);
    // // gpio_set_level(PIN_NUM_MOSI, 0);  // Does pin go HIGH?

    // // Example data
    // // uint8_t data[] = {0x12, 0x34, 0x56, 0x78};
    // // uint8_t data[] = {'r','G','B', '7'};
    // // uint8_t data[64*3+6];
    // uint8_t data[4];
    // memset(data, '1', sizeof(data));
    // // Send each 8-bit chunk
    // while (true) {
    //     // for (uint8_t row = 0; row < 4; row++) {
    //         data[0] = '1';
    //         data[1] = '2';
    //         data[2] = '3';
    //         data[3] = '4';
    //         // data[5] = row;
    //         // data[0] = 'R';
    //         // data[1] = 'b';
    //         // data[2] = 'g';
    //         // data[3] = '6';
    //         // data[5] = row;
    //         for (int i = 0; i < sizeof(data); i++) {
    //             spi_transaction_t t = {
    //                 .length = 8, // bits
    //                 .tx_buffer = &data[i],
    //             };
    //             spi_device_transmit(spi, &t); // blocking
    //              vTaskDelay(pdMS_TO_TICKS(100));  // Delay for 3 seconds
    //         }

    //     // }

    // }
}
