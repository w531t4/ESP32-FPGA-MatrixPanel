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

#ifndef ESPHOME_BUILD
extern "C" void app_main(void) { /* demo/standalone */ }
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

}
#endif
