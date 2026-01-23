// SPDX-FileCopyrightText: 2018-2032 mrcodetastic
// SPDX-FileCopyrightText: 2025 Aaron White <w531t4@gmail.com>
// SPDX-License-Identifier: MIT
#ifndef _ESP32_RGB_64_32_MATRIX_PANEL_SPI_CONFIG
#define _ESP32_RGB_64_32_MATRIX_PANEL_SPI_CONFIG

#include <stdint.h>
// #include cstdint.h
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include <esp_err.h>
#include <esp_log.h>

// Adapted from
// ESP32-HUB75-MatrixPanel-DMA/src/platforms/esp32/esp32-default-pins.hpp
#define SPI_CLK_PIN_DEFAULT 14
#define SPI_MOSI_PIN_DEFAULT 2
#define SPI_CE_PIN_DEFAULT 15

#define FPGA_RESETSTATUS_PIN_DEFAULT -1
#define FPGA_BUSY_PIN_DEFAULT -1

// Adapted from
// ESP32-HUB75-MatrixPanel-DMA/src/ESP32-HUB75-MatrixPanel-I2S-DMA.h

#ifndef MATRIX_WIDTH
#define MATRIX_WIDTH 64 // Single panel of 64 pixel width
#endif

#ifndef MATRIX_HEIGHT
#define MATRIX_HEIGHT                                                          \
    32 // CHANGE THIS VALUE to 64 IF USING 64px HIGH panel(s) with E PIN
#endif

#ifndef CHAIN_LENGTH
#define CHAIN_LENGTH                                                           \
    1 // Number of modules chained together, i.e. 4 panels chained result in
      // virtualmatrix 64x4=256 px long
#endif

#ifdef PIXEL_COLOR_DEPTH_BITS
#define PIXEL_COLOR_DEPTH_BITS_DEFAULT PIXEL_COLOR_DEPTH_BITS
#else
#define PIXEL_COLOR_DEPTH_BITS_DEFAULT 8
#endif

#define PIXEL_COLOR_DEPTH_BITS_MAX 12

/** @brief - configuration values for HUB75_I2S driver
 *  This structure holds configuration vars that are used as
 *  an initialization values when creating an instance of MatrixPanel_I2S_DMA
 * object. All params have it's default values.
 */
struct FPGA_SPI_CFG {

    /**
     * I2S clock speed selector
     */
    enum clk_speed {
        HZ_8M = 8000000,
        HZ_10M = 8000000,
        HZ_15M = 15000000, // for compatability
        HZ_16M = 16000000,
        HZ_20M = 20000000, // for compatability
        HZ_26M = 26666666, // for compatability
        HZ_40M = 40000000,
        HZ_80M = 80000000
    };

    //
    // Members must be in order of declaration or it breaks Arduino compiling
    // due to strict checking.
    //

    // physical width of a single matrix panel module (in pixels, usually it is
    // 64 ;) )
    uint16_t mx_width;

    // physical height of a single matrix panel module (in pixels, usually
    // almost always it is either 32 or 64)
    uint16_t mx_height;

    // number of chained panels regardless of the topology, default 1 - a single
    // matrix module
    uint16_t chain_length;

    // GPIO Mapping
    struct spi_pins {
        int8_t ce, clk, mosi, fpga_resetstatus, fpga_busy;
    } gpio;

    // SPI clock speed
    clk_speed spispeed;

    // Minimum refresh / scan rate needs to be configured on start due to
    // LSBMSB_TRANSITION_BIT calculation in allocateDMAmemory() Set this to '1'
    // to get all colour depths displayed with correct BCM time weighting.
    uint8_t min_refresh_rate;

    // FPGA status timeouts (milliseconds)
    uint16_t fpga_resetstatus_timeout_ms;
    uint16_t fpga_busy_timeout_ms;

    // struct constructor
    FPGA_SPI_CFG(
        uint16_t _w = MATRIX_WIDTH, uint16_t _h = MATRIX_HEIGHT,
        uint16_t _chain = CHAIN_LENGTH,
        spi_pins _pinmap = {SPI_CE_PIN_DEFAULT, SPI_CLK_PIN_DEFAULT,
                            SPI_MOSI_PIN_DEFAULT, FPGA_RESETSTATUS_PIN_DEFAULT,
                            FPGA_BUSY_PIN_DEFAULT},
        clk_speed _spispeed = HZ_8M, uint16_t _min_refresh_rate = 60,
        uint8_t _pixel_color_depth_bits = PIXEL_COLOR_DEPTH_BITS_DEFAULT,
        uint16_t _fpga_resetstatus_timeout_ms = 1000,
        uint16_t _fpga_busy_timeout_ms = 1000)
        : mx_width(_w), mx_height(_h), chain_length(_chain), gpio(_pinmap),
          spispeed(_spispeed), min_refresh_rate(_min_refresh_rate),
          fpga_resetstatus_timeout_ms(_fpga_resetstatus_timeout_ms),
          fpga_busy_timeout_ms(_fpga_busy_timeout_ms) {
        setPixelColorDepthBits(_pixel_color_depth_bits);
    }

    // pixel_color_depth_bits must be between 12 and 2, and mask_offset needs to
    // be calculated accordently so they have to be private with getter (and
    // setter)
    void setPixelColorDepthBits(uint8_t _pixel_color_depth_bits) {
        if (_pixel_color_depth_bits > PIXEL_COLOR_DEPTH_BITS_MAX ||
            _pixel_color_depth_bits < 2) {

            if (_pixel_color_depth_bits > PIXEL_COLOR_DEPTH_BITS_MAX) {
                pixel_color_depth_bits = PIXEL_COLOR_DEPTH_BITS_MAX;
            } else {
                pixel_color_depth_bits = 2;
            }
        } else {
            pixel_color_depth_bits = _pixel_color_depth_bits;
        }
    }

    uint8_t getPixelColorDepthBits() const { return pixel_color_depth_bits; }

  private:
    // these were priviously handeld as defines (PIXEL_COLOR_DEPTH_BITS,
    // MASK_OFFSET) to make it changable after compilation, it is now part of
    // the config
    uint8_t pixel_color_depth_bits;
}; // end of structure HUB75_I2S_CFG

#endif
