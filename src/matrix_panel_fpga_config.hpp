#ifndef _ESP32_RGB_64_32_MATRIX_PANEL_SPI_CONFIG
#define _ESP32_RGB_64_32_MATRIX_PANEL_SPI_CONFIG

#include <stdint.h>
// #include cstdint.h
#include <esp_err.h>
#include <esp_log.h>
#include "esp_attr.h"
#include "esp_heap_caps.h"

// Adapted from ESP32-HUB75-MatrixPanel-DMA/src/platforms/esp32/esp32-default-pins.hpp
#define SPI_CLK_PIN_DEFAULT 14
#define SPI_MOSI_PIN_DEFAULT 2
#define SPI_CE_PIN_DEFAULT 15
// #define R1_PIN_DEFAULT  25
// #define G1_PIN_DEFAULT  26
// #define B1_PIN_DEFAULT  27
// #define R2_PIN_DEFAULT  14
// #define G2_PIN_DEFAULT  12
// #define B2_PIN_DEFAULT  13

// #define A_PIN_DEFAULT   23
// #define B_PIN_DEFAULT   19
// #define C_PIN_DEFAULT   5
// #define D_PIN_DEFAULT   17
// #define E_PIN_DEFAULT   -1 // IMPORTANT: Change to a valid pin if using a 64x64px panel.

// #define LAT_PIN_DEFAULT 4
// #define OE_PIN_DEFAULT  15
// #define CLK_PIN_DEFAULT 16




// Adapted from ESP32-HUB75-MatrixPanel-DMA/src/ESP32-HUB75-MatrixPanel-I2S-DMA.h

#ifndef MATRIX_WIDTH
#define MATRIX_WIDTH 64 // Single panel of 64 pixel width
#endif

#ifndef MATRIX_HEIGHT
#define MATRIX_HEIGHT 32 // CHANGE THIS VALUE to 64 IF USING 64px HIGH panel(s) with E PIN
#endif

#ifndef CHAIN_LENGTH
#define CHAIN_LENGTH 1 // Number of modules chained together, i.e. 4 panels chained result in virtualmatrix 64x4=256 px long
#endif

#ifdef PIXEL_COLOR_DEPTH_BITS
#define PIXEL_COLOR_DEPTH_BITS_DEFAULT PIXEL_COLOR_DEPTH_BITS
#else
#define PIXEL_COLOR_DEPTH_BITS_DEFAULT 8
#endif

#define PIXEL_COLOR_DEPTH_BITS_MAX 12

/** @brief - configuration values for HUB75_I2S driver
 *  This structure holds configuration vars that are used as
 *  an initialization values when creating an instance of MatrixPanel_I2S_DMA object.
 *  All params have it's default values.
 */
struct FPGA_SPI_CFG
{

//   /**
//    * Enumeration of hardware-specific chips
//    * used to drive matrix modules
//    */
//   enum shift_driver
//   {
//     SHIFTREG = 0,
//     FM6124,
//     FM6126A,
//     ICN2038S,
//     MBI5124,
//     SM5266P,
//     DP3246_SM5368
//   };

  /**
   * I2S clock speed selector
   */
  enum clk_speed
  {
    HZ_8M = 8000000,
    HZ_10M = 8000000,
    HZ_15M = 16000000, // for compatability
    HZ_16M = 16000000,
    HZ_20M = 20000000 // for compatability
  };

  //
  // Members must be in order of declaration or it breaks Arduino compiling due to strict checking.
  //

  // physical width of a single matrix panel module (in pixels, usually it is 64 ;) )
  uint16_t mx_width;

  // physical height of a single matrix panel module (in pixels, usually almost always it is either 32 or 64)
  uint16_t mx_height;

  // number of chained panels regardless of the topology, default 1 - a single matrix module
  uint16_t chain_length;

  // GPIO Mapping
  struct spi_pins
  {
    int8_t ce, clk, mosi;
    // int8_t r1, g1, b1, r2, g2, b2, a, b, c, d, e, lat, oe, clk;
  } gpio;

//   // Matrix driver chip type - default is a plain shift register
//   shift_driver driver;

//   // use DMA double buffer (twice as much RAM required)
//   bool double_buff;

  // SPI clock speed
  clk_speed spispeed;

//   // How many clock cycles to blank OE before/after LAT signal change, default is 1 clock
//   uint8_t latch_blanking;

  // Minimum refresh / scan rate needs to be configured on start due to LSBMSB_TRANSITION_BIT calculation in allocateDMAmemory()
  // Set this to '1' to get all colour depths displayed with correct BCM time weighting.
  uint8_t min_refresh_rate;

  // struct constructor
  FPGA_SPI_CFG(
      uint16_t _w = MATRIX_WIDTH,
      uint16_t _h = MATRIX_HEIGHT,
      uint16_t _chain = CHAIN_LENGTH,
      spi_pins _pinmap = {
          SPI_CE_PIN_DEFAULT, SPI_CLK_PIN_DEFAULT, SPI_MOSI_PIN_DEFAULT},
    //   shift_driver _drv = SHIFTREG,
    //   bool _dbuff = false,
      clk_speed _spispeed = HZ_8M,
    //   uint8_t _latblk = DEFAULT_LAT_BLANKING, // Anything > 1 seems to cause artefacts on ICS panels
    //   bool _clockphase = true,
      uint16_t _min_refresh_rate = 60,
      uint8_t _pixel_color_depth_bits = PIXEL_COLOR_DEPTH_BITS_DEFAULT)
      : mx_width(_w),
        mx_height(_h),
        chain_length(_chain),
        gpio(_pinmap),
        //driver(_drv), double_buff(_dbuff),
        spispeed(_spispeed),
        // latch_blanking(_latblk),
        // clkphase(_clockphase),
        min_refresh_rate(_min_refresh_rate)
  {
    setPixelColorDepthBits(_pixel_color_depth_bits);
  }

  // pixel_color_depth_bits must be between 12 and 2, and mask_offset needs to be calculated accordently
  // so they have to be private with getter (and setter)
  void setPixelColorDepthBits(uint8_t _pixel_color_depth_bits)
  {
    if (_pixel_color_depth_bits > PIXEL_COLOR_DEPTH_BITS_MAX || _pixel_color_depth_bits < 2)
    {

      if (_pixel_color_depth_bits > PIXEL_COLOR_DEPTH_BITS_MAX)
      {
        pixel_color_depth_bits = PIXEL_COLOR_DEPTH_BITS_MAX;
      }
      else
      {
        pixel_color_depth_bits = 2;
      }
      // ESP_LOGW("HUB75_I2S_CFG", "Invalid pixel_color_depth_bits (%d): 2 <= pixel_color_depth_bits <= %d, choosing nearest valid %d", _pixel_color_depth_bits, PIXEL_COLOR_DEPTH_BITS_MAX, pixel_color_depth_bits);
    }
    else
    {
      pixel_color_depth_bits = _pixel_color_depth_bits;
    }
  }

  uint8_t getPixelColorDepthBits() const
  {
    return pixel_color_depth_bits;
  }

private:
  // these were priviously handeld as defines (PIXEL_COLOR_DEPTH_BITS, MASK_OFFSET)
  // to make it changable after compilation, it is now part of the config
  uint8_t pixel_color_depth_bits;
}; // end of structure HUB75_I2S_CFG

#endif
