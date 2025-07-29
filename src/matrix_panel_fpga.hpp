#ifndef _ESP32_RGB_64_32_MATRIX_PANEL_SPI_DMA
#define _ESP32_RGB_64_32_MATRIX_PANEL_SPI_DMA
#include <stdint.h>
#include "matrix_panel_fpga_config.hpp"
#include "driver/spi_master.h"

/***************************************************************************************/
class MatrixPanel_FPGA_SPI
{


  // ------- PUBLIC -------
public:
  /**
   * MatrixPanel_FPGA_SPI
   *
   * default predefined values are used for matrix configuration
   *
   */
  MatrixPanel_FPGA_SPI()
  {
  }

  /**
   * MatrixPanel_FPGA_SPI
   *
   * @param  {FPGA_SPI_CFG} opts : structure with matrix configuration
   *
   */
  MatrixPanel_FPGA_SPI(const FPGA_SPI_CFG &opts)
  {
    setCfg(opts);
  }

  void init_spi(const FPGA_SPI_CFG &cfg);
  /* Propagate the DMA pin configuration, allocate DMA buffs and start data output, initially blank */
  bool begin()
  {

    if (initialized)
      return true; // we don't do this twice or more!

    if (!config_set)
      return false;

    ESP_LOGI("begin()", "Using GPIO %d for SPI_CE_PIN", m_cfg.gpio.ce);
    ESP_LOGI("begin()", "Using GPIO %d for SPI_CLK_PIN", m_cfg.gpio.clk);
    ESP_LOGI("begin()", "Using GPIO %d for SPI_MOSI_PIN", m_cfg.gpio.mosi);
    // ESP_LOGI("begin()", "Using GPIO %d for R1_PIN", m_cfg.gpio.r1);
    // ESP_LOGI("begin()", "Using GPIO %d for G1_PIN", m_cfg.gpio.g1);
    // ESP_LOGI("begin()", "Using GPIO %d for B1_PIN", m_cfg.gpio.b1);
    // ESP_LOGI("begin()", "Using GPIO %d for R2_PIN", m_cfg.gpio.r2);
    // ESP_LOGI("begin()", "Using GPIO %d for G2_PIN", m_cfg.gpio.g2);
    // ESP_LOGI("begin()", "Using GPIO %d for B2_PIN", m_cfg.gpio.b2);
    // ESP_LOGI("begin()", "Using GPIO %d for A_PIN", m_cfg.gpio.a);
    // ESP_LOGI("begin()", "Using GPIO %d for B_PIN", m_cfg.gpio.b);
    // ESP_LOGI("begin()", "Using GPIO %d for C_PIN", m_cfg.gpio.c);
    // ESP_LOGI("begin()", "Using GPIO %d for D_PIN", m_cfg.gpio.d);
    // ESP_LOGI("begin()", "Using GPIO %d for E_PIN", m_cfg.gpio.e);
    // ESP_LOGI("begin()", "Using GPIO %d for LAT_PIN", m_cfg.gpio.lat);
    // ESP_LOGI("begin()", "Using GPIO %d for OE_PIN", m_cfg.gpio.oe);
    // ESP_LOGI("begin()", "Using GPIO %d for CLK_PIN", m_cfg.gpio.clk);

    // // initialize some specific panel drivers
    // if (m_cfg.driver)
    //   shiftDriver(m_cfg);

// #if defined(SPIRAM_DMA_BUFFER)
//     // Trick library into dropping colour depth slightly when using PSRAM.
//     m_cfg.i2sspeed = HUB75_I2S_CFG::HZ_8M;
// #endif

    ESP_LOGI("begin()", "FPGA effective display resolution of width: %dpx height: %dpx.", m_cfg.mx_width * m_cfg.chain_length, m_cfg.mx_height);

	if (m_cfg.mx_height % 2 != 0) {
		ESP_LOGE("begin()", "Error: m_cfg.mx_height must be an even number!");
		return false;
	}

    // /* As DMA buffers are dynamically allocated, we must allocated in begin()
    //  * Ref: https://github.com/espressif/arduino-esp32/issues/831
    //  */
    // if (!setupDMA(m_cfg))
    // {
    //   return false;
    // } // couldn't even get the basic ram required.

    if (!initialized)
    {
      ESP_LOGE("being()", "MatrixPanel_FPGA_SPI::begin() failed!");
    }

    // Flush the DMA buffers prior to configuring DMA - Avoid visual artefacts on boot.
    // resetbuffers(); // Must fill the DMA buffer with the initial output bit sequence or the panel will display garbage
    // ESP_LOGV("being()", "Completed resetbuffers()");

	// flipDMABuffer(); // display back buffer 0, draw to 1, ignored if double buffering isn't enabled.
    // ESP_LOGV("being()", "Completed flipDMABuffer()");

	// Start output output'
    // AARON
	// dma_bus.init();
    // ESP_LOGV("being()", "Completed dma_bus.init()");

	// dma_bus.dma_transfer_start();
    // ESP_LOGV("being()", "Completed dma_bus.dma_transfer_start()");

    return initialized;
  }

  // Obj destructor
  virtual ~MatrixPanel_FPGA_SPI()
  {
    // dma_bus.release();
  }

  /*
   *  overload for compatibility
   */
//   bool begin(int r1, int g1 = G1_PIN_DEFAULT, int b1 = B1_PIN_DEFAULT, int r2 = R2_PIN_DEFAULT, int g2 = G2_PIN_DEFAULT, int b2 = B2_PIN_DEFAULT, int a = A_PIN_DEFAULT, int b = B_PIN_DEFAULT, int c = C_PIN_DEFAULT, int d = D_PIN_DEFAULT, int e = E_PIN_DEFAULT, int lat = LAT_PIN_DEFAULT, int oe = OE_PIN_DEFAULT, int clk = CLK_PIN_DEFAULT);
  bool begin(const FPGA_SPI_CFG &cfg);

  // Adafruit's BASIC DRAW API (565 colour format)
  // virtual void drawPixel(int16_t x, int16_t y, uint32_t color); // overwrite adafruit implementation
  // virtual void fillScreen(uint16_t color);                      // overwrite adafruit implementation

  /**
   * A wrapper to fill whatever selected DMA buffer / screen with black
   */
  void clearScreen();

// #ifndef NO_FAST_FUNCTIONS
//   /**
//    * @brief - override Adafruit's FastVLine
//    * this works faster than multiple consecutive pixel by pixel drawPixel() call
//    */
//   virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
//   {
//     uint8_t r, g, b;
//     color565to888(color, r, g, b);

//     int16_t w = 1;
//     transform(x, y, w, h);
//     if (h > w)
//       vlineDMA(x, y, h, r, g, b);
//     else
//       hlineDMA(x, y, w, r, g, b);

//   }
//   // rgb888 overload
//   virtual inline void drawFastVLine(int16_t x, int16_t y, int16_t h, uint8_t r, uint8_t g, uint8_t b)
//   {
//     int16_t w = 1;
//     transform(x, y, w, h);
//     if (h > w)
//       vlineDMA(x, y, h, r, g, b);
//     else
//       hlineDMA(x, y, w, r, g, b);
//   };

//   /**
//    * @brief - override Adafruit's FastHLine
//    * this works faster than multiple consecutive pixel by pixel drawPixel() call
//    */
//   virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
//   {
//     uint8_t r, g, b;
//     color565to888(color, r, g, b);

//     int16_t h = 1;
//     transform(x, y, w, h);
//     if (h > w)
//       vlineDMA(x, y, h, r, g, b);
//     else
//       hlineDMA(x, y, w, r, g, b);

//   }
//   // rgb888 overload
//   virtual inline void drawFastHLine(int16_t x, int16_t y, int16_t w, uint8_t r, uint8_t g, uint8_t b)
//   {
//     int16_t h = 1;
//     transform(x, y, w, h);
//     if (h > w)
//       vlineDMA(x, y, h, r, g, b);
//     else
//       hlineDMA(x, y, w, r, g, b);
//   };

//   /**
//    * @brief - override Adafruit's fillRect
//    * this works much faster than multiple consecutive per-pixel drawPixel() calls
//    */
//   virtual void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
//   {
//     uint8_t r, g, b;
//     color565to888(color, r, g, b);

//     transform(x, y, w, h);
//     fillRectDMA(x, y, w, h, r, g, b);

//   }
  // rgb888 overload
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r, uint8_t g, uint8_t b);
// #endif

  void fillScreenRGB888(uint8_t r, uint8_t g, uint8_t b);
  void drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b);

// #ifdef USE_GFX_LITE
//   // 24bpp FASTLED CRGB colour struct support
//   void fillScreen(CRGB color);
//   void drawPixel(int16_t x, int16_t y, CRGB color);
// #endif

// #ifdef NO_GFX
    inline int16_t width() const { return m_cfg.mx_width * m_cfg.chain_length; }
    inline int16_t height() const { return m_cfg.mx_height; }
// #endif

  // void drawIcon(int *ico, int16_t x, int16_t y, int16_t cols, int16_t rows);

//   // Colour 444 is a 4 bit scale, so 0 to 15, colour 565 takes a 0-255 bit value, so scale up by 255/15 (i.e. 17)!
//   static uint16_t color444(uint8_t r, uint8_t g, uint8_t b) { return color565(r * 17, g * 17, b * 17); }
//   static uint16_t color565(uint8_t r, uint8_t g, uint8_t b); // This is what is used by Adafruit GFX!

//   /**
//    * @brief - convert RGB565 to RGB888
//    * @param uint16_t colour - RGB565 input colour
//    * @param uint8_t &r, &g, &b - refs to variables where converted colors would be emplaced
//    */
//   static void color565to888(const uint16_t color, uint8_t &r, uint8_t &g, uint8_t &b);

//   inline void flipDMABuffer()
//   {
//     if (!m_cfg.double_buff)
//     {
//       return;
//     }

//     dma_bus.flip_dma_output_buffer(back_buffer_id);

// 	//back_buffer_id ^= 1;
// 	back_buffer_id = back_buffer_id^1;
//     fb = &frame_buffer[back_buffer_id];



//   }

  // /**
  //  * @param uint8_t b - 8-bit brightness value
  //  */
  // void setBrightness(const uint8_t b)
  // {
  //   if (!initialized)
  //   {
  //     ESP_LOGI("setBrightness()", "Tried to set output brightness before begin()");
  //     return;
  //   }

  //   brightness = b;
  //   setBrightnessOE(b, 0);

  //   // if (m_cfg.double_buff)
  //   // {
  //   //   setBrightnessOE(b, 1);
  //   // }
  // }

  // /**
  //  * @param uint8_t b - 8-bit brightness value
  //  */
  // void setPanelBrightness(const uint8_t b)
  // {
  //   setBrightness(b);
  // }

  /**
   * this is just a wrapper to control brightness
   * with an 8-bit value (0-255), very popular in FastLED-based sketches :)
   * @param uint8_t b - 8-bit brightness value
   */
  virtual void setBrightness8(const uint8_t b);
  // {
  //   setBrightness(b);
  //   // setPanelBrightness(b * PIXELS_PER_ROW / 256);
  // }

//   /**
//    * @brief - Sets how many clock cycles to blank OE before/after LAT signal change
//    * @param uint8_t pulses - clocks before/after OE
//    * default is DEFAULT_LAT_BLANKING
//    * Max is MAX_LAT_BLANKING
//    * @returns - new value for m_cfg.latch_blanking
//    */
//   uint8_t setLatBlanking(uint8_t pulses);

  /**
   * Get a class configuration struct
   *
   */
  const FPGA_SPI_CFG &getCfg() const { return m_cfg; };

  inline bool setCfg(const FPGA_SPI_CFG &cfg)
  {
    if (initialized)
      return false;

    m_cfg = cfg;
    PIXELS_PER_ROW = m_cfg.mx_width * m_cfg.chain_length;
    // ROWS_PER_FRAME = m_cfg.mx_height / MATRIX_ROWS_IN_PARALLEL;
    ROWS_PER_FRAME = m_cfg.mx_height;
    // MASK_OFFSET = 16 - m_cfg.getPixelColorDepthBits();

    config_set = true;
    return true;
  }

//   /**
//    * Stop the ESP32 DMA Engine. Screen will forever be black until next ESP reboot.
//    */
//   void stopDMAoutput()
//   {
//     resetbuffers();
//     // i2s_parallel_stop_dma(ESP32_I2S_DEVICE);
//     dma_bus.dma_transfer_stop();
//   }

  // ------- PROTECTED -------
  // those might be useful for child classes, like VirtualMatrixPanel
protected:
//   /**
//    * @brief - clears and reinitializes colour/control data in DMA buffs
//    * When allocated, DMA buffs might be dirty, so we need to blank it and initialize ABCDE,LAT,OE control bits.
//    * Those control bits are constants during the entire DMA sweep and never changed when updating just pixel colour data
//    * so we could set it once on DMA buffs initialization and forget.
//    * This effectively clears buffers to blank BLACK and makes it ready to display output.
//    * (Brightness control via OE bit manipulation is another case)
//    */
//   void clearFrameBuffer(bool _buff_id);

//   /* Update a specific pixel in the DMA buffer to a colour */
//   void updateMatrixDMABuffer(uint16_t x, uint16_t y, uint8_t red, uint8_t green, uint8_t blue);

//   /* Update the entire DMA buffer (aka. The RGB Panel) a certain colour (wipe the screen basically) */
//   void updateMatrixDMABuffer(uint8_t red, uint8_t green, uint8_t blue);

//   /**
//    * wipes DMA buffer(s) and reset all colour/service bits
//    */
//   inline void resetbuffers()
//   {
//     clearFrameBuffer(0);
//     setBrightnessOE(brightness, 0);

//     if (m_cfg.double_buff) {

//       clearFrameBuffer(1);
//       setBrightnessOE(brightness, 1);

//     }
//   }

// #ifndef NO_FAST_FUNCTIONS
//   /**
//    * @brief - update DMA buff drawing horizontal line at specified coordinates
//    * @param x_ccord - line start coordinate x
//    * @param y_ccord - line start coordinate y
//    * @param l - line length
//    * @param r,g,b, - RGB888 colour
//    */
//   void hlineDMA(int16_t x_coord, int16_t y_coord, int16_t l, uint8_t red, uint8_t green, uint8_t blue);

//   /**
//    * @brief - update DMA buff drawing horizontal line at specified coordinates
//    * @param x_ccord - line start coordinate x
//    * @param y_ccord - line start coordinate y
//    * @param l - line length
//    * @param r,g,b, - RGB888 colour
//    */
//   void vlineDMA(int16_t x_coord, int16_t y_coord, int16_t l, uint8_t red, uint8_t green, uint8_t blue);

//   /**
//    * @brief - update DMA buff drawing a rectangular at specified coordinates
//    * uses Fast H/V line draw internally, works faster than multiple consecutive pixel by pixel calls to updateMatrixDMABuffer()
//    * @param int16_t x, int16_t y - coordinates of a top-left corner
//    * @param int16_t w, int16_t h - width and height of a rectangular, min is 1 px
//    * @param uint8_t r - RGB888 colour
//    * @param uint8_t g - RGB888 colour
//    * @param uint8_t b - RGB888 colour
//    */
  // void fillRectDMA(int16_t x_coord, int16_t y_coord, int16_t w, int16_t h, uint8_t r, uint8_t g, uint8_t b);
// #endif

  // ------- PRIVATE -------
private:

//   /* Setup the DMA Link List chain and configure the ESP32 DMA + I2S or LCD peripheral */
//   bool setupDMA(const FPGA_SPI_CFG &opts);

//   /**
//    * pre-init procedures for specific drivers
//    *
//    */
//   void shiftDriver(const FPGA_SPI_CFG &opts);

//   /**
//    * @brief - FM6124-family chips initialization routine
//    */
//   void fm6124init(const FPGA_SPI_CFG &_cfg);

//   /**
//    * @brief - DP3246-family chips initialization routine
//    */
//   void dp3246init(const FPGA_SPI_CFG& _cfg);

//   /**
//    * @brief - reset OE bits in DMA buffer in a way to control brightness
//    * @param brt - brightness level from 0 to row_width
//    * @param _buff_id - buffer id to control
//    */
//   // void brtCtrlOE(int brt, const bool _buff_id=0);

  // /**
  //  * @brief - reset OE bits in DMA buffer in a way to control brightness
  //  * @param brt - brightness level from 0 to row_width
  //  * @param _buff_id - buffer id to control
  //  */
  // void setBrightnessOE(uint8_t brt, const int _buff_id = 0);

//   /**
//    * @brief - transforms coordinates according to orientation
//    * @param x - x position origin
//    * @param y - y position origin
//    * @param w - rectangular width
//    * @param h - rectangular height
//    */
//   void transform(int16_t &x, int16_t &y, int16_t &w, int16_t &h)
//   {
// #ifndef NO_GFX
//     int16_t t;
//     switch (rotation)
//     {
//     case 1:
//       t = _height - 1 - y - (h - 1);
//       y = x;
//       x = t;
//       t = h;
//       h = w;
//       w = t;
//       return;
//     case 2:
//       x = _width - 1 - x - (w - 1);
//       y = _height - 1 - y - (h - 1);
//       return;
//     case 3:
//       t = y;
//       y = _width - 1 - x - (w - 1);
//       x = t;
//       t = h;
//       h = w;
//       w = t;
//       return;
//     }
// #endif
//   };

public:
  /**
   * Contains the resulting refresh rate (scan rate) that will be achieved
   * based on the i2sspeed, colour depth and min_refresh_rate requested.
   */
  int calculated_refresh_rate = 0;

protected:
//   Bus_Parallel16 dma_bus;
  spi_device_handle_t spi_bus;

private:

  // Matrix i2s settings
  FPGA_SPI_CFG m_cfg;

  /* Pixel data is organized from LSB to MSB sequentially by row, from row 0 to row matrixHeight/matrixRowsInParallel
   * (two rows of pixels are refreshed in parallel)
   * Memory is allocated (malloc'd) by the row, and not in one massive chunk, for flexibility.
   * The whole DMA framebuffer is just a vector of pointers to structs with ESP32_I2S_DMA_STORAGE_TYPE arrays
   * Since it's dimensions is unknown prior to class initialization, we just declare it here as empty struct and will do all allocations later.
   * Refer to rowBitStruct to get the idea of it's internal structure
   */
  // frameStruct frame_buffer[2];
  // frameStruct *fb; // What framebuffer we are writing pixel changes to? (pointer to either frame_buffer[0] or frame_buffer[1] basically ) used within updateMatrixDMABuffer(...)

  // volatile int back_buffer_id = 0;      // If using double buffer, which one is NOT active (ie. being displayed) to write too?
  int brightness = 128;        // If you get ghosting... reduce brightness level. ((60/64)*255) seems to be the limit before ghosting on a 64 pixel wide physical panel for some panels.
  // int lsbMsbTransitionBit = 0; // For colour depth calculations

  /* ESP32-HUB75-MatrixPanel-I2S-DMA functioning constants
   * we should not those once object instance initialized it's DMA structs
   * they weree const, but this lead to bugs, when the default constructor was called.
   * So now they could be changed, but shouldn't. Maybe put a cpp lock around it, so it can't be changed after initialisation
   */
  uint16_t PIXELS_PER_ROW = m_cfg.mx_width * m_cfg.chain_length;      // number of pixels in a single row of all chained matrix modules (WIDTH of a combined matrix chain)
//   uint8_t ROWS_PER_FRAME = m_cfg.mx_height / MATRIX_ROWS_IN_PARALLEL; // RPF - rows per frame, either 16 or 32 depending on matrix module
  uint8_t ROWS_PER_FRAME = m_cfg.mx_height;
//   uint8_t MASK_OFFSET = 16 - m_cfg.getPixelColorDepthBits();

  // Other private variables
  bool initialized = false;
  bool config_set = false;

}; // end Class header

#endif
