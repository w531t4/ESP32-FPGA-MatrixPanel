#ifndef _ESP32_RGB_64_32_MATRIX_PANEL_SPI_DMA
#define _ESP32_RGB_64_32_MATRIX_PANEL_SPI_DMA
#include <stdint.h>
#include "matrix_panel_fpga_config.hpp"
#include "driver/gpio.h"
#include "driver/spi_master.h"

/***************************************************************************************/
class MatrixPanel_FPGA_SPI
{


  // ------- PUBLIC -------
public:
  MatrixPanel_FPGA_SPI()
  {
  }
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
    ESP_LOGI("begin()", "FPGA effective display resolution of width: %dpx height: %dpx.", m_cfg.mx_width * m_cfg.chain_length, m_cfg.mx_height);

	if (m_cfg.mx_height % 2 != 0) {
		ESP_LOGE("begin()", "Error: m_cfg.mx_height must be an even number!");
		return false;
	}

    init_spi(m_cfg);

    // Reset the fpga state
    gpio_set_direction((gpio_num_t) m_cfg.gpio.fpga_reset, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t) m_cfg.gpio.fpga_reset, 0);  // LOW
    gpio_set_level((gpio_num_t) m_cfg.gpio.fpga_reset, 1);  // HIGH
    gpio_set_level((gpio_num_t) m_cfg.gpio.fpga_reset, 0);  // LOW

    while (!initialized);
    if (!initialized)
    {
      ESP_LOGE("being()", "MatrixPanel_FPGA_SPI::begin() failed!");
    }
    return initialized;
  }

  // Obj destructor
  virtual ~MatrixPanel_FPGA_SPI()
  {
    // dma_bus.release();
  }

  bool begin(const FPGA_SPI_CFG &cfg);
  void clearScreen();

  // rgb888 overload
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r, uint8_t g, uint8_t b);
  void fillScreenRGB888(uint8_t r, uint8_t g, uint8_t b);
  void drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b);
  void drawRowRGB888(const uint8_t y, uint8_t *data, size_t length);
  void drawFrameRGB888(uint8_t *data, size_t length);
  void swapFrame();
    inline int16_t width() const { return m_cfg.mx_width * m_cfg.chain_length; }
    inline int16_t height() const { return m_cfg.mx_height; }
  virtual void setBrightness8(const uint8_t b);
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

protected:
private:

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

  int brightness = 128;        // If you get ghosting... reduce brightness level. ((60/64)*255) seems to be the limit before ghosting on a 64 pixel wide physical panel for some panels.

  uint16_t PIXELS_PER_ROW = m_cfg.mx_width * m_cfg.chain_length;      // number of pixels in a single row of all chained matrix modules (WIDTH of a combined matrix chain)
  uint8_t ROWS_PER_FRAME = m_cfg.mx_height;

  // Other private variables
  bool initialized = false;
  bool config_set = false;

};

#endif
