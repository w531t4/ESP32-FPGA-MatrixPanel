#include "matrix_panel_fpga.hpp"
#include "driver/gpio.h"

void MatrixPanel_FPGA_SPI::drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b) {
    uint8_t buf[7];
    uint8_t buf_len = 0;

    buf[buf_len++] = 'P';                        // Command byte
    buf[buf_len++] = static_cast<uint8_t>(y);    // Y coordinate

    if (x <= 0xFF) {
        buf[buf_len++] = static_cast<uint8_t>(x);              // X (1 byte)
    } else {
        buf[buf_len++] = static_cast<uint8_t>(x & 0xFF);       // X LSB
        buf[buf_len++] = static_cast<uint8_t>((x >> 8) & 0xFF); // X MSB
    }

    buf[buf_len++] = r; // Red
    buf[buf_len++] = g; // Green
    buf[buf_len++] = b; // Blue

    // Send each 8-bit chunk
    for (int i = 0; i < buf_len; i++) {
      spi_transaction_t t = {
          .length = 8, // bits
          .tx_buffer = &buf[i],
      };
      spi_device_transmit(spi_bus, &t); // blocking
    }

};

void MatrixPanel_FPGA_SPI::fillScreenRGB888(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t buf[4];
    uint8_t buf_len = 0;

    buf[buf_len++] = 'F';                        // Command byte
    buf[buf_len++] = r; // Red
    buf[buf_len++] = g; // Green
    buf[buf_len++] = b; // Blue
    // Send each 8-bit chunk
    for (int i = 0; i < buf_len; i++) {
      spi_transaction_t t = {
          .length = 8, // bits
          .tx_buffer = &buf[i],
      };
      spi_device_transmit(spi_bus, &t); // blocking
    }
};
void MatrixPanel_FPGA_SPI::clearScreen() {
    uint8_t buf[1];
    uint8_t buf_len = 0;

    buf[buf_len++] = 'Z';                        // Command byte

    // Send each 8-bit chunk
    for (int i = 0; i < buf_len; i++) {
      spi_transaction_t t = {
          .length = 8, // bits
          .tx_buffer = &buf[i],
      };
      spi_device_transmit(spi_bus, &t); // blocking
    }
};

void MatrixPanel_FPGA_SPI::setBrightness8(const uint8_t b) {
    uint8_t buf[2];
    uint8_t buf_len = 0;

    buf[buf_len++] = 'T';                        // Command byte

    buf[buf_len++] = b; // brightness

    // Send each 8-bit chunk
    for (int i = 0; i < buf_len; i++) {
      spi_transaction_t t = {
          .length = 8, // bits
          .tx_buffer = &buf[i],
      };
      spi_device_transmit(spi_bus, &t); // blocking
    }
};

void MatrixPanel_FPGA_SPI::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r, uint8_t g, uint8_t b) {
    uint8_t buf[4];
    uint8_t buf_len = 0;
    // x1, y1, width, hieght, capture

    buf[buf_len++] = 'f';                        // Command byte
    if (x <= 0xFF) {
        buf[buf_len++] = static_cast<uint8_t>(x);              // X (1 byte)
    } else {
        buf[buf_len++] = static_cast<uint8_t>(x & 0xFF);       // X LSB
        buf[buf_len++] = static_cast<uint8_t>((x >> 8) & 0xFF); // X MSB
    }
    buf[buf_len++] = static_cast<uint8_t>(y);    // Y coordinate

    if (w <= 0xFF) {
        buf[buf_len++] = static_cast<uint8_t>(w);              // X (1 byte)
    } else {
        buf[buf_len++] = static_cast<uint8_t>(w & 0xFF);       // X LSB
        buf[buf_len++] = static_cast<uint8_t>((w >> 8) & 0xFF); // X MSB
    }
    buf[buf_len++] = static_cast<uint8_t>(h);    // Y coordinate

    buf[buf_len++] = r; // Red
    buf[buf_len++] = g; // Green
    buf[buf_len++] = b; // Blue

    // Send each 8-bit chunk
      for (int i = 0; i < buf_len; i++) {
          spi_transaction_t t = {
              .length = 8, // bits
              .tx_buffer = &buf[i],
          };
          spi_device_transmit(spi_bus, &t); // blocking
      }
};

void MatrixPanel_FPGA_SPI::init_spi(const FPGA_SPI_CFG &cfg) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = (gpio_num_t)cfg.gpio.mosi,
        .miso_io_num = -1, // Not used
        .sclk_io_num = (gpio_num_t)cfg.gpio.clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .mode = 0,                          // SPI mode 0
        .clock_speed_hz = cfg.spispeed, // 10 MHz

        .spics_io_num = (gpio_num_t)cfg.gpio.ce,
        .queue_size = 1,
    };
    gpio_reset_pin((gpio_num_t)cfg.gpio.mosi);
    gpio_reset_pin((gpio_num_t)cfg.gpio.clk);
    gpio_reset_pin((gpio_num_t)cfg.gpio.ce);
    spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(HSPI_HOST, &devcfg, &spi_bus);
}
