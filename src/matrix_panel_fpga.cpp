#include "matrix_panel_fpga.hpp"
#include "driver/gpio.h"
#include <cstring>

void MatrixPanel_FPGA_SPI::swapFrame() {
    if (!initialized)
    {
      ESP_LOGI("drawRowRGB888()", "Tried to set output brightness before begin()");
      return;
    }
    uint8_t buf[1];
    uint16_t buf_len = 0;

    buf[buf_len++] = 't';                        // Command byte

    // Send each 8-bit chunk
    spi_transaction_t t = {
        .length = (size_t)(8*buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRowRGB888", "SPI transmit failed: %s", esp_err_to_name(err));
    }
};

void MatrixPanel_FPGA_SPI::fulfillWatchdog() {
    if (!initialized)
    {
      ESP_LOGI("fulfillWatchdog()", "Tried to fulfill watchdog before begin()");
      return;
    }
    uint8_t buf[9] = {'W', 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEB, 0xDA, 0xED}; // 'W' is the command
    uint16_t buf_len = 9;


    // Send each 8-bit chunk
    spi_transaction_t t = {
        .length = (size_t)(8*buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:fulfillWatchdog", "SPI transmit failed: %s", esp_err_to_name(err));
    }
};

void MatrixPanel_FPGA_SPI::drawFrameRGB888(uint8_t *data, size_t length) {
    // Currently fails due to spi transaction size
    if (!initialized)
    {
      ESP_LOGI("drawFrameRGB888()", "Tried to set output brightness before begin()");
      return;
    }
    size_t expected_row_bytes = (width() * height() * 3)
                                // + 1  // row
                                + 1  // command
                                ;
    if (data == nullptr) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawFrameRGB888", "Invalid data passed to drawFrameRGB888 nullptr! (length=%d)", length);
        return;
    }

    // uint8_t buf = 'Y';
    uint8_t *buf = (uint8_t *) heap_caps_malloc(1, MALLOC_CAP_DMA);
    buf[0] = 'Y';

    // Send each 8-bit chunk
    spi_transaction_t t = {
        .length = (size_t)(8), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawFrameRGB888", "SPI transmit failed: %s", esp_err_to_name(err));
    }
    heap_caps_free(buf);
    spi_transaction_t t2 = {
        .length = (size_t)(length*8), // bits
        .tx_buffer = data,
    };
    esp_err_t err2 = spi_device_transmit(spi_bus, &t2);
    if (err2 != ESP_OK) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawFrameRGB888", "SPI transmit failed: %s", esp_err_to_name(err2));
    }
};

void MatrixPanel_FPGA_SPI::drawRowRGB888(const uint8_t y, uint8_t *data, size_t length) {
    if (!initialized)
    {
      ESP_LOGI("drawRowRGB888()", "Tried to set output brightness before begin()");
      return;
    }
    size_t expected_row_bytes = (width() * 3)
                                + 1  // row
                                + 1  // command
                                ;
    if (data == nullptr) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRowRGB888", "Invalid data passed to drawRowRGB888 nullptr! (y=%d length=%d expected_row_bytes=%d)", y, length, expected_row_bytes);
        return;
    }

    uint8_t buf[expected_row_bytes];
    uint16_t buf_len = 0;

    buf[buf_len++] = 'L';                        // Command byte
    buf[buf_len++] = static_cast<uint8_t>(y);    // Y coordinate
    memcpy(&buf[2], data, length);
    buf_len = buf_len + length;

    // Send each 8-bit chunk
    spi_transaction_t t = {
        .length = (size_t)(8*buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRowRGB888", "SPI transmit failed: %s", esp_err_to_name(err));
    }
};

void MatrixPanel_FPGA_SPI::drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b) {
    if (!initialized)
    {
      ESP_LOGI("drawPixelRGB888()", "Tried to set output brightness before begin()");
      return;
    }
    uint8_t buf[7];
    uint8_t buf_len = 0;

    buf[buf_len++] = 'P';                        // Command byte
    buf[buf_len++] = static_cast<uint8_t>(y);    // Y coordinate

    if (PIXELS_PER_ROW <= 0xff) {
        buf[buf_len++] = static_cast<uint8_t>(x);              // X (1 byte)
    } else {
        buf[buf_len++] = static_cast<uint8_t>(x & 0xFF);       // X LSB
        buf[buf_len++] = static_cast<uint8_t>((x >> 8) & 0xFF); // X MSB
    }

    buf[buf_len++] = r; // Red
    buf[buf_len++] = g; // Green
    buf[buf_len++] = b; // Blue

    // Send each 8-bit chunk
    spi_transaction_t t = {
        .length = (size_t)(8*buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel", "SPI transmit failed: %s", esp_err_to_name(err));
    }
};

void MatrixPanel_FPGA_SPI::fillScreenRGB888(uint8_t r, uint8_t g, uint8_t b) {
    if (!initialized)
    {
      ESP_LOGI("fillScreenRGB888()", "Tried to set output brightness before begin()");
      return;
    }
    uint8_t buf[4];
    uint8_t buf_len = 0;

    buf[buf_len++] = 'F';                        // Command byte
    buf[buf_len++] = r; // Red
    buf[buf_len++] = g; // Green
    buf[buf_len++] = b; // Blue
    // Send each 8-bit chunk
    spi_transaction_t t = {
        .length = (size_t)(8*buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel", "SPI transmit failed: %s", esp_err_to_name(err));
    }
};
void MatrixPanel_FPGA_SPI::clearScreen() {
    if (!initialized)
    {
      ESP_LOGI("clearScreen()", "Tried to set output brightness before begin()");
      return;
    }
    uint8_t buf[1];
    uint8_t buf_len = 0;

    buf[buf_len++] = 'Z';                        // Command byte

    // Send each 8-bit chunk
    spi_transaction_t t = {
        .length = (size_t)(8*buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel", "SPI transmit failed: %s", esp_err_to_name(err));
    }
};

void MatrixPanel_FPGA_SPI::setBrightness8(const uint8_t b) {
    if (!initialized)
    {
      ESP_LOGI("setBrightness8()", "Tried to set output brightness before begin()");
      return;
    }
    uint8_t buf[2];
    uint8_t buf_len = 0;

    buf[buf_len++] = 'T';                        // Command byte

    buf[buf_len++] = b; // brightness

    // Send each 8-bit chunk
    spi_transaction_t t = {
        .length = (size_t)(8*buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel", "SPI transmit failed: %s", esp_err_to_name(err));
    }
};

void MatrixPanel_FPGA_SPI::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r, uint8_t g, uint8_t b) {
    if (!initialized)
    {
      ESP_LOGI("setBrightness8()", "Tried to set output brightness before begin()");
      return;
    }
    uint8_t buf[10];
    uint8_t buf_len = 0;
    // x1, y1, width, hieght, capture

    buf[buf_len++] = 'f';                        // Command byte
    if (PIXELS_PER_ROW <= 0xff) {
        buf[buf_len++] = static_cast<uint8_t>(x);              // X (1 byte)
    } else {
        buf[buf_len++] = static_cast<uint8_t>(x & 0xFF);       // X LSB
        buf[buf_len++] = static_cast<uint8_t>((x >> 8) & 0xFF); // X MSB
    }
    buf[buf_len++] = static_cast<uint8_t>(y);    // Y coordinate

    if (PIXELS_PER_ROW <= 0xff) {
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
    spi_transaction_t t = {
        .length = (size_t)(8*buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel", "SPI transmit failed: %s", esp_err_to_name(err));
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
        .flags = SPI_DEVICE_NO_DUMMY,
        .queue_size = 1,
    };
    gpio_reset_pin((gpio_num_t)cfg.gpio.mosi);
    gpio_reset_pin((gpio_num_t)cfg.gpio.clk);
    gpio_reset_pin((gpio_num_t)cfg.gpio.ce);
    spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(HSPI_HOST, &devcfg, &spi_bus);
    initialized = true;
}
