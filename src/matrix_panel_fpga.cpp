// SPDX-FileCopyrightText: 2025 Aaron White <w531t4@gmail.com>
// SPDX-License-Identifier: MIT
#include "matrix_panel_fpga.hpp"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <algorithm>
#include <vector>
#include <cstring>

void IRAM_ATTR MatrixPanel_FPGA_SPI::fpga_resetstatus_isr_(void *arg) {
    auto *self = static_cast<MatrixPanel_FPGA_SPI *>(arg);
    self->fpga_reset_seen_ = true;
}

bool MatrixPanel_FPGA_SPI::lock_spi_() {
    if (spi_mutex_ == nullptr)
        return true;
    if (xSemaphoreTake(spi_mutex_, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE("MatrixPanel", "SPI lock failed");
        return false;
    }
    return true;
}

void MatrixPanel_FPGA_SPI::unlock_spi_() {
    if (spi_mutex_ == nullptr)
        return;
    xSemaphoreGive(spi_mutex_);
}

void MatrixPanel_FPGA_SPI::do_swapFrame_() {
    if (!initialized) {
        ESP_LOGI("drawRowRGB888()",
                 "Tried to set output brightness before begin()");
        return;
    }
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
    if (!wait_for_fpga_resetstatus_())
        return;
    uint8_t buf[1];
    uint16_t buf_len = 0;

    buf[buf_len++] = 't'; // Command
                          // byte

    // Send each 8-bit
    // chunk
    spi_transaction_t t = {
        .length = (size_t)(8 * buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRowRGB888",
                 "SPI transmit failed: %s", esp_err_to_name(err));
    }
    wait_for_fpga_busy_clear_();
};

void MatrixPanel_FPGA_SPI::swapFrame() {
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return; // drop until worker is ready
        Job j;
        j.op = Op::SWAP;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_swapFrame_();
}

void MatrixPanel_FPGA_SPI::do_fulfillWatchdog_() {
    if (!initialized) {
        ESP_LOGI("fulfillWatchdog()",
                 "Tried to fulfill watchdog before begin()");
        return;
    }
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
    if (!wait_for_fpga_resetstatus_())
        return;
    uint8_t buf[9] = {'W',  0xDE, 0xAD, 0xBE, 0xEF,
                      0xFE, 0xEB, 0xDA, 0xED}; // 'W' is
                                               // the
                                               // command
    uint16_t buf_len = 9;

    // Send each 8-bit
    // chunk
    spi_transaction_t t = {
        .length = (size_t)(8 * buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:fulfillWatchdog",
                 "SPI transmit failed: %s", esp_err_to_name(err));
    }
    wait_for_fpga_busy_clear_();
};

void MatrixPanel_FPGA_SPI::fulfillWatchdog() {
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return; // drop until worker is ready
        Job j;
        j.op = Op::WATCHDOG;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_fulfillWatchdog_();
}

void MatrixPanel_FPGA_SPI::init_fpga_resetstatus_gpio_() {
    if (m_cfg.gpio.fpga_resetstatus < 0)
        return;
    fpga_resetstatus_configured_ = true;
    gpio_reset_pin((gpio_num_t)m_cfg.gpio.fpga_resetstatus);
    gpio_set_direction((gpio_num_t)m_cfg.gpio.fpga_resetstatus, GPIO_MODE_INPUT);

    static bool isr_service_installed = false;
    if (!isr_service_installed) {
        esp_err_t err = gpio_install_isr_service(0);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
            isr_service_installed = true;
        } else {
            ESP_LOGE("fpga_resetstatus", "ISR service install failed: %s",
                     esp_err_to_name(err));
        }
    }
    if (isr_service_installed) {
        gpio_set_intr_type((gpio_num_t)m_cfg.gpio.fpga_resetstatus,
                           GPIO_INTR_NEGEDGE);
        esp_err_t err = gpio_isr_handler_add(
            (gpio_num_t)m_cfg.gpio.fpga_resetstatus,
            &MatrixPanel_FPGA_SPI::fpga_resetstatus_isr_, this);
        if (err != ESP_OK) {
            ESP_LOGE("fpga_resetstatus", "ISR attach failed: %s",
                     esp_err_to_name(err));
        }
    }
    if (gpio_get_level((gpio_num_t)m_cfg.gpio.fpga_resetstatus) == 0) {
        fpga_reset_seen_ = true;
    }
}

void MatrixPanel_FPGA_SPI::init_fpga_busy_gpio_() {
    if (m_cfg.gpio.fpga_busy < 0)
        return;
    fpga_busy_configured_ = true;
    gpio_reset_pin((gpio_num_t)m_cfg.gpio.fpga_busy);
    gpio_set_direction((gpio_num_t)m_cfg.gpio.fpga_busy, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)m_cfg.gpio.fpga_busy, GPIO_FLOATING);
}

bool MatrixPanel_FPGA_SPI::wait_for_fpga_resetstatus_() {
    if (!fpga_resetstatus_configured_)
        return true;
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout =
        pdMS_TO_TICKS(m_cfg.fpga_resetstatus_timeout_ms);
    while (gpio_get_level((gpio_num_t)m_cfg.gpio.fpga_resetstatus) == 0) {
        if ((xTaskGetTickCount() - start) > timeout) {
            ESP_LOGW("fpga_resetstatus", "Timeout waiting for FPGA resetstatus");
            return false;
        }
        vTaskDelay(1);
    }
    return true;
}

bool MatrixPanel_FPGA_SPI::wait_for_fpga_busy_clear_() {
    if (!fpga_busy_configured_)
        return true;
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(m_cfg.fpga_busy_timeout_ms);
    while (gpio_get_level((gpio_num_t)m_cfg.gpio.fpga_busy) != 0) {
        if ((xTaskGetTickCount() - start) > timeout) {
            ESP_LOGW("fpga_busy", "Timeout waiting for FPGA busy clear");
            return false;
        }
        vTaskDelay(1);
    }
    return true;
}

bool MatrixPanel_FPGA_SPI::consume_fpga_reset() {
    if (!fpga_resetstatus_configured_)
        return false;
    if (!fpga_reset_seen_)
        return false;
    if (gpio_get_level((gpio_num_t)m_cfg.gpio.fpga_resetstatus) == 0)
        return false;
    fpga_reset_seen_ = false;
    reset_epoch_++;
    return true;
}

void MatrixPanel_FPGA_SPI::resync_after_fpga_reset(uint8_t brightness) {
    if (!initialized) {
        ESP_LOGI("resync_after_fpga_reset()",
                 "Tried to resync before begin()");
        return;
    }
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return;
        xQueueReset(tx_q_);
        Job j;
        j.op = Op::CLEAR;
        (void)xQueueSend(tx_q_, &j, 0);
        j.op = Op::SET_BRIGHTNESS;
        j.u8 = brightness;
        (void)xQueueSend(tx_q_, &j, 0);
        j.op = Op::SWAP;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_clearScreen_();
    do_setBrightness8_(brightness);
    do_swapFrame_();
}

void MatrixPanel_FPGA_SPI::do_drawFrameRGB888_(const uint8_t *data,
                                               size_t length) {
    // Currently fails
    // due to spi
    // transaction size
    if (!initialized) {
        ESP_LOGI("drawFrameRGB888()",
                 "Tried to set output brightness before begin()");
        return;
    }
    size_t expected_row_bytes = (width() * height() * 3)
                                // + 1  //
                                // row
                                + 1 // command
        ;
    if (data == nullptr) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawFrameRGB888",
                 "Invalid data passed to drawFrameRGB888 nullptr! (length=%d)",
                 length);
        return;
    }
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
    if (!wait_for_fpga_resetstatus_())
        return;

    const size_t chunk_bytes =
        std::min(static_cast<size_t>(SPI_MAX_DMA_LEN), length);
    uint8_t *buf =
        static_cast<uint8_t *>(heap_caps_malloc(chunk_bytes, MALLOC_CAP_DMA));
    if (buf == nullptr) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawFrameRGB888",
                 "DMA alloc failed for frame chunk (%u bytes)",
                 static_cast<unsigned>(chunk_bytes));
        return;
    }

    buf[0] = 'Y';
    spi_transaction_t t = {
        .length = (size_t)(8), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawFrameRGB888",
                 "SPI transmit failed: %s", esp_err_to_name(err));
        heap_caps_free(buf);
        return;
    }

    size_t offset = 0;
    while (offset < length) {
        const size_t chunk = std::min(length - offset, chunk_bytes);
        memcpy(buf, data + offset, chunk);
        spi_transaction_t t2 = {
            .length = static_cast<size_t>(chunk * 8), // bits
            .tx_buffer = buf,
        };
        esp_err_t err2 = spi_device_transmit(spi_bus, &t2);
        if (err2 != ESP_OK) {
            ESP_LOGE("MatrixPanel_FPGA_SPI:drawFrameRGB888",
                     "SPI transmit failed: %s", esp_err_to_name(err2));
            break;
        }
        offset += chunk;
    }

    heap_caps_free(buf);
    wait_for_fpga_busy_clear_();
};

void MatrixPanel_FPGA_SPI::drawFrameRGB888(const uint8_t *data, size_t length) {
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return; // drop until worker is ready
        Job j;
        j.op = Op::DRAW_FRAME;
        j.data = data;
        j.length = length;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_drawFrameRGB888_(data, length);
}

void MatrixPanel_FPGA_SPI::do_drawRowRGB888_(const uint8_t y,
                                             const uint8_t *data,
                                             size_t length) {
    if (!initialized) {
        ESP_LOGI("drawRowRGB888()",
                 "Tried to set output brightness before begin()");
        return;
    }
    size_t expected_row_bytes = (width() * 3) + 1 // row
                                + 1               // command
        ;
    if (data == nullptr) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRowRGB888",
                 "Invalid data passed to drawRowRGB888 nullptr! (y=%d "
                 "length=%d expected_row_bytes=%d)",
                 y, length, expected_row_bytes);
        return;
    }
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
    if (!wait_for_fpga_resetstatus_())
        return;

    uint8_t buf[expected_row_bytes];
    uint16_t buf_len = 0;

    buf[buf_len++] = 'L';                     // Command
                                              // byte
    buf[buf_len++] = static_cast<uint8_t>(y); // Y
                                              // coordinate
    memcpy(&buf[2], data, length);
    buf_len = buf_len + length;

    // Send each 8-bit
    // chunk
    spi_transaction_t t = {
        .length = (size_t)(8 * buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRowRGB888",
                 "SPI transmit failed: %s", esp_err_to_name(err));
    }
    wait_for_fpga_busy_clear_();
};

void MatrixPanel_FPGA_SPI::drawRowRGB888(const uint8_t y, const uint8_t *data,
                                         size_t length) {
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return;
        Job j;
        j.op = Op::DRAW_ROW;
        j.y = y;
        j.data = data;
        j.length = length;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_drawRowRGB888_(y, data, length);
}

bool MatrixPanel_FPGA_SPI::queue_has_space(size_t slots) const {
    if (!use_worker_ || !tx_q_)
        return true;
    return uxQueueSpacesAvailable(tx_q_) >= slots;
}

bool MatrixPanel_FPGA_SPI::worker_is_idle() const {
    if (!use_worker_ || !tx_q_)
        return true;
    return uxQueueMessagesWaiting(tx_q_) == 0 && !worker_busy_;
}

void MatrixPanel_FPGA_SPI::do_drawPixelRGB888_(int16_t x, int16_t y, uint8_t r,
                                               uint8_t g, uint8_t b) {
    if (!initialized) {
        ESP_LOGI("drawPixelRGB888()",
                 "Tried to set output brightness before begin()");
        return;
    }
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
    if (!wait_for_fpga_resetstatus_())
        return;
    uint8_t buf[7];
    uint8_t buf_len = 0;

    buf[buf_len++] = 'P';                     // Command
                                              // byte
    buf[buf_len++] = static_cast<uint8_t>(y); // Y
                                              // coordinate

    if (PIXELS_PER_ROW <= 0xff) {
        buf[buf_len++] = static_cast<uint8_t>(x); // X (1 byte)
    } else {
        buf[buf_len++] = static_cast<uint8_t>((x >> 8) & 0xFF); // X MSB
        buf[buf_len++] = static_cast<uint8_t>(x & 0xFF);        // X LSB
    }

    buf[buf_len++] = r; // Red
    buf[buf_len++] = g; // Green
    buf[buf_len++] = b; // Blue

    // Send each 8-bit
    // chunk
    spi_transaction_t t = {
        .length = (size_t)(8 * buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel", "SPI transmit failed: %s",
                 esp_err_to_name(err));
    }
    wait_for_fpga_busy_clear_();
};

void MatrixPanel_FPGA_SPI::drawPixelRGB888(int16_t x, int16_t y, uint8_t r,
                                           uint8_t g, uint8_t b) {
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return;
        Job j;
        j.op = Op::DRAW_PIXEL;
        j.x = x;
        j.y = (uint8_t)y;
        j.r = r;
        j.g = g;
        j.b = b;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_drawPixelRGB888_(x, y, r, g, b);
}

void MatrixPanel_FPGA_SPI::do_fillScreenRGB888_(uint8_t r, uint8_t g,
                                                uint8_t b) {
    if (!initialized) {
        ESP_LOGI("fillScreenRGB888()",
                 "Tried to set output brightness before begin()");
        return;
    }
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
    if (!wait_for_fpga_resetstatus_())
        return;
    uint8_t buf[4];
    uint8_t buf_len = 0;

    buf[buf_len++] = 'F'; // Command
                          // byte
    buf[buf_len++] = r;   // Red
    buf[buf_len++] = g;   // Green
    buf[buf_len++] = b;   // Blue
    // Send each 8-bit
    // chunk
    spi_transaction_t t = {
        .length = (size_t)(8 * buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel", "SPI transmit failed: %s",
                 esp_err_to_name(err));
    }
    wait_for_fpga_busy_clear_();
};

void MatrixPanel_FPGA_SPI::fillScreenRGB888(uint8_t r, uint8_t g, uint8_t b) {
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return;
        Job j;
        j.op = Op::FILL_SCREEN;
        j.r = r;
        j.g = g;
        j.b = b;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_fillScreenRGB888_(r, g, b);
}

void MatrixPanel_FPGA_SPI::do_copyFrame_() {
    if (!initialized) {
        ESP_LOGI("copyFrame()",
                 "Tried to execute command before begin()");
        return;
    }
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
        return;
    if (!wait_for_fpga_resetstatus_())
        return;
    uint8_t buf[1];
    uint8_t buf_len = 0;

    buf[buf_len++] = 'C'; // Command
                          // byte

    // Send each 8-bit
    // chunk
    spi_transaction_t t = {
        .length = (size_t)(8 * buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel", "SPI transmit failed: %s",
                 esp_err_to_name(err));
    }
    wait_for_fpga_busy_clear_();
};

void MatrixPanel_FPGA_SPI::copyFrame() {
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return;
        Job j;
        j.op = Op::COPY_FRAME;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_copyFrame_();
}

void MatrixPanel_FPGA_SPI::do_clearScreen_() {
    if (!initialized) {
        ESP_LOGI("clearScreen()",
                 "Tried to set output brightness before begin()");
        return;
    }
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
    if (!wait_for_fpga_resetstatus_())
        return;
    uint8_t buf[1];
    uint8_t buf_len = 0;

    buf[buf_len++] = 'Z'; // Command
                          // byte

    // Send each 8-bit
    // chunk
    spi_transaction_t t = {
        .length = (size_t)(8 * buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel", "SPI transmit failed: %s",
                 esp_err_to_name(err));
    }
    wait_for_fpga_busy_clear_();
};

void MatrixPanel_FPGA_SPI::clearScreen() {
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return;
        Job j;
        j.op = Op::CLEAR;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_clearScreen_();
}

void MatrixPanel_FPGA_SPI::do_setBrightness8_(const uint8_t b) {
    if (!initialized) {
        ESP_LOGI("setBrightness8()",
                 "Tried to set output brightness before begin()");
        return;
    }
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
    if (!wait_for_fpga_resetstatus_())
        return;
    uint8_t buf[2];
    uint8_t buf_len = 0;

    buf[buf_len++] = 'T'; // Command
                          // byte

    buf[buf_len++] = b; // brightness

    // Send each 8-bit
    // chunk
    spi_transaction_t t = {
        .length = (size_t)(8 * buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel", "SPI transmit failed: %s",
                 esp_err_to_name(err));
    }
    wait_for_fpga_busy_clear_();
};

void MatrixPanel_FPGA_SPI::setBrightness8(const uint8_t b) {
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return;
        Job j;
        j.op = Op::SET_BRIGHTNESS;
        j.u8 = b;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_setBrightness8_(b);
}

void MatrixPanel_FPGA_SPI::do_fillRect_(int16_t x, int16_t y, int16_t w,
                                        int16_t h, uint8_t r, uint8_t g,
                                        uint8_t b) {
    if (!initialized) {
        ESP_LOGI("setBrightness8()",
                 "Tried to set output brightness before begin()");
        return;
    }
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
    if (!wait_for_fpga_resetstatus_())
        return;
    uint8_t buf[10];
    uint8_t buf_len = 0;
    // x1, y1, width,
    // hieght, capture

    buf[buf_len++] = 'f'; // Command
                          // byte
    if (PIXELS_PER_ROW <= 0xff) {
        buf[buf_len++] = static_cast<uint8_t>(x); // X (1 byte)
    } else {
        buf[buf_len++] = static_cast<uint8_t>((x >> 8) & 0xFF); // X MSB
        buf[buf_len++] = static_cast<uint8_t>(x & 0xFF);        // X LSB
    }
    buf[buf_len++] = static_cast<uint8_t>(y); // Y
                                              // coordinate

    if (PIXELS_PER_ROW <= 0xff) {
        buf[buf_len++] = static_cast<uint8_t>(w); // X (1 byte)
    } else {
        buf[buf_len++] = static_cast<uint8_t>((w >> 8) & 0xFF); // X MSB
        buf[buf_len++] = static_cast<uint8_t>(w & 0xFF);        // X LSB
    }
    buf[buf_len++] = static_cast<uint8_t>(h); // Y
                                              // coordinate

    buf[buf_len++] = r; // Red
    buf[buf_len++] = g; // Green
    buf[buf_len++] = b; // Blue

    // Send each 8-bit
    // chunk
    spi_transaction_t t = {
        .length = (size_t)(8 * buf_len), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel", "SPI transmit failed: %s",
                 esp_err_to_name(err));
    }
    wait_for_fpga_busy_clear_();
};

void MatrixPanel_FPGA_SPI::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                    uint8_t r, uint8_t g, uint8_t b) {
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return;
        Job j;
        j.op = Op::FILL_RECT;
        j.x = x;
        j.y = (uint8_t)y;
        j.w = w;
        j.h = h;
        j.r = r;
        j.g = g;
        j.b = b;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_fillRect_(x, y, w, h, r, g, b);
}

void MatrixPanel_FPGA_SPI::run_test_graphic(uint32_t delay_ms) {
    if (!initialized) {
        ESP_LOGW("MatrixPanel_FPGA_SPI",
                 "run_test_graphic() called before begin()");
        return;
    }

    const TickType_t delay_ticks =
        pdMS_TO_TICKS(delay_ms > 0 ? delay_ms : 0);
    const auto delay_if_needed = [&]() {
        if (delay_ticks > 0) {
            vTaskDelay(delay_ticks);
        }
    };
    const auto wait_for_worker_idle = [&]() {
        if (!use_worker_)
            return;
        while (!worker_is_idle()) {
            vTaskDelay(1);
        }
    };

    // NOTE: Remember origin (0,0) is in upper left hand corner of display
    // Summary: expect the following visible elements in order:
    // - neutral grey background
    // - red top band drawn through drawRowRGB888
    // - green bottom band via fillRect
    // - blue center band via fillRect
    // - yellow left column, magenta right column for edge accents
    // - opposing diagonals (orange/white) to cover drawPixelRGB888
    // - brightness pushed to 255 and the frame swapped to display the pattern
    clearScreen();
    delay_if_needed();
    // Fill a neutral grey so the bright accents stand out.
    fillScreenRGB888(0x18, 0x18, 0x18);
    delay_if_needed();

    const int width = this->width();
    const int height = this->height();
    const int horizontal_band_height =
        std::min(height, std::max(1, height / 12));
    const int vertical_bar_width =
        std::min(width, std::max(1, width / 16));

    const size_t row_bytes = static_cast<size_t>(width) * 3;
    std::vector<uint8_t> red_row(row_bytes);
    // Prepare a solid red row buffer for drawRowRGB888.
    for (int x = 0; x < width; ++x) {
        const size_t idx = static_cast<size_t>(x) * 3;
        red_row[idx] = 0xFF;
        red_row[idx + 1] = 0x00;
        red_row[idx + 2] = 0x00;
    }
    // Draw the red top band with drawRowRGB888 so this code path is exercised.
    for (int y = 0; y < horizontal_band_height; ++y) {
        drawRowRGB888(static_cast<uint8_t>(y), red_row.data(), row_bytes);
        delay_if_needed();
    }

    // Bottom band: green stripe using fillRect.
    fillRect(0, height - horizontal_band_height, width, horizontal_band_height,
             0x00, 0xFF, 0x00);
    delay_if_needed();

    // Middle band: blue stripe centered vertically.
    const int middle_y =
        std::max(0, (height / 2) - (horizontal_band_height / 2));
    fillRect(0, middle_y, width, horizontal_band_height, 0x00, 0x00, 0xFF);
    delay_if_needed();

    // Left accent column: yellow.
    fillRect(0, 0, vertical_bar_width, height, 0xFF, 0xFF, 0x33);
    wait_for_worker_idle();
    delay_if_needed();

    // Right accent column: magenta/pink.
    fillRect(std::max(width - vertical_bar_width, 0), 0, vertical_bar_width, height,
             0xFF, 0x00, 0xFF);
    wait_for_worker_idle();
    delay_if_needed();

    // Diagonals: orange from top-left and white from top-right.
    const int diag_length = std::min(width, height);
    const int diag_step = std::max(1, diag_length / 16);
    for (int i = 0; i < diag_length; i += diag_step) {
        drawPixelRGB888(i, i, 0xFF, 0x80, 0x00);
        delay_if_needed();
        drawPixelRGB888(width - 1 - i, i, 0xFF, 0xFF, 0xFF);
        delay_if_needed();
    }

    // Force full brightness so the bands are vivid.
    setBrightness8(255);
    delay_if_needed();

    // Swap the frame so the constructed graphic becomes visible.
    swapFrame();
    delay_if_needed();
}

void MatrixPanel_FPGA_SPI::init_spi(const FPGA_SPI_CFG &cfg) {
    ESP_LOGD("spi_init", "using core=%d", xPortGetCoreID());
    spi_bus_config_t buscfg = {
        .mosi_io_num = (gpio_num_t)cfg.gpio.mosi,
        .miso_io_num = -1, // Not
                           // used
        .sclk_io_num = (gpio_num_t)cfg.gpio.clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .mode = 0,                      // SPI
                                        // mode
                                        // 0
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
    if (!spi_mutex_) {
        spi_mutex_ = xSemaphoreCreateMutex();
        if (!spi_mutex_) {
            ESP_LOGE("spi_init", "Failed to create SPI mutex");
        }
    }
    initialized = true;
    ESP_LOGD("spi_init", "done");
}
