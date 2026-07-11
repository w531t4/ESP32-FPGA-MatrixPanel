// SPDX-FileCopyrightText: 2025 Aaron White <w531t4@gmail.com>
// SPDX-License-Identifier: MIT
#include "matrix_panel_fpga.hpp"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <algorithm>
#include <cstddef>
#include <cstring>
#include <vector>

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
        return;
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
        return;
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
    if (!fpga_reset_seen_.exchange(false))
        return false;
    reset_epoch_++;
    // FPGA reset restarts the status mailbox at seq 0; forget the old seq so
    // the next readStatus doesn't reject a genuinely fresh frame as stale.
    have_last_seq_ = false;
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
        return;
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

void MatrixPanel_FPGA_SPI::do_drawRectRGB888_(int16_t x, int16_t y, int16_t w,
                                              int16_t h, const uint8_t *data,
                                              size_t length) {
    if (!initialized) {
        ESP_LOGI("drawRectRGB888()",
                 "Tried to set output brightness before begin()");
        return;
    }
    if (data == nullptr) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRectRGB888",
                 "Invalid data passed to drawRectRGB888 nullptr! (length=%d)",
                 length);
        return;
    }
    if (x < 0 || y < 0 || w <= 0 || h <= 0) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRectRGB888",
                 "Invalid rect params x=%d y=%d w=%d h=%d", x, y, w, h);
        return;
    }
    const size_t expected_len = static_cast<size_t>(w) *
                                static_cast<size_t>(h) * 3;
    if (length != expected_len) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRectRGB888",
                 "Invalid data length=%d expected=%d", length, expected_len);
        return;
    }
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
        return;
    if (!wait_for_fpga_resetstatus_())
        return;

    uint8_t header[7];
    uint8_t header_len = 0;
    header[header_len++] = 'X'; // Command byte
    if (PIXELS_PER_ROW <= 0xff) {
        header[header_len++] = static_cast<uint8_t>(x);
    } else {
        header[header_len++] = static_cast<uint8_t>((x >> 8) & 0xFF);
        header[header_len++] = static_cast<uint8_t>(x & 0xFF);
    }
    header[header_len++] = static_cast<uint8_t>(y);
    if (PIXELS_PER_ROW <= 0xff) {
        header[header_len++] = static_cast<uint8_t>(w);
    } else {
        header[header_len++] = static_cast<uint8_t>((w >> 8) & 0xFF);
        header[header_len++] = static_cast<uint8_t>(w & 0xFF);
    }
    header[header_len++] = static_cast<uint8_t>(h);

    spi_transaction_t t = {
        .length = (size_t)(8 * header_len), // bits
        .tx_buffer = header,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRectRGB888",
                 "SPI transmit failed: %s", esp_err_to_name(err));
        return;
    }

    const size_t chunk_bytes =
        std::min(static_cast<size_t>(SPI_MAX_DMA_LEN), length);
    uint8_t *buf =
        static_cast<uint8_t *>(heap_caps_malloc(chunk_bytes, MALLOC_CAP_DMA));
    if (buf == nullptr) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRectRGB888",
                 "DMA alloc failed for rect chunk (%u bytes)",
                 static_cast<unsigned>(chunk_bytes));
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
            ESP_LOGE("MatrixPanel_FPGA_SPI:drawRectRGB888",
                     "SPI transmit failed: %s", esp_err_to_name(err2));
            break;
        }
        offset += chunk;
    }

    heap_caps_free(buf);
    wait_for_fpga_busy_clear_();
}
void MatrixPanel_FPGA_SPI::do_drawRectRGB888_prealloc_(
    int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *data,
    size_t length) {
    if (!initialized) {
        ESP_LOGI("drawRectRGB888_prealloc()",
                 "Tried to set output brightness before begin()");
        return;
    }
    if (data == nullptr) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRectRGB888_prealloc",
                 "Invalid data passed to drawRectRGB888_prealloc nullptr! "
                 "(length=%d)",
                 length);
        return;
    }
    if (x < 0 || y < 0 || w <= 0 || h <= 0) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRectRGB888_prealloc",
                 "Invalid rect params x=%d y=%d w=%d h=%d", x, y, w, h);
        return;
    }
    const size_t expected_len = static_cast<size_t>(w) *
                                static_cast<size_t>(h) * 3;
    if (length != expected_len) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRectRGB888_prealloc",
                 "Invalid data length=%d expected=%d", length, expected_len);
        return;
    }
    if (w == 1 && y == 0 && h == height()) {
        do_drawColumnRGB888_(x, data, length);
        return;
    }
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
        return;
    if (!wait_for_fpga_resetstatus_())
        return;

    uint8_t header[7];
    uint8_t header_len = 0;
    header[header_len++] = 'X'; // Command byte
    if (PIXELS_PER_ROW <= 0xff) {
        header[header_len++] = static_cast<uint8_t>(x);
    } else {
        header[header_len++] = static_cast<uint8_t>((x >> 8) & 0xFF);
        header[header_len++] = static_cast<uint8_t>(x & 0xFF);
    }
    header[header_len++] = static_cast<uint8_t>(y);
    if (PIXELS_PER_ROW <= 0xff) {
        header[header_len++] = static_cast<uint8_t>(w);
    } else {
        header[header_len++] = static_cast<uint8_t>((w >> 8) & 0xFF);
        header[header_len++] = static_cast<uint8_t>(w & 0xFF);
    }
    header[header_len++] = static_cast<uint8_t>(h);

    spi_transaction_t t = {
        .length = (size_t)(8 * header_len), // bits
        .tx_buffer = header,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawRectRGB888_prealloc",
                 "SPI transmit failed: %s", esp_err_to_name(err));
        return;
    }

    size_t offset = 0;
    while (offset < length) {
        const size_t chunk =
            std::min(length - offset, static_cast<size_t>(SPI_MAX_DMA_LEN));
        spi_transaction_t t2 = {
            .length = static_cast<size_t>(chunk * 8), // bits
            .tx_buffer = data + offset,
        };
        esp_err_t err2 = spi_device_transmit(spi_bus, &t2);
        if (err2 != ESP_OK) {
            ESP_LOGE("MatrixPanel_FPGA_SPI:drawRectRGB888_prealloc",
                     "SPI transmit failed: %s", esp_err_to_name(err2));
            break;
        }
        offset += chunk;
    }
    wait_for_fpga_busy_clear_();
}

void MatrixPanel_FPGA_SPI::drawRectRGB888(int16_t x, int16_t y, int16_t w,
                                          int16_t h, const uint8_t *data,
                                          size_t length) {
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return;
        Job j;
        j.op = Op::DRAW_RECT;
        j.x = x;
        j.y = static_cast<uint8_t>(y);
        j.w = w;
        j.h = h;
        j.data = data;
        j.length = length;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_drawRectRGB888_(x, y, w, h, data, length);
}

void MatrixPanel_FPGA_SPI::drawRectRGB888_prealloc(int16_t x, int16_t y,
                                                   int16_t w, int16_t h,
                                                   const uint8_t *data,
                                                   size_t length) {
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return;
        Job j;
        j.op = Op::DRAW_RECT_PREALLOC;
        j.x = x;
        j.y = static_cast<uint8_t>(y);
        j.w = w;
        j.h = h;
        j.data = data;
        j.length = length;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_drawRectRGB888_prealloc_(x, y, w, h, data, length);
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
        return;
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

void MatrixPanel_FPGA_SPI::do_drawColumnRGB888_(int16_t x, const uint8_t *data,
                                                size_t length) {
    if (!initialized) {
        ESP_LOGI("drawColumnRGB888()",
                 "Tried to set output brightness before begin()");
        return;
    }
    if (data == nullptr) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawColumnRGB888",
                 "Invalid data passed to drawColumnRGB888 nullptr! (length=%d)",
                 length);
        return;
    }
    if (x < 0 || x >= width()) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawColumnRGB888",
                 "Invalid column x=%d", x);
        return;
    }
    const size_t expected_len =
        static_cast<size_t>(height()) * static_cast<size_t>(3);
    if (length != expected_len) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawColumnRGB888",
                 "Invalid data length=%d expected=%d", length, expected_len);
        return;
    }
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
        return;
    if (!wait_for_fpga_resetstatus_())
        return;

    uint8_t header_len = 0;
    uint8_t header[3];
    header[header_len++] = 'K'; // Command byte
    if (PIXELS_PER_ROW <= 0xff) {
        header[header_len++] = static_cast<uint8_t>(x);
    } else {
        header[header_len++] = static_cast<uint8_t>((x >> 8) & 0xFF);
        header[header_len++] = static_cast<uint8_t>(x & 0xFF);
    }

    const size_t total_bytes = length + header_len;
    uint8_t buf[total_bytes];
    memcpy(buf, header, header_len);
    memcpy(&buf[header_len], data, length);

    spi_transaction_t t = {
        .length = static_cast<size_t>(total_bytes * 8), // bits
        .tx_buffer = buf,
    };
    esp_err_t err = spi_device_transmit(spi_bus, &t);
    if (err != ESP_OK) {
        ESP_LOGE("MatrixPanel_FPGA_SPI:drawColumnRGB888",
                 "SPI transmit failed: %s", esp_err_to_name(err));
        return;
    }
    wait_for_fpga_busy_clear_();
}

void MatrixPanel_FPGA_SPI::drawColumnRGB888(int16_t x, const uint8_t *data,
                                            size_t length) {
    if (use_worker_) {
        if (!tx_q_ || !tx_task_)
            return;
        Job j;
        j.op = Op::DRAW_COL;
        j.x = x;
        j.data = data;
        j.length = length;
        (void)xQueueSend(tx_q_, &j, 0);
        return;
    }
    do_drawColumnRGB888_(x, data, length);
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
        return;
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
        return;
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
        return;
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
        return;
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
        return;
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
    // - yellow left band + magenta right band via drawColumnRGB888
    // - opposing diagonals (orange/white) to cover drawPixelRGB888
    // - centered drawRectRGB888 rect: size=max(4, width/5)xmax(4, height/3)
    //      BLACK        GRADIANT         RED
    //      .............GRADIANT..............
    //      .............GRADIANT..............
    //      TEAL/GREEN   GRADIANT        YELLOW
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

    // Left accent band: yellow using drawColumnRGB888.
    std::vector<uint8_t> column_buf(static_cast<size_t>(height) * 3);
    for (int y = 0; y < height; ++y) {
        const size_t idx = static_cast<size_t>(y) * 3;
        column_buf[idx] = 0xFF;
        column_buf[idx + 1] = 0xFF;
        column_buf[idx + 2] = 0x33;
    }
    for (int x = 0; x < vertical_bar_width; ++x) {
        drawColumnRGB888(x, column_buf.data(), column_buf.size());
    }
    wait_for_worker_idle();
    delay_if_needed();

    // Right accent band: magenta using drawColumnRGB888.
    for (int y = 0; y < height; ++y) {
        const size_t idx = static_cast<size_t>(y) * 3;
        column_buf[idx] = 0xFF;
        column_buf[idx + 1] = 0x00;
        column_buf[idx + 2] = 0xFF;
    }
    const int right_start = std::max(width - vertical_bar_width, 0);
    for (int x = right_start; x < width; ++x) {
        drawColumnRGB888(x, column_buf.data(), column_buf.size());
    }
    wait_for_worker_idle();
    delay_if_needed();

    // Buffered rectangle: gradient checker to exercise drawRectRGB888.
    const int rect_w = std::max(4, width / 5);
    const int rect_h = std::max(4, height / 3);
    const int rect_x = std::max(0, (width - rect_w) / 2);
    const int rect_y = std::max(0, (height - rect_h) / 2);
    const int rect_w_denom = std::max(1, rect_w - 1);
    const int rect_h_denom = std::max(1, rect_h - 1);
    std::vector<uint8_t> rect_buf(
        static_cast<size_t>(rect_w) * static_cast<size_t>(rect_h) * 3);
    for (int y = 0; y < rect_h; ++y) {
        for (int x = 0; x < rect_w; ++x) {
            const size_t idx =
                (static_cast<size_t>(y) * rect_w + x) * 3;
            const uint8_t r = static_cast<uint8_t>(
                (x * 255) / rect_w_denom);
            const uint8_t g = static_cast<uint8_t>(
                (y * 255) / rect_h_denom);
            const uint8_t b = ((x + y) & 1) ? 0x20 : 0xA0;
            rect_buf[idx] = r;
            rect_buf[idx + 1] = g;
            rect_buf[idx + 2] = b;
        }
    }
    drawRectRGB888(rect_x, rect_y, rect_w, rect_h, rect_buf.data(),
                   rect_buf.size());
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

esp_err_t MatrixPanel_FPGA_SPI::init_spi(const FPGA_SPI_CFG &cfg) {
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
    // SPI2_HOST exists on all ESP32 targets; on legacy ESP32 it is the same
    // host HSPI_HOST aliased (the alias does not exist on ESP32-S3).
    esp_err_t err = init_spi_bus_device_(SPI2_HOST, buscfg, devcfg,
                                         SPI_DMA_CH_AUTO, spi_bus);
    if (err != ESP_OK)
        return err;
    if (!spi_mutex_) {
        spi_mutex_ = xSemaphoreCreateMutex();
        if (!spi_mutex_) {
            ESP_LOGE("spi_init", "Failed to create SPI mutex");
            spi_bus_remove_device(spi_bus);
            spi_bus = nullptr;
            spi_bus_free(SPI2_HOST);
            return ESP_ERR_NO_MEM;
        }
    }
    initialized = true;
    ESP_LOGD("spi_init", "done");
    return ESP_OK;
}

esp_err_t MatrixPanel_FPGA_SPI::init_spi_bus_device_(
    spi_host_device_t host, const spi_bus_config_t &buscfg,
    const spi_device_interface_config_t &devcfg, spi_dma_chan_t dma_chan,
    spi_device_handle_t &handle_out) {
    esp_err_t err = spi_bus_initialize(host, &buscfg, dma_chan);
    if (err != ESP_OK) {
        ESP_LOGE("spi_init", "spi_bus_initialize failed: %s",
                 esp_err_to_name(err));
        return err;
    }
    err = spi_bus_add_device(host, &devcfg, &handle_out);
    if (err != ESP_OK) {
        ESP_LOGE("spi_init", "spi_bus_add_device failed: %s",
                 esp_err_to_name(err));
        spi_bus_free(host);
        return err;
    }
    return ESP_OK;
}

esp_err_t MatrixPanel_FPGA_SPI::init_status_spi_(const FPGA_SPI_CFG &cfg) {
    if (cfg.status_gpio.sck < 0 || cfg.status_gpio.cs < 0 ||
        cfg.status_gpio.miso < 0)
        return ESP_OK; // feature not configured
    spi_bus_config_t buscfg = {
        .mosi_io_num = -1, // responder is TX-only
        .miso_io_num = (gpio_num_t)cfg.status_gpio.miso,
        .sclk_io_num = (gpio_num_t)cfg.status_gpio.sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    // The responder synchronizes cs_n through 2 flip-flops (~160 ns) before
    // it presents the first frame byte, so CS must assert well before the
    // first clock edge or byte 0 is lost. cs_ena_pretrans provides that setup
    // time (half-duplex only) and is measured in SPI bit-cycles, so derive
    // the count from the clock: >= 500 ns of setup, ceil-rounded. The
    // hardware caps the field at 16 cycles, which keeps >= 500 ns up to
    // 32 MHz (and still >= 160 ns to 100 MHz).
    constexpr uint64_t kCsSetupNs = 500;
    const uint16_t cs_setup_cycles = (uint16_t)std::min<uint64_t>(
        16, (kCsSetupNs * cfg.status_spispeed + 999999999) / 1000000000);
    spi_device_interface_config_t devcfg = {
        .mode = 3, // CPOL=1/CPHA=1 per reg_spi_responder
        .cs_ena_pretrans = cs_setup_cycles,
        .clock_speed_hz = (int)cfg.status_spispeed,
        .spics_io_num = (gpio_num_t)cfg.status_gpio.cs,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY,
        .queue_size = 1,
    };
    gpio_reset_pin((gpio_num_t)cfg.status_gpio.sck);
    gpio_reset_pin((gpio_num_t)cfg.status_gpio.cs);
    gpio_reset_pin((gpio_num_t)cfg.status_gpio.miso);
    // 12-byte frames don't need DMA; disabling it allows a stack rx buffer.
    esp_err_t err = init_spi_bus_device_(SPI3_HOST, buscfg, devcfg,
                                         SPI_DMA_DISABLED, status_spi_dev_);
    if (err == ESP_OK)
        status_spi_configured_ = true;
    return err;
}

// CRC-16/XMODEM (poly 0x1021, init 0, no reflection, no final xor); matches
// the FPGA's crc16.sv over the 10-byte frame body.
static constexpr uint16_t crc16_xmodem_(const uint8_t *d, size_t n) {
    uint16_t crc = 0;
    while (n--) {
        crc ^= (uint16_t)(*d++ << 8);
        for (int i = 0; i < 8; ++i)
            crc = crc & 0x8000 ? (uint16_t)((crc << 1) ^ 0x1021)
                               : (uint16_t)(crc << 1);
    }
    return crc;
}
static constexpr uint8_t kCrcCheckInput[] = {'1', '2', '3', '4', '5',
                                             '6', '7', '8', '9'};
static_assert(crc16_xmodem_(kCrcCheckInput, 9) == 0x31C3,
              "CRC-16/XMODEM self-check failed");

bool MatrixPanel_FPGA_SPI::readStatus(uint8_t addr, uint64_t &value) {
    constexpr int kRetries = 3;
    if (!initialized || !status_spi_configured_ || addr == STATUS_ADDR_NONE) {
        last_status_error_ = StatusReadError::NOT_CONFIGURED;
        return false;
    }
    // One lock spans request and readback so no other 'S' can interleave.
    SpiLockGuard spi_lock(this);
    if (!spi_lock.locked())
        return false;
    if (!wait_for_fpga_resetstatus_())
        return false;
    // Outer: re-issue the request (command lost / stale seq / wrong echo).
    for (int req = 0; req < kRetries; req++) {
        uint8_t cmd[2] = {'S', addr};
        spi_transaction_t tc = {
            .length = 8 * sizeof(cmd), // spi_transaction_t.length is in bits
            .tx_buffer = cmd,
        };
        if (spi_device_transmit(spi_bus, &tc) != ESP_OK) {
            last_status_error_ = StatusReadError::CMD_TX_ERROR;
            continue;
        }
        // Inner: re-read only; a CRC failure is a clock-domain tear and the
        // responder re-frames the same mailbox on each CS assertion.
        for (int rd = 0; rd < kRetries; rd++) {
            StatusFrame frame;
            // Half-duplex RX-only: length is the (empty) TX phase, rxlength
            // is the RX phase. Both are in bits.
            spi_transaction_t tr = {
                .length = 0,
                .rxlength = 8 * sizeof(frame),
                .rx_buffer = &frame,
            };
            if (spi_device_transmit(status_spi_dev_, &tr) != ESP_OK) {
                last_status_error_ = StatusReadError::BUS_RX_ERROR;
                continue;
            }
            memcpy(last_status_frame_, &frame, sizeof(frame));
            // CRC covers every field before the crc member.
            if (crc16_xmodem_(reinterpret_cast<const uint8_t *>(&frame),
                              offsetof(StatusFrame, crc)) !=
                (uint16_t)((frame.crc[0] << 8) | frame.crc[1])) {
                last_status_error_ = StatusReadError::CRC_MISMATCH;
                continue;
            }
            if (frame.addr != addr) {
                last_status_error_ = StatusReadError::ADDR_MISMATCH;
                break; // mailbox holds another request; re-issue ours
            }
            if (have_last_seq_ && frame.seq == last_seq_) {
                last_status_error_ = StatusReadError::STALE_SEQ;
                break; // stale: our request not latched yet; re-issue
            }
            last_seq_ = frame.seq;
            have_last_seq_ = true;
            last_status_error_ = StatusReadError::NONE;
            value = 0;
            for (size_t i = 0; i < sizeof(frame.value); i++)
                value = (value << 8) | frame.value[i];
            return true;
        }
    }
    ESP_LOGW("readStatus", "no fresh frame for addr 0x%02X (%s)", addr,
             status_error_str(last_status_error_));
    return false;
}
