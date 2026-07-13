// SPDX-FileCopyrightText: 2018-2032 mrcodetastic
// SPDX-FileCopyrightText: 2025 Aaron White <w531t4@gmail.com>
// SPDX-License-Identifier: MIT
#ifndef _ESP32_RGB_64_32_MATRIX_PANEL_SPI_DMA
#define _ESP32_RGB_64_32_MATRIX_PANEL_SPI_DMA
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "matrix_panel_fpga_config.hpp"
#include <atomic>
#include <cstdio>
#include <cstring>
#include <stdint.h>

/***************************************************************************************/
class MatrixPanel_FPGA_SPI {

    // ------- PUBLIC -------
  public:
    MatrixPanel_FPGA_SPI() {}
    MatrixPanel_FPGA_SPI(const FPGA_SPI_CFG &opts) { setCfg(opts); }

    esp_err_t init_spi(const FPGA_SPI_CFG &cfg);
    /* Propagate the DMA pin configuration, allocate DMA buffs and start data
     * output, initially blank */
    bool begin() {
        if (initialized)
            return true; // we don't do this twice or more!

        if (!config_set)
            return false;

        ESP_LOGI("begin()", "Using GPIO %d for SPI_CE_PIN", m_cfg.gpio.ce);
        ESP_LOGI("begin()", "Using GPIO %d for SPI_CLK_PIN", m_cfg.gpio.clk);
        ESP_LOGI("begin()", "Using GPIO %d for SPI_MOSI_PIN", m_cfg.gpio.mosi);
        ESP_LOGI("begin()",
                 "FPGA effective display resolution of width: %dpx height: "
                 "%dpx.",
                 m_cfg.mx_width * m_cfg.chain_length, m_cfg.mx_height);

        if (m_cfg.mx_height % 2 != 0) {
            ESP_LOGE("begin()",
                     "Error: m_cfg.mx_height must be an even number!");
            return false;
        }

        if (init_spi(m_cfg) != ESP_OK) {
            ESP_LOGE("begin()", "MatrixPanel_FPGA_SPI::begin() failed!");
            return false;
        }
        if (init_status_spi_(m_cfg) != ESP_OK) {
            ESP_LOGW("begin()", "status SPI init failed; readback disabled");
        } else if (status_spi_configured_) {
            ESP_LOGI("begin()", "Using GPIO %d for STATUS_SPI_SCK_PIN",
                     m_cfg.status_gpio.sck);
            ESP_LOGI("begin()", "Using GPIO %d for STATUS_SPI_CS_PIN",
                     m_cfg.status_gpio.cs);
            ESP_LOGI("begin()", "Using GPIO %d for STATUS_SPI_MISO_PIN",
                     m_cfg.status_gpio.miso);
            ESP_LOGI("begin()",
                     "Status SPI ready on SPI3_HOST (mode 3, %u Hz)",
                     (unsigned)m_cfg.status_spispeed);
        } else {
            ESP_LOGI("begin()",
                     "Status SPI disabled (STATUS_SPI pins not configured)");
        }
        init_fpga_resetstatus_gpio_();
        init_fpga_busy_gpio_();

        start_worker();
        return initialized;
    }

    // Obj destructor
    virtual ~MatrixPanel_FPGA_SPI() {
        // dma_bus.release();
    }

    bool begin(const FPGA_SPI_CFG &cfg);
    void clearScreen();

    void copyFrame();
    // rgb888 overload
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r,
                  uint8_t g, uint8_t b);
    void fillScreenRGB888(uint8_t r, uint8_t g, uint8_t b);
    void drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b);
    void drawRowRGB888(const uint8_t y, const uint8_t *data, size_t length);
    void drawColumnRGB888(int16_t x, const uint8_t *data, size_t length);
    void drawFrameRGB888(const uint8_t *data, size_t length);
    bool queue_has_space(size_t slots = 1) const;
    bool worker_is_idle() const;
    bool is_worker_enabled() const { return use_worker_; }
    void drawRectRGB888(int16_t x, int16_t y, int16_t w, int16_t h,
                        const uint8_t *data, size_t length);
    void drawRectRGB888_prealloc(int16_t x, int16_t y, int16_t w, int16_t h,
                                 const uint8_t *data, size_t length);
    void run_test_graphic(uint32_t delay_ms = 10);
    void swapFrame();
    void fulfillWatchdog();
    void resync_after_fpga_reset(uint8_t brightness);
    bool consume_fpga_reset();
    // Non-blocking readiness probe: false while the FPGA is held in
    // reset/config (e.g. a remote JTAG reflash). Reads the same pin/level that
    // consume_fpga_reset() checks; always true when RESETSTATUS is not wired.
    inline bool fpga_ready() const {
        if (!fpga_resetstatus_configured_)
            return true;
        return gpio_get_level((gpio_num_t)m_cfg.gpio.fpga_resetstatus) != 0;
    }
    uint32_t get_reset_epoch() const { return reset_epoch_; }
    // Status readback registers; mirrors status_addr_e in the FPGA's
    // packages/types.sv -- keep in sync.
    static constexpr uint8_t STATUS_ADDR_FLAGS = 0x00;
    static constexpr uint8_t STATUS_ADDR_RGB = 0x01;
    static constexpr uint8_t STATUS_ADDR_BRIGHTNESS = 0x02;
    // Rolling measurement of kilobytes/s received by the FPGA from the ESP32.
    static constexpr uint8_t STATUS_ADDR_RX_KBPS = 0x03;
    // HUB75 frame-emit rate, Hz, 5 s sliding average.
    static constexpr uint8_t STATUS_ADDR_HUB75_FPS = 0x04;
    // Framebuffer swap rate, /s, 5 s average (0 if !DOUBLE_BUFFER).
    static constexpr uint8_t STATUS_ADDR_FB_FPS = 0x05;
    // Whole seconds since reset (plain counter).
    static constexpr uint8_t STATUS_ADDR_UPTIME = 0x06;
    // Packed gateware version, decoded by readVersion() (mirrors reg_version.sv).
    static constexpr uint8_t STATUS_ADDR_VERSION = 0x07;
    static constexpr uint8_t STATUS_ADDR_NONE = 0xFF; // reserved sentinel
    struct FpgaStatusFlags {
        bool fpga_ready, ctrl_busy, ctrl_ready_for_data;
    };
    // Decoded STATUS_ADDR_VERSION; widths mirror version_t in reg_version.sv.
    struct FpgaVersion {
        uint8_t major, minor, patch; // each <= 127
        uint32_t git_sha;            // high 32 bits of the HEAD hash
        uint16_t commits;            // since the tag, <= 1023
        bool dirty;
    };
    static constexpr size_t VERSION_STR_MAX = 40; // >= any formatVersion() output
    // Field sizes of the responder's mailbox frame; mirror the STATUS_*_BYTES
    // parameters in the FPGA's packages/params.sv. addr and seq are single
    // bytes (scalar members below).
    static constexpr size_t STATUS_VALUE_LEN = 8;
    static constexpr size_t STATUS_CRC_LEN = 2;
    // Wire format of the frame: field order/offsets come from member order,
    // sizes from the constants above. Multi-byte fields are MSB-first on the
    // wire. All members are bytes, so the struct matches the wire layout
    // exactly (packed guards against any padding).
    struct __attribute__((packed)) StatusFrame {
        uint8_t addr; // echo of the requested register (0xFF = never latched)
        uint8_t seq;  // mailbox latch counter
        uint8_t value[STATUS_VALUE_LEN]; // register value, MSB first
        uint8_t crc[STATUS_CRC_LEN]; // CRC-16/XMODEM over addr..value, MSB 1st
    };
    static constexpr size_t STATUS_FRAME_LEN = sizeof(StatusFrame);
    // Why the most recent readStatus() attempt failed (or NONE on success).
    // When several retries fail for different reasons, the last one wins.
    enum class StatusReadError : uint8_t {
        NONE = 0,
        NOT_CONFIGURED,  // status bus disabled or begin() not run
        CMD_TX_ERROR,    // READSTATUS command failed on the main bus
        BUS_RX_ERROR,    // frame transfer failed on the status bus
        CRC_MISMATCH,    // frame corrupted (tear, floating MISO, bad wiring)
        ADDR_MISMATCH,   // mailbox echoed a different register address
        STALE_SEQ,       // frame valid but seq unchanged: request not latched
    };
    static const char *status_error_str(StatusReadError e) {
        switch (e) {
        case StatusReadError::NONE:
            return "none";
        case StatusReadError::NOT_CONFIGURED:
            return "not configured";
        case StatusReadError::CMD_TX_ERROR:
            return "command TX failed (main bus)";
        case StatusReadError::BUS_RX_ERROR:
            return "frame RX failed (status bus)";
        case StatusReadError::CRC_MISMATCH:
            return "CRC mismatch";
        case StatusReadError::ADDR_MISMATCH:
            return "address echo mismatch";
        case StatusReadError::STALE_SEQ:
            return "stale seq (request not latched)";
        }
        return "unknown";
    }
    StatusReadError last_status_error() const { return last_status_error_; }
    // Copies the raw bytes of the last frame clocked off the status bus
    // (all zeros if no transfer has completed yet).
    void last_status_frame(uint8_t out[STATUS_FRAME_LEN]) const {
        memcpy(out, last_status_frame_, STATUS_FRAME_LEN);
    }
    bool status_spi_available() const { return status_spi_configured_; }
    // Synchronous register readback over the status SPI. Sends ['S'][addr] on
    // the command bus, clocks the 12-byte mailbox frame from the responder,
    // validates CRC/echo/seq with retries. False if unavailable or no fresh
    // frame.
    bool readStatus(uint8_t addr, uint64_t &value);
    bool readFlags(FpgaStatusFlags &out) {
        uint64_t v;
        if (!readStatus(STATUS_ADDR_FLAGS, v))
            return false;
        // FLAGS register bit layout, per the FPGA's display_core.sv:
        // value[2:0] = {fpga_ready, ctrl_busy, ctrl_ready_for_data}
        out.fpga_ready = (v >> 2) & 1;
        out.ctrl_busy = (v >> 1) & 1;
        out.ctrl_ready_for_data = (v >> 0) & 1;
        return true;
    }
    bool readVersion(FpgaVersion &out) {
        uint64_t v;
        if (!readStatus(STATUS_ADDR_VERSION, v))
            return false;
        // version_t is packed MSB-first (reg_version.sv); take() consumes the
        // fields in declaration order, widths matching the SV logic[] sizes
        // (7+7+7+32+10+1 = 64).
        unsigned pos = 0;
        auto take = [&](unsigned width) {
            pos += width;
            return (uint32_t)((v >> (64 - pos)) & ((1ull << width) - 1));
        };
        out.major = take(7);
        out.minor = take(7);
        out.patch = take(7);
        out.git_sha = take(32);
        out.commits = take(10);
        out.dirty = take(1);
        return true;
    }
    // "1.2.3" at an exact clean tag, else "1.2.3+<commits>.sha<sha>" with a
    // "-dirty" suffix when the tree was dirty.
    static int formatVersion(const FpgaVersion &v, char *buf, size_t len) {
        if (v.commits == 0 && !v.dirty)
            return snprintf(buf, len, "%u.%u.%u", v.major, v.minor, v.patch);
        return snprintf(buf, len, "%u.%u.%u+%u.sha%08x%s", v.major, v.minor,
                        v.patch, v.commits, v.git_sha, v.dirty ? "-dirty" : "");
    }
    // read + format in one call; buf should be >= VERSION_STR_MAX.
    bool readVersionString(char *buf, size_t len) {
        FpgaVersion v;
        if (!readVersion(v))
            return false;
        formatVersion(v, buf, len);
        return true;
    }
    inline int16_t width() const { return m_cfg.mx_width * m_cfg.chain_length; }
    inline int16_t height() const { return m_cfg.mx_height; }
    void setBrightness8(const uint8_t b);
    const FPGA_SPI_CFG &getCfg() const { return m_cfg; };

    inline bool setCfg(const FPGA_SPI_CFG &cfg) {
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
    bool lock_spi_();
    void unlock_spi_();
    class SpiLockGuard {
      public:
        explicit SpiLockGuard(MatrixPanel_FPGA_SPI *self)
            : self_(self), locked_(self_->lock_spi_()) {}
        ~SpiLockGuard() {
            if (locked_)
                self_->unlock_spi_();
        }
        bool locked() const { return locked_; }

      private:
        MatrixPanel_FPGA_SPI *self_;
        bool locked_;
    };

    void do_clearScreen_();
    void do_copyFrame_();
    void do_fillRect_(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r,
                      uint8_t g, uint8_t b);
    void do_fillScreenRGB888_(uint8_t r, uint8_t g, uint8_t b);
    void do_drawPixelRGB888_(int16_t x, int16_t y, uint8_t r, uint8_t g,
                             uint8_t b);
    void do_drawRowRGB888_(const uint8_t y, const uint8_t *data, size_t length);
    void do_drawColumnRGB888_(int16_t x, const uint8_t *data, size_t length);
    void do_drawFrameRGB888_(const uint8_t *data, size_t length);
    void do_drawRectRGB888_(int16_t x, int16_t y, int16_t w, int16_t h,
                            const uint8_t *data, size_t length);
    void do_drawRectRGB888_prealloc_(int16_t x, int16_t y, int16_t w, int16_t h,
                                     const uint8_t *data, size_t length);
    void do_swapFrame_();
    void do_fulfillWatchdog_();
    void do_setBrightness8_(const uint8_t b);
    esp_err_t init_status_spi_(const FPGA_SPI_CFG &cfg);
    static esp_err_t init_spi_bus_device_(
        spi_host_device_t host, const spi_bus_config_t &buscfg,
        const spi_device_interface_config_t &devcfg, spi_dma_chan_t dma_chan,
        spi_device_handle_t &handle_out);
    void init_fpga_resetstatus_gpio_();
    void init_fpga_busy_gpio_();
    bool wait_for_fpga_resetstatus_();
    bool wait_for_fpga_busy_clear_();
    static void fpga_resetstatus_isr_(void *arg);
    // Matrix i2s settings
    FPGA_SPI_CFG m_cfg;

    int brightness = 128; // If you get ghosting... reduce brightness level.
                          // ((60/64)*255) seems to be the limit before ghosting
                          // on a 64 pixel wide physical panel for some panels.

    uint16_t PIXELS_PER_ROW =
        m_cfg.mx_width *
        m_cfg.chain_length; // number of pixels in a single row of all chained
                            // matrix modules (WIDTH of a combined matrix
                            // chain)
    uint8_t ROWS_PER_FRAME = m_cfg.mx_height;

    // Other private variables
    bool initialized = false;
    bool config_set = false;
    bool fpga_resetstatus_configured_ = false;
    bool fpga_busy_configured_ = false;
    std::atomic<bool> fpga_reset_seen_{false};
    std::atomic<uint32_t> reset_epoch_{0};
    spi_device_handle_t status_spi_dev_ = nullptr;
    bool status_spi_configured_ = false;
    // Freshness tracking for readStatus (mailbox seq); cleared on FPGA reset.
    std::atomic<bool> have_last_seq_{false};
    uint8_t last_seq_ = 0;
    // Diagnostics from the most recent readStatus() (guarded by spi_mutex_).
    StatusReadError last_status_error_ = StatusReadError::NONE;
    uint8_t last_status_frame_[STATUS_FRAME_LEN] = {};
    volatile bool worker_busy_ = false;
    SemaphoreHandle_t spi_mutex_ = nullptr;

    enum class Op : uint8_t {
        DRAW_ROW,
        DRAW_COL,
        SWAP,
        WATCHDOG,
        FILL_SCREEN,
        SET_BRIGHTNESS,
        CLEAR,
        DRAW_FRAME,
        DRAW_RECT,
        DRAW_RECT_PREALLOC,
        COPY_FRAME,
        DRAW_PIXEL,
        FILL_RECT
    };
    struct Job {
        Op op;
        const uint8_t *data = nullptr; // for DRAW_ROW / DRAW_COL / DRAW_FRAME / DRAW_RECT / DRAW_RECT_PREALLOC
        size_t length = 0; // for DRAW_ROW / DRAW_COL / DRAW_FRAME / DRAW_RECT / DRAW_RECT_PREALLOC

        uint8_t y = 0; // row index
        int16_t x = 0; // for DRAW_PIXEL / DRAW_RECT / DRAW_RECT_PREALLOC / FILL_RECT
        int16_t w = 0; // for DRAW_RECT / DRAW_RECT_PREALLOC / FILL_RECT
        int16_t h = 0; // for DRAW_RECT / DRAW_RECT_PREALLOC / FILL_RECT

        uint8_t r = 0, g = 0,
                b = 0;  // for colors (FILL_SCREEN / DRAW_PIXEL / FILL_RECT)
        uint8_t u8 = 0; // for SET_BRIGHTNESS
    };
    bool use_worker_ = false;
    uint32_t worker_core_ = 0;
    TaskHandle_t tx_task_ = nullptr;
    QueueHandle_t tx_q_ = nullptr;
    static void tx_task_entry_(void *arg) {
        static_cast<MatrixPanel_FPGA_SPI *>(arg)->tx_task_run_();
    }
    void tx_task_run_() {
        // ESP_LOGI("cpu_probe","core=%d", xPortGetCoreID());
        Job j;
        for (;;) {
            if (xQueueReceive(tx_q_, &j, portMAX_DELAY) != pdTRUE)
                continue;
            this->worker_busy_ = true;
            if (j.op == Op::DRAW_ROW)
                do_drawRowRGB888_(j.y, j.data, j.length);
            else if (j.op == Op::DRAW_COL)
                do_drawColumnRGB888_(j.x, j.data, j.length);
            else if (j.op == Op::SWAP)
                do_swapFrame_();
            else if (j.op == Op::WATCHDOG)
                do_fulfillWatchdog_();
            else if (j.op == Op::FILL_SCREEN)
                do_fillScreenRGB888_(j.r, j.g, j.b);
            else if (j.op == Op::SET_BRIGHTNESS)
                do_setBrightness8_(j.u8);
            else if (j.op == Op::CLEAR)
                do_clearScreen_();
            else if (j.op == Op::COPY_FRAME)
                do_copyFrame_();
            else if (j.op == Op::DRAW_FRAME)
                do_drawFrameRGB888_(j.data, j.length);
            else if (j.op == Op::DRAW_RECT)
                do_drawRectRGB888_(j.x, j.y, j.w, j.h, j.data, j.length);
            else if (j.op == Op::DRAW_RECT_PREALLOC)
                do_drawRectRGB888_prealloc_(j.x, j.y, j.w, j.h, j.data,
                                            j.length);
            else if (j.op == Op::DRAW_PIXEL)
                do_drawPixelRGB888_(j.x, j.y, j.r, j.g, j.b);
            else if (j.op == Op::FILL_RECT)
                do_fillRect_(j.x, j.y, j.w, j.h, j.r, j.g, j.b);
            this->worker_busy_ = false;
        }
    }
    static constexpr UBaseType_t TX_TASK_PRIO = 2; // was 3
  public:
    void set_worker_core(uint32_t core) { worker_core_ = core; }
    void enable_worker(bool on) { use_worker_ = on; }
    void start_worker() {
        if (!use_worker_)
            return;
        if (!tx_q_)
            tx_q_ = xQueueCreate(34, sizeof(Job));
        if (!tx_task_)
            xTaskCreatePinnedToCore(tx_task_entry_, "mp_spi_tx", 4096, this,
                                    TX_TASK_PRIO, &tx_task_, worker_core_);
    }
};

#endif
