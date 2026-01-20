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
#include <stdint.h>

/***************************************************************************************/
class MatrixPanel_FPGA_SPI {

    // ------- PUBLIC -------
  public:
    MatrixPanel_FPGA_SPI() {}
    MatrixPanel_FPGA_SPI(const FPGA_SPI_CFG &opts) { setCfg(opts); }

    void init_spi(const FPGA_SPI_CFG &cfg);
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

        init_spi(m_cfg);
        init_fpga_resetstatus_gpio_();

        start_worker();
        while (!initialized)
            ;
        if (!initialized) {
            ESP_LOGE("being()", "MatrixPanel_FPGA_SPI::begin() failed!");
        }
        return initialized;
    }

    // Obj destructor
    virtual ~MatrixPanel_FPGA_SPI() {
        // dma_bus.release();
    }

    bool begin(const FPGA_SPI_CFG &cfg);
    void clearScreen();

    // rgb888 overload
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r,
                  uint8_t g, uint8_t b);
    void fillScreenRGB888(uint8_t r, uint8_t g, uint8_t b);
    void drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b);
    void drawRowRGB888(const uint8_t y, const uint8_t *data, size_t length);
    void drawFrameRGB888(const uint8_t *data, size_t length);
    bool queue_has_space(size_t slots = 1) const;
    bool worker_is_idle() const;
    bool is_worker_enabled() const { return use_worker_; }
    void run_test_graphic(uint32_t delay_ms = 10);
    void swapFrame();
    void fulfillWatchdog();
    void resync_after_fpga_reset(uint8_t brightness);
    bool consume_fpga_reset();
    uint32_t get_reset_epoch() const { return reset_epoch_; }
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
    void do_fillRect_(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r,
                      uint8_t g, uint8_t b);
    void do_fillScreenRGB888_(uint8_t r, uint8_t g, uint8_t b);
    void do_drawPixelRGB888_(int16_t x, int16_t y, uint8_t r, uint8_t g,
                             uint8_t b);
    void do_drawRowRGB888_(const uint8_t y, const uint8_t *data, size_t length);
    void do_drawFrameRGB888_(const uint8_t *data, size_t length);
    void do_swapFrame_();
    void do_fulfillWatchdog_();
    void do_setBrightness8_(const uint8_t b);
    void init_fpga_resetstatus_gpio_();
    bool wait_for_fpga_resetstatus_();
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
    volatile bool fpga_reset_seen_ = false;
    uint32_t reset_epoch_ = 0;
    volatile bool worker_busy_ = false;
    SemaphoreHandle_t spi_mutex_ = nullptr;

    enum class Op : uint8_t {
        DRAW_ROW,
        SWAP,
        WATCHDOG,
        FILL_SCREEN,
        SET_BRIGHTNESS,
        CLEAR,
        DRAW_FRAME,
        DRAW_PIXEL,
        FILL_RECT
    };
    struct Job {
        Op op;
        const uint8_t *data = nullptr; // for DRAW_ROW / DRAW_FRAME
        size_t length = 0;             // for DRAW_ROW / DRAW_FRAME

        uint8_t y = 0; // row index
        int16_t x = 0; // for DRAW_PIXEL / FILL_RECT
        int16_t w = 0; // for FILL_RECT
        int16_t h = 0; // for FILL_RECT

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
            else if (j.op == Op::DRAW_FRAME)
                do_drawFrameRGB888_(j.data, j.length);
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
