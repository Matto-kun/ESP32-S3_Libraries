/**
 * @file        driver_st7789_interface.cpp
 * @brief       ESP32-S3 LibDriver ST7789 interface (spi_api + gpio_api)
 * @author      Mat RB
 * @date        2026-08-12
 * @version     1.0.0
 *
 * @details
 *   Implements driver_st7789_interface.h for ESP32-S3. SPI uses project spi_api
 *   (CS tied low on the module — no chip-select framing). DC/RST use gpio_api.
 *   Board pins come from display_board_set().
 *
 * @note LibDriver author: Shifeng Li — https://github.com/libdriver/st7789
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#include "driver_st7789_interface.h"
#include "display/display_board.hpp"
#include "spi_api.hpp"
#include "gpio_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdarg.h>
#include <stdio.h>
#include <new>

/* ── Module state ────────────────────────────────────────────────────────── */
static const char* TAG = "ST7789_IF";

static display_board_t s_board = {};
static bool            s_board_set = false;

static SPI*  s_spi = nullptr;
static GPIO* s_dc  = nullptr;
static GPIO* s_rst = nullptr;

alignas(SPI)  static uint8_t s_spi_storage[sizeof(SPI)];
alignas(GPIO) static uint8_t s_dc_storage[sizeof(GPIO)];
alignas(GPIO) static uint8_t s_rst_storage[sizeof(GPIO)];

void display_board_set(const display_board_t& board)
{
    s_board = board;
    s_board_set = true;
}

const display_board_t& display_board_get(void)
{
    return s_board;
}

uint8_t st7789_interface_spi_init(void)
{
    if (!s_board_set) {
        ESP_LOGE(TAG, "display_board_set() required before spi_init");
        return 1;
    }
    if (s_board.mosi_pin == kSpiPinUnused || s_board.sclk_pin == kSpiPinUnused) {
        ESP_LOGE(TAG, "MOSI and SCLK are required");
        return 1;
    }

    if (s_spi != nullptr) {
        return 0;
    }

    spi_cfg_t cfg = {};
    cfg.host      = s_board.host;
    cfg.mosi_pin  = s_board.mosi_pin;
    cfg.miso_pin  = kSpiPinUnused;
    cfg.sclk_pin  = s_board.sclk_pin;
    cfg.clock_hz  = s_board.clock_hz;
    cfg.mode      = eSpiMode0;
    cfg.bit_order = eSpiBitOrderMsb;
    /* Blocking TX with DMA (SPI_DMA_CH_AUTO in spi_api). Async queue deferred. */
    cfg.max_transfer = 4092;
    cfg.queue_size = 1U;

    s_spi = new (s_spi_storage) SPI(cfg);
    if (s_spi->init() != ESP_OK) {
        s_spi->~SPI();
        s_spi = nullptr;
        ESP_LOGE(TAG, "SPI init failed");
        return 1;
    }

    ESP_LOGI(TAG, "SPI inited host=%d mosi=%d sclk=%d clk=%lu",
             (int)s_board.host, (int)s_board.mosi_pin, (int)s_board.sclk_pin,
             (unsigned long)s_board.clock_hz);
    return 0;
}

uint8_t st7789_interface_spi_deinit(void)
{
    if (s_spi == nullptr) {
        return 0;
    }
    (void)s_spi->deinit();
    s_spi->~SPI();
    s_spi = nullptr;
    return 0;
}

uint8_t st7789_interface_spi_write_cmd(uint8_t *buf, uint16_t len)
{
    if (s_spi == nullptr || !s_spi->isInited()) {
        return 1;
    }
    if (buf == nullptr && len > 0U) {
        return 1;
    }
    if (len == 0U) {
        return 0;
    }

    const size_t max_chunk = static_cast<size_t>(s_spi->getConfig().max_transfer);
    size_t offset = 0U;
    while (offset < static_cast<size_t>(len)) {
        size_t chunk = static_cast<size_t>(len) - offset;
        if (chunk > max_chunk && max_chunk > 0U) {
            chunk = max_chunk;
        }
        if (s_spi->write(buf + offset, chunk) != ESP_OK) {
            return 1;
        }
        offset += chunk;
    }
    return 0;
}

void st7789_interface_delay_ms(uint32_t ms)
{
    if (ms == 0U) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void st7789_interface_debug_print(const char *const fmt, ...)
{
    char buf[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    ESP_LOGI(TAG, "%s", buf);
}

uint8_t st7789_interface_cmd_data_gpio_init(void)
{
    if (!s_board_set || s_board.dc_pin < 0) {
        ESP_LOGE(TAG, "DC pin required");
        return 1;
    }
    if (s_dc != nullptr) {
        return 0;
    }

    gpio_cfg_t cfg = {};
    cfg.pin_num   = static_cast<uint8_t>(s_board.dc_pin);
    cfg.direction = eGpioDirOutput;
    cfg.pull      = eGpioPullNone;

    s_dc = new (s_dc_storage) GPIO(cfg);
    if (s_dc->init() != ESP_OK) {
        s_dc->~GPIO();
        s_dc = nullptr;
        return 1;
    }
    (void)s_dc->write(false); /* command level idle */
    return 0;
}

uint8_t st7789_interface_cmd_data_gpio_deinit(void)
{
    if (s_dc == nullptr) {
        return 0;
    }
    (void)s_dc->deinit();
    s_dc->~GPIO();
    s_dc = nullptr;
    return 0;
}

uint8_t st7789_interface_cmd_data_gpio_write(uint8_t value)
{
    if (s_dc == nullptr) {
        return 1;
    }
    return (s_dc->write(value != 0U) == ESP_OK) ? 0U : 1U;
}

uint8_t st7789_interface_reset_gpio_init(void)
{
    if (!s_board_set || s_board.rst_pin < 0) {
        ESP_LOGE(TAG, "RST pin required");
        return 1;
    }
    if (s_rst != nullptr) {
        return 0;
    }

    gpio_cfg_t cfg = {};
    cfg.pin_num   = static_cast<uint8_t>(s_board.rst_pin);
    cfg.direction = eGpioDirOutput;
    cfg.pull      = eGpioPullNone;

    s_rst = new (s_rst_storage) GPIO(cfg);
    if (s_rst->init() != ESP_OK) {
        s_rst->~GPIO();
        s_rst = nullptr;
        return 1;
    }
    (void)s_rst->write(true); /* release reset */
    return 0;
}

uint8_t st7789_interface_reset_gpio_deinit(void)
{
    if (s_rst == nullptr) {
        return 0;
    }
    (void)s_rst->deinit();
    s_rst->~GPIO();
    s_rst = nullptr;
    return 0;
}

uint8_t st7789_interface_reset_gpio_write(uint8_t value)
{
    if (s_rst == nullptr) {
        return 1;
    }
    return (s_rst->write(value != 0U) == ESP_OK) ? 0U : 1U;
}
