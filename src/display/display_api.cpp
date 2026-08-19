/**
 * @file        display_api.cpp
 * @brief       Display façade implementation over LibDriver ST7789 basic API
 * @author      Mat RB
 * @date        2026-08-12
 * @version     1.0.0
 */

#include "display/display_api.hpp"
#include "driver_st7789_basic.h"
#include "esp_log.h"

#include <new>
#include <string.h>

static const char* TAG = "DISPLAY";

Display::Display(const display_board_t& board)
    : m_board(board)
{
}

Display::~Display()
{
    if (m_inited) {
        (void)deinit();
    } else {
        releaseBacklight();
    }
}

esp_err_t Display::initBacklight()
{
    if (m_board.bl_pin < 0) {
        return ESP_OK;
    }
    if (m_bl != nullptr) {
        return ESP_OK;
    }

    gpio_cfg_t cfg = {};
    cfg.pin_num   = static_cast<uint8_t>(m_board.bl_pin);
    cfg.direction = eGpioDirOutput;
    cfg.pull      = eGpioPullNone;

    m_bl = new (m_bl_storage) GPIO(cfg);
    if (m_bl->init() != ESP_OK) {
        m_bl->~GPIO();
        m_bl = nullptr;
        ESP_LOGE(TAG, "Backlight GPIO init failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void Display::releaseBacklight()
{
    if (m_bl == nullptr) {
        return;
    }
    (void)m_bl->deinit();
    m_bl->~GPIO();
    m_bl = nullptr;
}

esp_err_t Display::init()
{
    if (m_inited) {
        return ESP_OK;
    }

    display_board_set(m_board);

    if (st7789_basic_init() != 0U) {
        ESP_LOGE(TAG, "st7789_basic_init failed");
        return ESP_FAIL;
    }

    if (initBacklight() != ESP_OK) {
        (void)st7789_basic_deinit();
        return ESP_FAIL;
    }

    if (setBacklight(true) != ESP_OK) {
        (void)st7789_basic_deinit();
        releaseBacklight();
        return ESP_FAIL;
    }

    if (st7789_basic_display_on() != 0U) {
        ESP_LOGE(TAG, "display_on failed");
        (void)setBacklight(false);
        (void)st7789_basic_deinit();
        releaseBacklight();
        return ESP_FAIL;
    }

    if (st7789_basic_clear() != 0U) {
        ESP_LOGW(TAG, "initial clear failed");
    }

    m_inited = true;
    ESP_LOGI(TAG, "Display inited 240x240");
    return ESP_OK;
}

esp_err_t Display::deinit()
{
    if (!m_inited) {
        return ESP_OK;
    }

    (void)st7789_basic_display_off();
    (void)setBacklight(false);
    (void)st7789_basic_deinit();
    releaseBacklight();
    m_inited = false;
    ESP_LOGI(TAG, "Display deinited");
    return ESP_OK;
}

esp_err_t Display::clear(uint16_t color)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (color == kColorBlack) {
        return (st7789_basic_clear() == 0U) ? ESP_OK : ESP_FAIL;
    }
    return fillRect(0, 0, 239, 239, color);
}

esp_err_t Display::fillRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    return (st7789_basic_rect(x0, y0, x1, y1, color) == 0U) ? ESP_OK : ESP_FAIL;
}

esp_err_t Display::drawString(uint16_t x, uint16_t y, const char* str, uint16_t color)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (str == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint16_t len = static_cast<uint16_t>(strlen(str));
    return (st7789_basic_string(x, y, const_cast<char*>(str), len, color, ST7789_FONT_12) == 0U)
               ? ESP_OK
               : ESP_FAIL;
}

esp_err_t Display::setBacklight(bool on)
{
    if (m_board.bl_pin < 0) {
        m_bl_on = on;
        return ESP_OK;
    }
    if (m_bl == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const esp_err_t err = m_bl->write(on);
    if (err == ESP_OK) {
        m_bl_on = on;
    }
    return err;
}

bool Display::isInited() const
{
    return m_inited;
}
