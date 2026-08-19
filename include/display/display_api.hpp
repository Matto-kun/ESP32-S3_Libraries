/**
 * @file        display_api.hpp
 * @brief       Project façade over LibDriver ST7789 basic example API
 * @author      Mat RB
 * @date        2026-08-12
 * @version     1.0.0
 *
 * @details
 *   Wraps st7789_basic_* with esp_err_t and board/backlight ownership.
 *   Panel size is 240x240. Underlying driver: LibDriver ST7789 by Shifeng Li
 *   https://github.com/libdriver/st7789
 *
 * @platform    ESP32-S3
 */

#ifndef DISPLAY_API_HPP
#define DISPLAY_API_HPP

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "display/display_board.hpp"
#include "gpio_api.hpp"

/** RGB565 helpers */
static constexpr uint16_t kColorBlack   = 0x0000U;
static constexpr uint16_t kColorWhite   = 0xFFFFU;
static constexpr uint16_t kColorRed     = 0xF800U;
static constexpr uint16_t kColorGreen   = 0x07E0U;
static constexpr uint16_t kColorBlue    = 0x001FU;
static constexpr uint16_t kColorYellow  = 0xFFE0U;
static constexpr uint16_t kColorCyan    = 0x07FFU;
static constexpr uint16_t kColorMagenta = 0xF81FU;
static constexpr uint16_t kColorGray    = 0x8410U;
static constexpr uint16_t kColorDkGray  = 0x4208U;

class Display
{
public:
    explicit Display(const display_board_t& board);
    ~Display();

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t clear(uint16_t color = kColorBlack);
    esp_err_t fillRect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
    esp_err_t drawString(uint16_t x, uint16_t y, const char* str, uint16_t color);
    esp_err_t setBacklight(bool on);

    bool isInited() const;

private:
    display_board_t m_board  = {};
    bool            m_inited = false;
    bool            m_bl_on  = false;
    GPIO*           m_bl     = nullptr;
    alignas(GPIO) uint8_t m_bl_storage[sizeof(GPIO)];

    esp_err_t initBacklight();
    void      releaseBacklight();
};

#endif /* DISPLAY_API_HPP */
