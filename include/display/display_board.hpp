/**
 * @file        display_board.hpp
 * @brief       Board-side pin/clock config for ST7789 display (caller fills)
 * @author      Mat RB
 * @date        2026-08-12
 * @version     1.0.0
 *
 * @details
 *   Pins and SPI parameters are supplied by the application — this header
 *   defines the config shape only. Values typically come from the product
 *   pinout (e.g. proto_esp32_s3 docs/pinout_gpio.json).
 *
 * @platform    ESP32-S3
 */

#ifndef DISPLAY_BOARD_HPP
#define DISPLAY_BOARD_HPP

#include <stdint.h>
#include <stddef.h>
#include "spi_api.hpp"

/**
 * @brief Display wiring and SPI parameters for the LibDriver interface.
 */
typedef struct {
    spi_host_t host     = eSpiHost2;           ///< GP-SPI host
    int8_t     mosi_pin = kSpiPinUnused;       ///< MOSI
    int8_t     sclk_pin = kSpiPinUnused;       ///< SCLK
    int8_t     dc_pin   = kSpiPinUnused;       ///< Data/Command
    int8_t     rst_pin  = kSpiPinUnused;       ///< Reset
    int8_t     bl_pin   = kSpiPinUnused;       ///< Backlight (optional; unused = no BL control)
    uint32_t   clock_hz = 20000000U;           ///< SPI clock Hz
} display_board_t;

/**
 * @brief Install board config used by st7789_interface_* callbacks.
 *
 * Must be called before display / st7789_basic_init().
 */
void display_board_set(const display_board_t& board);

/**
 * @brief Read the currently installed board config.
 */
const display_board_t& display_board_get(void);

#endif /* DISPLAY_BOARD_HPP */
