/**
 * @file        display_driver_smoke.cpp
 * @brief       Flashable smoke test for ST7789 + spi_api (static menu bar)
 * @author      Mat RB
 * @date        2026-08-12
 *
 * @details
 *   Pins from proto_esp32_s3 pinout_gpio.json (example-local only).
 *   Draws a static bottom bar; button UX is deferred to examples/.
 */

#include "display/display_api.hpp"
#include "display/layout/menu_icons.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "SMOKE";

/* Example-local pinout (proto_esp32_s3/docs/pinout_gpio.json) */
static constexpr int8_t kPinMosi = 35;
static constexpr int8_t kPinSclk = 36;
static constexpr int8_t kPinDc   = 40;
static constexpr int8_t kPinRst  = 41;
static constexpr int8_t kPinBl   = 42;

extern "C" void app_main(void)
{
    display_board_t board = {};
    board.host     = eSpiHost2;
    board.mosi_pin = kPinMosi;
    board.sclk_pin = kPinSclk;
    board.dc_pin   = kPinDc;
    board.rst_pin  = kPinRst;
    board.bl_pin   = kPinBl;
    board.clock_hz = 20000000U;

    Display display(board);
    if (display.init() != ESP_OK) {
        ESP_LOGE(TAG, "Display init FAILED");
        return;
    }

    ESP_LOGI(TAG, "Display init OK — drawing static menu bar");

    (void)display.clear(kColorBlack);
    (void)display_layout::draw_menu_bar(display, display_layout::eMenuBle,
                                        kColorDkGray, kColorBlue,
                                        kColorWhite, kColorYellow);
    (void)display.drawString(8, 8, "ST7789 smoke", kColorWhite);

    ESP_LOGI(TAG, "Smoke draw complete — idle");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
