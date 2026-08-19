/**
 * @file        display_menu_example.cpp
 * @brief       Two-level menu: root BLE/WIFI/SENSOR + SENSOR submenu
 * @author      Mat RB
 * @date        2026-08-12
 *
 * @details
 *   Pins from proto_esp32_s3/docs/pinout_gpio.json (example-local).
 *
 *   Bottom GPIO 5 = advance menu highlight
 *   Top    GPIO 6 = accept (mode icon, enter SENSOR, or back)
 *
 *   When the panel is powered from the SoC, the app must allow the rail to
 *   settle before display.init() (do not put that delay in the library).
 */

#include "display/display_api.hpp"
#include "display/layout/menu_icons.hpp"
#include "gpio_api.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MENU_EX";

static constexpr int8_t kPinMosi = 35;
static constexpr int8_t kPinSclk = 36;
static constexpr int8_t kPinDc   = 40;
static constexpr int8_t kPinRst  = 41;
static constexpr int8_t kPinBl   = 42;
static constexpr uint8_t kBtnBottom = 5; /* advance */
static constexpr uint8_t kBtnTop    = 6; /* accept */

/** App-owned settle time after SoC boot before talking to ST7789. */
static constexpr uint32_t kPanelPowerSettleMs = 150U;

enum UiLevel : uint8_t {
    eUiRoot   = 0,
    eUiSensor = 1,
};

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

    /* Panel shares SoC power — settle before ST7789 SPI/reset. */
    ESP_LOGI(TAG, "Panel power settle %lu ms", (unsigned long)kPanelPowerSettleMs);
    vTaskDelay(pdMS_TO_TICKS(kPanelPowerSettleMs));

    if (display.init() != ESP_OK) {
        ESP_LOGE(TAG, "Display init failed");
        return;
    }

    gpio_cfg_t btn_cfg = {};
    btn_cfg.direction   = eGpioDirInput;
    btn_cfg.pull        = eGpioPullUp;
    btn_cfg.debounce_ms = 20;

    btn_cfg.pin_num = kBtnBottom;
    GPIO btn_adv(btn_cfg);
    btn_cfg.pin_num = kBtnTop;
    GPIO btn_ok(btn_cfg);

    if (btn_adv.init() != ESP_OK || btn_ok.init() != ESP_OK) {
        ESP_LOGE(TAG, "Button init failed");
        return;
    }

    UiLevel level = eUiRoot;
    uint8_t root_sel   = display_layout::eMenuBle;
    uint8_t sensor_sel = display_layout::eSensorIrCam;
    bool prev_adv = true;
    bool prev_ok  = true;

    (void)display.clear(kColorBlack);
    (void)display_layout::draw_menu_bar(display, root_sel,
                                        kColorDkGray, kColorGreen,
                                        kColorWhite, kColorYellow);
    ESP_LOGI(TAG, "Menu bar drawn");

    while (true) {
        bool adv = true;
        bool ok  = true;
        (void)btn_adv.read(adv);
        (void)btn_ok.read(ok);

        if (prev_adv && !adv) {
            if (level == eUiRoot) {
                root_sel = static_cast<uint8_t>(
                    (root_sel + 1U) % display_layout::kMenuSelectCount);
                (void)display_layout::draw_menu_bar(display, root_sel,
                                                    kColorDkGray, kColorGreen,
                                                    kColorWhite, kColorYellow);
                ESP_LOGI(TAG, "Root advance -> %u", (unsigned)root_sel);
            } else {
                sensor_sel = static_cast<uint8_t>(
                    (sensor_sel + 1U) % display_layout::kSensorMenuSelectCount);
                (void)display_layout::draw_sensor_menu_bar(display, sensor_sel,
                                                           kColorDkGray, kColorGreen,
                                                           kColorWhite, kColorYellow);
                ESP_LOGI(TAG, "Sensor advance -> %u", (unsigned)sensor_sel);
            }
        }

        if (prev_ok && !ok) {
            if (level == eUiRoot) {
                if (root_sel == display_layout::eMenuFwd) {
                    ESP_LOGI(TAG, "Forward (no-op, menu full)");
                } else if (root_sel == display_layout::eMenuSensor) {
                    level = eUiSensor;
                    sensor_sel = display_layout::eSensorIrCam;
                    (void)display.fillRect(
                        display_layout::kIconBand.x0, display_layout::kIconBand.y0,
                        display_layout::kIconBand.x1, display_layout::kIconBand.y1,
                        kColorBlack);
                    (void)display_layout::draw_sensor_menu_bar(display, sensor_sel,
                                                               kColorDkGray, kColorGreen,
                                                               kColorWhite, kColorYellow);
                    ESP_LOGI(TAG, "Enter SENSOR submenu");
                } else {
                    (void)display_layout::draw_mode_icon(display, root_sel,
                                                         kColorCyan, kColorBlack);
                    ESP_LOGI(TAG, "Accept root mode %u", (unsigned)root_sel);
                }
            } else if (sensor_sel == display_layout::eSensorFwd) {
                ESP_LOGI(TAG, "Forward (no-op, menu full)");
            } else if (sensor_sel == display_layout::eSensorBack) {
                level = eUiRoot;
                (void)display.fillRect(
                    display_layout::kIconBand.x0, display_layout::kIconBand.y0,
                    display_layout::kIconBand.x1, display_layout::kIconBand.y1,
                    kColorBlack);
                (void)display_layout::draw_menu_bar(display, root_sel,
                                                    kColorDkGray, kColorGreen,
                                                    kColorWhite, kColorYellow);
                ESP_LOGI(TAG, "Back to root menu");
            } else {
                (void)display_layout::draw_sensor_mode_icon(display, sensor_sel,
                                                            kColorCyan, kColorBlack);
                ESP_LOGI(TAG, "Accept sensor mode %u", (unsigned)sensor_sel);
            }
        }

        prev_adv = adv;
        prev_ok  = ok;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
