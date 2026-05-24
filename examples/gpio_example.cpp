/**
 * @file        gpio_example.cpp
 * @brief       GPIO driver usage examples — digital output, input, ISR, and RTC GPIO
 * @author      Mat RB
 * @date        2026-05-23
 * @version     1.0.0
 *
 * @details
 *   Demonstrates all four major use-cases of the GPIO driver class:
 *
 *     1. Digital output   — LED blink via write() and toggle()
 *     2. Digital input    — software-debounced read via gpio_cfg_t.debounce_ms
 *     3. ISR input        — falling-edge interrupt with a FreeRTOS-safe counter
 *     4. RTC GPIO         — output + hold across simulated sleep
 *
 *   Called from app_main() in main.c via extern "C" linkage.
 *
 *   Pin assignments (ESP32-S3-DevKitC-1):
 *     kLedPin    GPIO 2  — general-purpose output (change to your LED pin)
 *     kButtonPin GPIO 0  — BOOT button, active-low, internal pull-up
 *     kRtcPin    GPIO 10 — RTC-capable pad for the RTC GPIO example
 *
 * @platform    ESP32-S3 (ESP-IDF v6.0.1)
 */

/* ── Project Includes ────────────────────────────────────────────────────── */
#include "gpio_api.hpp"

/* ── Framework Includes ──────────────────────────────────────────────────── */
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ── Module-local Constants ──────────────────────────────────────────────── */
static const char* TAG = "GPIO_EX";

static constexpr uint8_t  kLedPin      = 2U;    ///< Digital output — adjust to your board's LED
static constexpr uint8_t  kButtonPin   = 0U;    ///< Digital input  — BOOT button (active-low)
static constexpr uint8_t  kRtcPin      = 10U;   ///< RTC GPIO example pad
static constexpr uint32_t kBlinkMs     = 500U;  ///< Blink half-period in ms
static constexpr uint32_t kListenMs    = 3000U; ///< ISR listen window in ms

/* ── ISR State ───────────────────────────────────────────────────────────── */
static volatile uint32_t s_button_isr_count = 0U;

/**
 * @brief Falling-edge ISR for the boot button.
 *
 * Runs in IRAM — no malloc, no blocking calls. Increments a counter only.
 */
static void IRAM_ATTR button_isr_handler(void* arg)
{
    (void)arg;
    s_button_isr_count = s_button_isr_count + 1U;
}


/* ═══════════════════════════════════════════════════════════════════════════
   Example 1 — Digital Output: LED blink
   ═══════════════════════════════════════════════════════════════════════════ */

static void example_digital_output(void)
{
    ESP_LOGI(TAG, "--- Example 1: Digital output (LED blink on GPIO%d) ---", kLedPin);

    GPIO led(kLedPin, eGpioDirOutput);

    if (led.init() != ESP_OK) {
        ESP_LOGE(TAG, "LED init failed — skipping example");
        return;
    }

    for (uint8_t i = 0U; i < 6U; i++) {
        led.toggle();
        ESP_LOGI(TAG, "  LED toggled → %s", (i % 2U == 0U) ? "HIGH" : "LOW");
        vTaskDelay(pdMS_TO_TICKS(kBlinkMs));
    }

    led.write(false);  // Leave LED off
    ESP_LOGI(TAG, "  Drive strength demonstration:");
    led.setDriveStrength(eGpioDriveWeak);
    ESP_LOGI(TAG, "    Set Weak  (~5 mA)");
    led.setDriveStrength(eGpioDriveStrong);
    ESP_LOGI(TAG, "    Set Strong (~20 mA)");

    // Destructor calls deinit() — GPIO reset to default state automatically
}


/* ═══════════════════════════════════════════════════════════════════════════
   Example 2 — Digital Input: software-debounced read
   ═══════════════════════════════════════════════════════════════════════════ */

static void example_digital_input(void)
{
    ESP_LOGI(TAG, "--- Example 2: Digital input with 20 ms debounce (GPIO%d) ---", kButtonPin);

    gpio_cfg_t cfg      = {};
    cfg.pin_num         = kButtonPin;
    cfg.direction       = eGpioDirInput;
    cfg.pull            = eGpioPullUp;
    cfg.debounce_ms     = 20U;

    GPIO button(cfg);

    if (button.init() != ESP_OK) {
        ESP_LOGE(TAG, "Button init failed — skipping example");
        return;
    }

    ESP_LOGI(TAG, "  Sampling button 5 times (hold BOOT to see LOW):");
    bool level = false;
    for (uint8_t i = 0U; i < 5U; i++) {
        if (button.read(level) == ESP_OK) {
            ESP_LOGI(TAG, "  Sample %d → %s", i + 1, level ? "HIGH (released)" : "LOW (pressed)");
        }
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}


/* ═══════════════════════════════════════════════════════════════════════════
   Example 3 — ISR: falling-edge interrupt counter
   ═══════════════════════════════════════════════════════════════════════════ */

static void example_interrupt(void)
{
    ESP_LOGI(TAG, "--- Example 3: Falling-edge ISR on GPIO%d (%lu ms window) ---",
             kButtonPin, (unsigned long)kListenMs);

    gpio_cfg_t cfg  = {};
    cfg.pin_num     = kButtonPin;
    cfg.direction   = eGpioDirInput;
    cfg.pull        = eGpioPullUp;

    GPIO button(cfg);

    if (button.init() != ESP_OK) {
        ESP_LOGE(TAG, "Button init failed — skipping example");
        return;
    }

    s_button_isr_count = 0U;

    if (button.attachInterrupt(eGpioIsrEdgeFall, button_isr_handler, nullptr) != ESP_OK) {
        ESP_LOGE(TAG, "attachInterrupt failed — skipping example");
        return;
    }

    ESP_LOGI(TAG, "  Press BOOT button during the window...");
    vTaskDelay(pdMS_TO_TICKS(kListenMs));

    button.detachInterrupt();

    ESP_LOGI(TAG, "  Recorded %lu falling edge(s) via ISR",
             (unsigned long)s_button_isr_count);
}


/* ═══════════════════════════════════════════════════════════════════════════
   Example 4 — RTC GPIO: output + hold
   ═══════════════════════════════════════════════════════════════════════════ */

static void example_rtc_gpio(void)
{
    ESP_LOGI(TAG, "--- Example 4: RTC GPIO output + hold on GPIO%d ---", kRtcPin);

    gpio_cfg_t cfg  = {};
    cfg.pin_num     = kRtcPin;
    cfg.gpio_type   = eGpioTypeRtc;
    cfg.direction   = eGpioDirOutput;
    cfg.pull        = eGpioPullNone;

    GPIO rtc_out(cfg);

    if (rtc_out.init() != ESP_OK) {
        ESP_LOGE(TAG, "RTC GPIO init failed — skipping example");
        return;
    }

    rtc_out.write(true);
    ESP_LOGI(TAG, "  RTC GPIO%d driven HIGH", kRtcPin);
    vTaskDelay(pdMS_TO_TICKS(500));

    rtc_out.write(false);
    ESP_LOGI(TAG, "  RTC GPIO%d driven LOW", kRtcPin);

    // Hold latches the output level so the pad keeps its state during deep sleep
    rtc_out.enableHold();
    ESP_LOGI(TAG, "  Hold enabled — pad state preserved through deep sleep");

    rtc_out.disableHold();
    ESP_LOGI(TAG, "  Hold disabled");

    // RTC wakeup requires an input-configured pin; demonstrate the guard
    esp_err_t rc = rtc_out.enableRtcWakeup(eGpioIsrEdgeRise);
    if (rc == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGI(TAG, "  enableRtcWakeup on output-only pin returned NOT_SUPPORTED (expected)");
    }
}


/* ═══════════════════════════════════════════════════════════════════════════
   Entry point — called from app_main() in main.c
   ═══════════════════════════════════════════════════════════════════════════ */

extern "C" void gpio_example_run(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  GPIO driver example — ESP32-S3");
    ESP_LOGI(TAG, "========================================");

    example_digital_output();
    vTaskDelay(pdMS_TO_TICKS(500));

    example_digital_input();
    vTaskDelay(pdMS_TO_TICKS(500));

    example_interrupt();
    vTaskDelay(pdMS_TO_TICKS(500));

    example_rtc_gpio();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  All examples complete");
    ESP_LOGI(TAG, "========================================");
}
