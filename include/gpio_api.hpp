/**
 * @file        gpio_api.hpp
 * @brief       Single-pin GPIO driver class for ESP32-S3 (digital and RTC GPIO domains)
 * @author      Mat RB
 * @date        2026-05-23
 * @version     1.0.0
 *
 * @details
 *   Wraps the ESP-IDF GPIO and RTC IO driver APIs behind a unified C++ interface.
 *   Supports standard digital GPIO (pins 0–48) and RTC GPIO (pins 0–21), selected
 *   at init() time via gpio_type in the configuration struct. All ESP-IDF peripheral
 *   types (driver/gpio.h, driver/rtc_io.h) are isolated to gpio_api.cpp; only
 *   esp_err_t crosses this header boundary.
 *
 * @platform    ESP32-S3 (ESP-IDF v6.0.1)
 */

#ifndef GPIO_API_HPP
#define GPIO_API_HPP

/* ── System Includes ─────────────────────────────────────────────────────── */
#include <stdint.h>
#include <stdbool.h>

/* ── Framework Includes ──────────────────────────────────────────────────── */
#include "esp_err.h"

/* ── Constants ───────────────────────────────────────────────────────────── */

/// Highest GPIO number that supports the RTC domain on ESP32-S3.
static constexpr uint8_t kRtcGpioMaxPinNum = 21U;

/* ── Enumerations ────────────────────────────────────────────────────────── */

/**
 * @brief GPIO hardware domain selector.
 */
typedef enum {
    eGpioTypeDigital = 0,   ///< Standard digital GPIO — full pin range (0..48)
    eGpioTypeRtc     = 1,   ///< RTC GPIO domain — restricted to pins 0..21
} gpio_type_t;

/**
 * @brief GPIO pin direction / drive mode.
 *
 * @note eGpioDirOutputOd and eGpioDirInputOutputOd are not supported by the RTC
 *       GPIO subsystem; init() returns ESP_ERR_NOT_SUPPORTED for those modes when
 *       gpio_type is eGpioTypeRtc.
 */
typedef enum {
    eGpioDirInput          = 0,   ///< Input only
    eGpioDirOutput         = 1,   ///< Push-pull output
    eGpioDirInputOutput    = 2,   ///< Simultaneous input/output (push-pull)
    eGpioDirOutputOd       = 3,   ///< Open-drain output only — digital GPIO only
    eGpioDirInputOutputOd  = 4,   ///< Open-drain output with input feedback — digital GPIO only
} gpio_direction_t;

/**
 * @brief GPIO output drive strength.
 *
 * @note Valid for digital GPIO only. setDriveStrength() returns
 *       ESP_ERR_NOT_SUPPORTED on RTC GPIO pins.
 *
 *       Maps to GPIO_DRIVE_CAP_0 through GPIO_DRIVE_CAP_3 respectively.
 *       Drive strength is applied only via setDriveStrength() — init() leaves
 *       the pin at the hardware default (GPIO_DRIVE_CAP_2, ~20 mA).
 */
typedef enum {
    eGpioDriveWeak       = 0,   ///< ~5 mA  (GPIO_DRIVE_CAP_0) — digital GPIO only
    eGpioDriveMedium     = 1,   ///< ~10 mA (GPIO_DRIVE_CAP_1) — digital GPIO only
    eGpioDriveStrong     = 2,   ///< ~20 mA (GPIO_DRIVE_CAP_2) — digital GPIO only
    eGpioDriveStrongest  = 3,   ///< ~40 mA (GPIO_DRIVE_CAP_3) — digital GPIO only
} gpio_drive_strength_t;

/**
 * @brief GPIO interrupt / wakeup trigger edge.
 */
typedef enum {
    eGpioIsrDisable    = 0,   ///< No interrupt trigger   (GPIO_INTR_DISABLE)
    eGpioIsrEdgeRise   = 1,   ///< Rising edge trigger    (GPIO_INTR_POSEDGE)
    eGpioIsrEdgeFall   = 2,   ///< Falling edge trigger   (GPIO_INTR_NEGEDGE)
    eGpioIsrEdgeAny    = 3,   ///< Any edge trigger       (GPIO_INTR_ANYEDGE)
    eGpioIsrLevelLow   = 4,   ///< Low level trigger      (GPIO_INTR_LOW_LEVEL)
    eGpioIsrLevelHigh  = 5,   ///< High level trigger     (GPIO_INTR_HIGH_LEVEL)
} gpio_isr_edge_t;

/**
 * @brief Pull resistor configuration.
 */
typedef enum {
    eGpioPullNone    = 0,   ///< No pull resistor (floating)
    eGpioPullUp      = 1,   ///< Internal pull-up enabled
    eGpioPullDown    = 2,   ///< Internal pull-down enabled
    eGpioPullUpDown  = 3,   ///< Both pull-up and pull-down enabled
} gpio_pull_t;

/// ISR callback — invoked from interrupt context. Must be minimal: no blocking, no malloc.
typedef void (*gpio_isr_callback_t)(void* arg);

/* ── Structures ──────────────────────────────────────────────────────────── */

/**
 * @brief GPIO driver configuration.
 *
 * All fields carry safe defaults; pin_num and direction are the only fields
 * that must be set explicitly for a minimal digital GPIO instance.
 *
 * Example (convenience constructor):
 * @code
 *   GPIO led(4, eGpioDirOutput);
 *   led.init();
 * @endcode
 *
 * Example (full config):
 * @code
 *   gpio_cfg_t cfg = {};
 *   cfg.pin_num    = 4;
 *   cfg.gpio_type  = eGpioTypeDigital;
 *   cfg.direction  = eGpioDirOutput;
 *   cfg.pull       = eGpioPullNone;
 *   GPIO led(cfg);
 *   led.init();
 * @endcode
 */
typedef struct {
    uint8_t          pin_num     = 0;                 ///< GPIO pin number (digital: 0..48; RTC: 0..21)
    gpio_type_t      gpio_type   = eGpioTypeDigital;  ///< GPIO domain (default: digital)
    gpio_direction_t direction   = eGpioDirInput;     ///< Pin direction / drive mode (default: input)
    gpio_pull_t      pull        = eGpioPullNone;     ///< Pull resistor (default: floating)
    uint16_t         debounce_ms = 0;                 ///< Software debounce window in ms (0 = disabled)
} gpio_cfg_t;

/* ── Class Definition ────────────────────────────────────────────────────── */

/**
 * @brief Single-pin GPIO driver for ESP32-S3, supporting digital and RTC GPIO domains.
 *
 * Hardware is untouched until init() is called. The destructor performs safe teardown
 * in all cases (including uninitialised instances).
 *
 * Example:
 * @code
 *   GPIO led(5, eGpioDirOutput);
 *   if (led.init() == ESP_OK) {
 *       led.write(true);
 *       led.toggle();
 *   }
 * @endcode
 */
class GPIO
{
public:
    explicit GPIO(const gpio_cfg_t& config);           ///< Full-config constructor; hardware untouched until init()
    GPIO(uint8_t pin_num, gpio_direction_t direction); ///< Convenience: digital, no pull, no debounce
    ~GPIO();                                           ///< Calls detachInterrupt() then deinit() if active

    // ── Core ──────────────────────────────────────────────────────────────────────
    esp_err_t init();    ///< digital: gpio_config() | RTC: rtc_gpio_init() + dir + pull
    esp_err_t deinit();  ///< digital: gpio_reset_pin() | RTC: rtc_gpio_deinit()

    // ── I/O ───────────────────────────────────────────────────────────────────────
    esp_err_t write(bool level);     ///< gpio_set_level() | rtc_gpio_set_level()
    esp_err_t read(bool& level_out); ///< Single read, or software-debounced read when debounce_ms > 0
    esp_err_t toggle();              ///< Inverts and writes the cached output level

    // ── Configuration ─────────────────────────────────────────────────────────────
    esp_err_t setDirection(gpio_direction_t direction);          ///< Branches on gpio_type
    esp_err_t setPull(gpio_pull_t pull);                         ///< Branches on gpio_type
    esp_err_t setDriveStrength(gpio_drive_strength_t strength);  ///< Digital GPIO only

    // ── ISR ───────────────────────────────────────────────────────────────────────
    esp_err_t attachInterrupt(gpio_isr_edge_t edge, gpio_isr_callback_t callback, void* arg);
    esp_err_t detachInterrupt();

    // ── Sleep / Power ─────────────────────────────────────────────────────────────
    esp_err_t enableHold();                           ///< Freeze pin during sleep — digital + RTC
    esp_err_t disableHold();                          ///< Release hold — digital + RTC
    esp_err_t enableRtcWakeup(gpio_isr_edge_t edge);  ///< Deep-sleep wakeup — RTC GPIO only
    esp_err_t disableRtcWakeup();                     ///< RTC GPIO only
    esp_err_t isolateRtc();                           ///< Isolate pad in deep-sleep — RTC GPIO only

    // ── Status ────────────────────────────────────────────────────────────────────
    const gpio_cfg_t&     getConfig()        const; ///< Returns current configuration struct
    bool                  isInited()         const; ///< Returns m_inited
    gpio_drive_strength_t getDriveStrength() const; ///< Returns cached drive strength (digital only)

private:
    gpio_cfg_t            m_config        = {};
    bool                  m_inited        = false;
    bool                  m_isr_attached  = false;
    bool                  m_last_level    = false;             ///< Cached output level for toggle()
    gpio_drive_strength_t m_drive         = eGpioDriveMedium; ///< Drive strength cache; default medium
    gpio_isr_callback_t   m_isr_callback  = nullptr;
    void*                 m_isr_arg       = nullptr;

    static bool           s_isr_service_installed;  ///< One-time global ISR service install guard

    // Implementation helpers — not part of the public API
    esp_err_t initDigital();
    esp_err_t initRtc();
    esp_err_t applyRtcPull(gpio_pull_t pull);
    bool      readRaw() const;
};

#endif /* GPIO_API_HPP */
