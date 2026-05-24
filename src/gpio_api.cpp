/**
 * @file        gpio_api.cpp
 * @brief       GPIO driver implementation — digital and RTC GPIO paths for ESP32-S3
 * @author      Mat RB
 * @date        2026-05-23
 * @version     1.0.0
 *
 * @details
 *   Implements the GPIO class defined in gpio_api.hpp. All ESP-IDF peripheral types
 *   (driver/gpio.h, driver/rtc_io.h) are confined to this translation unit; the
 *   public header exposes only esp_err_t and project-owned enums/structs.
 *
 *   Digital GPIO path  — gpio_config(), gpio_set_level(), gpio_get_level(), etc.
 *   RTC GPIO path      — rtc_gpio_init(), rtc_gpio_set_level(), rtc_gpio_get_level(), etc.
 *   ISR path           — gpio_install_isr_service(), gpio_isr_handler_add/remove()
 *   Sleep/power path   — gpio_hold_en/dis(), rtc_gpio_hold_en/dis(),
 *                        rtc_gpio_wakeup_enable/disable(), rtc_gpio_isolate()
 *
 * @platform    ESP32-S3 (ESP-IDF v6.0.1)
 */

/* ── Project Includes ────────────────────────────────────────────────────── */
#include "gpio_api.hpp"

/* ── Framework Includes ──────────────────────────────────────────────────── */
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ── Module-local Constants ──────────────────────────────────────────────── */
static const char* TAG = "GPIO";

/* ── Static Member Definition ────────────────────────────────────────────── */
bool GPIO::s_isr_service_installed = false;

/* ── File-scope Type Mapping Helpers ─────────────────────────────────────── */

/**
 * @brief Map project gpio_direction_t to ESP-IDF gpio_mode_t.
 */
static gpio_mode_t toEspMode(gpio_direction_t dir)
{
    switch (dir) {
        case eGpioDirOutput:        return GPIO_MODE_OUTPUT;
        case eGpioDirInputOutput:   return GPIO_MODE_INPUT_OUTPUT;
        case eGpioDirOutputOd:      return GPIO_MODE_OUTPUT_OD;
        case eGpioDirInputOutputOd: return GPIO_MODE_INPUT_OUTPUT_OD;
        case eGpioDirInput:         /* fall-through */
        default:                    return GPIO_MODE_INPUT;
    }
}

/**
 * @brief Map project gpio_pull_t to ESP-IDF gpio_pull_mode_t.
 */
static gpio_pull_mode_t toEspPull(gpio_pull_t pull)
{
    switch (pull) {
        case eGpioPullUp:      return GPIO_PULLUP_ONLY;
        case eGpioPullDown:    return GPIO_PULLDOWN_ONLY;
        case eGpioPullUpDown:  return GPIO_PULLUP_PULLDOWN;
        case eGpioPullNone:    /* fall-through */
        default:               return GPIO_FLOATING;
    }
}

/**
 * @brief Map project gpio_drive_strength_t to ESP-IDF gpio_drive_cap_t.
 *
 * Sequential mapping: Weak=CAP_0 (~5 mA), Medium=CAP_1 (~10 mA),
 * Strong=CAP_2 (~20 mA, hardware default), Strongest=CAP_3 (~40 mA).
 */
static gpio_drive_cap_t toEspDrive(gpio_drive_strength_t strength)
{
    switch (strength) {
        case eGpioDriveWeak:       return GPIO_DRIVE_CAP_0;
        case eGpioDriveMedium:     return GPIO_DRIVE_CAP_1;
        case eGpioDriveStrong:     return GPIO_DRIVE_CAP_2;
        case eGpioDriveStrongest:  return GPIO_DRIVE_CAP_3;
        default:                   return GPIO_DRIVE_CAP_DEFAULT;
    }
}

/**
 * @brief Map project gpio_isr_edge_t to ESP-IDF gpio_int_type_t.
 */
static gpio_int_type_t toEspIntrType(gpio_isr_edge_t edge)
{
    switch (edge) {
        case eGpioIsrEdgeRise:   return GPIO_INTR_POSEDGE;
        case eGpioIsrEdgeFall:   return GPIO_INTR_NEGEDGE;
        case eGpioIsrEdgeAny:    return GPIO_INTR_ANYEDGE;
        case eGpioIsrLevelLow:   return GPIO_INTR_LOW_LEVEL;
        case eGpioIsrLevelHigh:  return GPIO_INTR_HIGH_LEVEL;
        case eGpioIsrDisable:    /* fall-through */
        default:                 return GPIO_INTR_DISABLE;
    }
}

/**
 * @brief Map project gpio_direction_t to ESP-IDF rtc_gpio_mode_t.
 */
static rtc_gpio_mode_t toRtcMode(gpio_direction_t dir)
{
    switch (dir) {
        case eGpioDirOutput:       return RTC_GPIO_MODE_OUTPUT_ONLY;
        case eGpioDirInputOutput:  return RTC_GPIO_MODE_INPUT_OUTPUT;
        case eGpioDirInput:        /* fall-through */
        default:                   return RTC_GPIO_MODE_INPUT_ONLY;
    }
}


/* ═══════════════════════════════════════════════════════════════════════════
   Constructors / Destructor
   ═══════════════════════════════════════════════════════════════════════════ */

GPIO::GPIO(const gpio_cfg_t& config)
    : m_config(config)
{
}

GPIO::GPIO(uint8_t pin_num, gpio_direction_t direction)
{
    m_config.pin_num   = pin_num;
    m_config.direction = direction;
    // gpio_cfg_t default member initialisers supply:
    //   gpio_type = eGpioTypeDigital, pull = eGpioPullNone, debounce_ms = 0
}

GPIO::~GPIO()
{
    if (m_isr_attached) {
        (void)detachInterrupt();
    }
    if (m_inited) {
        (void)deinit();
    }
}


/* ═══════════════════════════════════════════════════════════════════════════
   Core
   ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t GPIO::init()
{
    if (m_inited) {
        ESP_LOGW(TAG, "Pin %d already inited", m_config.pin_num);
        return ESP_OK;
    }

    if (m_config.gpio_type == eGpioTypeRtc) {
        return initRtc();
    }
    return initDigital();
}

esp_err_t GPIO::deinit()
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Pin %d not inited (deinit)", m_config.pin_num);
        return ESP_OK;
    }

    if (m_isr_attached) {
        (void)detachInterrupt();
    }

    if (m_config.gpio_type == eGpioTypeRtc) {
        ESP_RETURN_ON_ERROR(
            rtc_gpio_deinit((gpio_num_t)m_config.pin_num),
            TAG, "Pin %d rtc_gpio_deinit failed: %s",
            m_config.pin_num, esp_err_to_name(err_rc_));
    } else {
        ESP_RETURN_ON_ERROR(
            gpio_reset_pin((gpio_num_t)m_config.pin_num),
            TAG, "Pin %d gpio_reset_pin failed: %s",
            m_config.pin_num, esp_err_to_name(err_rc_));
    }

    m_inited = false;
    ESP_LOGI(TAG, "Pin %d deinited", m_config.pin_num);
    return ESP_OK;
}


/* ═══════════════════════════════════════════════════════════════════════════
   Private Init Helpers
   ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t GPIO::initDigital()
{
    gpio_config_t cfg = {};
    cfg.pin_bit_mask  = (1ULL << static_cast<uint32_t>(m_config.pin_num));
    cfg.mode          = toEspMode(m_config.direction);
    cfg.pull_up_en    = (m_config.pull == eGpioPullUp || m_config.pull == eGpioPullUpDown)
                        ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    cfg.pull_down_en  = (m_config.pull == eGpioPullDown || m_config.pull == eGpioPullUpDown)
                        ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    cfg.intr_type     = GPIO_INTR_DISABLE;

    ESP_RETURN_ON_ERROR(
        gpio_config(&cfg),
        TAG, "Pin %d gpio_config failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    m_inited = true;
    ESP_LOGI(TAG, "Pin %d inited (digital)", m_config.pin_num);
    return ESP_OK;
}

esp_err_t GPIO::initRtc()
{
    if (m_config.pin_num > kRtcGpioMaxPinNum) {
        ESP_LOGE(TAG, "Pin %d exceeds RTC GPIO max (%d)", m_config.pin_num, kRtcGpioMaxPinNum);
        return ESP_ERR_INVALID_ARG;
    }

    if (!rtc_gpio_is_valid_gpio((gpio_num_t)m_config.pin_num)) {
        ESP_LOGE(TAG, "Pin %d is not a valid RTC GPIO", m_config.pin_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (m_config.direction == eGpioDirOutputOd || m_config.direction == eGpioDirInputOutputOd) {
        ESP_LOGE(TAG, "Pin %d OD mode not supported on RTC GPIO (direction=%d)",
                 m_config.pin_num, (int)m_config.direction);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_RETURN_ON_ERROR(
        rtc_gpio_init((gpio_num_t)m_config.pin_num),
        TAG, "Pin %d rtc_gpio_init failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    ESP_RETURN_ON_ERROR(
        rtc_gpio_set_direction((gpio_num_t)m_config.pin_num, toRtcMode(m_config.direction)),
        TAG, "Pin %d rtc_gpio_set_direction failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    ESP_RETURN_ON_ERROR(
        applyRtcPull(m_config.pull),
        TAG, "Pin %d set RTC pull failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    m_inited = true;
    ESP_LOGI(TAG, "Pin %d inited (RTC)", m_config.pin_num);
    return ESP_OK;
}

esp_err_t GPIO::applyRtcPull(gpio_pull_t pull)
{
    gpio_num_t pin  = static_cast<gpio_num_t>(m_config.pin_num);
    bool       up   = (pull == eGpioPullUp   || pull == eGpioPullUpDown);
    bool       down = (pull == eGpioPullDown || pull == eGpioPullUpDown);

    ESP_RETURN_ON_ERROR(
        up ? rtc_gpio_pullup_en(pin) : rtc_gpio_pullup_dis(pin),
        TAG, "Pin %d RTC pullup control failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    ESP_RETURN_ON_ERROR(
        down ? rtc_gpio_pulldown_en(pin) : rtc_gpio_pulldown_dis(pin),
        TAG, "Pin %d RTC pulldown control failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    return ESP_OK;
}

bool GPIO::readRaw() const
{
    if (m_config.gpio_type == eGpioTypeRtc) {
        return rtc_gpio_get_level((gpio_num_t)m_config.pin_num) != 0U;
    }
    return gpio_get_level((gpio_num_t)m_config.pin_num) != 0;
}


/* ═══════════════════════════════════════════════════════════════════════════
   I/O
   ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t GPIO::write(bool level)
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Pin %d not inited (write)", m_config.pin_num);
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t lv = level ? 1U : 0U;

    if (m_config.gpio_type == eGpioTypeRtc) {
        ESP_RETURN_ON_ERROR(
            rtc_gpio_set_level((gpio_num_t)m_config.pin_num, lv),
            TAG, "Pin %d rtc_gpio_set_level failed: %s",
            m_config.pin_num, esp_err_to_name(err_rc_));
    } else {
        ESP_RETURN_ON_ERROR(
            gpio_set_level((gpio_num_t)m_config.pin_num, (int)lv),
            TAG, "Pin %d gpio_set_level failed: %s",
            m_config.pin_num, esp_err_to_name(err_rc_));
    }

    m_last_level = level;
    return ESP_OK;
}

esp_err_t GPIO::read(bool& level_out)
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Pin %d not inited (read)", m_config.pin_num);
        return ESP_ERR_INVALID_STATE;
    }

    if (m_config.debounce_ms == 0U) {
        level_out = readRaw();
        return ESP_OK;
    }

    bool     stable = readRaw();
    uint16_t count  = 0U;

    while (count < m_config.debounce_ms) {
        vTaskDelay(pdMS_TO_TICKS(1));
        bool current = readRaw();
        if (current != stable) {
            stable = current;
            count  = 0U;
        } else {
            count++;
        }
    }

    level_out = stable;
    return ESP_OK;
}

esp_err_t GPIO::toggle()
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Pin %d not inited (toggle)", m_config.pin_num);
        return ESP_ERR_INVALID_STATE;
    }
    return write(!m_last_level);
}


/* ═══════════════════════════════════════════════════════════════════════════
   Configuration
   ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t GPIO::setDirection(gpio_direction_t direction)
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Pin %d not inited (setDirection)", m_config.pin_num);
        return ESP_ERR_INVALID_STATE;
    }

    if (m_config.gpio_type != eGpioTypeRtc) {
        ESP_RETURN_ON_ERROR(
            gpio_set_direction((gpio_num_t)m_config.pin_num, toEspMode(direction)),
            TAG, "Pin %d gpio_set_direction failed: %s",
            m_config.pin_num, esp_err_to_name(err_rc_));
        m_config.direction = direction;
        ESP_LOGI(TAG, "Pin %d direction set to %d", m_config.pin_num, (int)direction);
        return ESP_OK;
    }

    // RTC path
    if (direction == eGpioDirOutputOd || direction == eGpioDirInputOutputOd) {
        ESP_LOGE(TAG, "Pin %d OD mode not supported on RTC GPIO", m_config.pin_num);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_RETURN_ON_ERROR(
        rtc_gpio_set_direction((gpio_num_t)m_config.pin_num, toRtcMode(direction)),
        TAG, "Pin %d rtc_gpio_set_direction failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    m_config.direction = direction;
    ESP_LOGI(TAG, "Pin %d direction set to %d (RTC)", m_config.pin_num, (int)direction);
    return ESP_OK;
}

esp_err_t GPIO::setPull(gpio_pull_t pull)
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Pin %d not inited (setPull)", m_config.pin_num);
        return ESP_ERR_INVALID_STATE;
    }

    if (m_config.gpio_type != eGpioTypeRtc) {
        ESP_RETURN_ON_ERROR(
            gpio_set_pull_mode((gpio_num_t)m_config.pin_num, toEspPull(pull)),
            TAG, "Pin %d gpio_set_pull_mode failed: %s",
            m_config.pin_num, esp_err_to_name(err_rc_));
        m_config.pull = pull;
        ESP_LOGI(TAG, "Pin %d pull set to %d", m_config.pin_num, (int)pull);
        return ESP_OK;
    }

    // RTC path
    ESP_RETURN_ON_ERROR(
        applyRtcPull(pull),
        TAG, "Pin %d set RTC pull failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    m_config.pull = pull;
    ESP_LOGI(TAG, "Pin %d pull set to %d (RTC)", m_config.pin_num, (int)pull);
    return ESP_OK;
}

esp_err_t GPIO::setDriveStrength(gpio_drive_strength_t strength)
{
    if (m_config.gpio_type == eGpioTypeRtc) {
        ESP_LOGE(TAG, "Pin %d setDriveStrength not supported on RTC GPIO", m_config.pin_num);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!m_inited) {
        ESP_LOGW(TAG, "Pin %d not inited (setDriveStrength)", m_config.pin_num);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(
        gpio_set_drive_capability((gpio_num_t)m_config.pin_num, toEspDrive(strength)),
        TAG, "Pin %d gpio_set_drive_capability failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    m_drive = strength;
    ESP_LOGI(TAG, "Pin %d drive strength set to %d", m_config.pin_num, (int)strength);
    return ESP_OK;
}


/* ═══════════════════════════════════════════════════════════════════════════
   ISR
   ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t GPIO::attachInterrupt(gpio_isr_edge_t edge, gpio_isr_callback_t callback, void* arg)
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Pin %d not inited (attachInterrupt)", m_config.pin_num);
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_isr_service_installed) {
        ESP_RETURN_ON_ERROR(
            gpio_install_isr_service(0),
            TAG, "Pin %d gpio_install_isr_service failed: %s",
            m_config.pin_num, esp_err_to_name(err_rc_));
        s_isr_service_installed = true;
    }

    ESP_RETURN_ON_ERROR(
        gpio_set_intr_type((gpio_num_t)m_config.pin_num, toEspIntrType(edge)),
        TAG, "Pin %d gpio_set_intr_type failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add((gpio_num_t)m_config.pin_num, (gpio_isr_t)callback, arg),
        TAG, "Pin %d gpio_isr_handler_add failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    ESP_RETURN_ON_ERROR(
        gpio_intr_enable((gpio_num_t)m_config.pin_num),
        TAG, "Pin %d gpio_intr_enable failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    m_isr_callback = callback;
    m_isr_arg      = arg;
    m_isr_attached = true;

    ESP_LOGI(TAG, "Pin %d ISR attached, edge=%d", m_config.pin_num, (int)edge);
    return ESP_OK;
}

esp_err_t GPIO::detachInterrupt()
{
    if (!m_isr_attached) {
        ESP_LOGW(TAG, "Pin %d no ISR attached (detach)", m_config.pin_num);
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(
        gpio_intr_disable((gpio_num_t)m_config.pin_num),
        TAG, "Pin %d gpio_intr_disable failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_remove((gpio_num_t)m_config.pin_num),
        TAG, "Pin %d gpio_isr_handler_remove failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    m_isr_attached = false;
    m_isr_callback = nullptr;
    m_isr_arg      = nullptr;

    return ESP_OK;
}


/* ═══════════════════════════════════════════════════════════════════════════
   Sleep / Power
   ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t GPIO::enableHold()
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Pin %d not inited (enableHold)", m_config.pin_num);
        return ESP_ERR_INVALID_STATE;
    }

    if (m_config.gpio_type == eGpioTypeRtc) {
        ESP_RETURN_ON_ERROR(
            rtc_gpio_hold_en((gpio_num_t)m_config.pin_num),
            TAG, "Pin %d rtc_gpio_hold_en failed: %s",
            m_config.pin_num, esp_err_to_name(err_rc_));
    } else {
        ESP_RETURN_ON_ERROR(
            gpio_hold_en((gpio_num_t)m_config.pin_num),
            TAG, "Pin %d gpio_hold_en failed: %s",
            m_config.pin_num, esp_err_to_name(err_rc_));
    }

    ESP_LOGI(TAG, "Pin %d hold enabled", m_config.pin_num);
    return ESP_OK;
}

esp_err_t GPIO::disableHold()
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Pin %d not inited (disableHold)", m_config.pin_num);
        return ESP_ERR_INVALID_STATE;
    }

    if (m_config.gpio_type == eGpioTypeRtc) {
        ESP_RETURN_ON_ERROR(
            rtc_gpio_hold_dis((gpio_num_t)m_config.pin_num),
            TAG, "Pin %d rtc_gpio_hold_dis failed: %s",
            m_config.pin_num, esp_err_to_name(err_rc_));
    } else {
        ESP_RETURN_ON_ERROR(
            gpio_hold_dis((gpio_num_t)m_config.pin_num),
            TAG, "Pin %d gpio_hold_dis failed: %s",
            m_config.pin_num, esp_err_to_name(err_rc_));
    }

    ESP_LOGI(TAG, "Pin %d hold disabled", m_config.pin_num);
    return ESP_OK;
}

esp_err_t GPIO::enableRtcWakeup(gpio_isr_edge_t edge)
{
    if (m_config.gpio_type != eGpioTypeRtc) {
        ESP_LOGE(TAG, "Pin %d enableRtcWakeup not supported on digital GPIO", m_config.pin_num);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!m_inited) {
        ESP_LOGW(TAG, "Pin %d not inited (enableRtcWakeup)", m_config.pin_num);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(
        rtc_gpio_wakeup_enable((gpio_num_t)m_config.pin_num, toEspIntrType(edge)),
        TAG, "Pin %d rtc_gpio_wakeup_enable failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    ESP_LOGI(TAG, "Pin %d RTC wakeup enabled, edge=%d", m_config.pin_num, (int)edge);
    return ESP_OK;
}

esp_err_t GPIO::disableRtcWakeup()
{
    if (m_config.gpio_type != eGpioTypeRtc) {
        ESP_LOGE(TAG, "Pin %d disableRtcWakeup not supported on digital GPIO", m_config.pin_num);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!m_inited) {
        ESP_LOGW(TAG, "Pin %d not inited (disableRtcWakeup)", m_config.pin_num);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(
        rtc_gpio_wakeup_disable((gpio_num_t)m_config.pin_num),
        TAG, "Pin %d rtc_gpio_wakeup_disable failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    return ESP_OK;
}

esp_err_t GPIO::isolateRtc()
{
    if (m_config.gpio_type != eGpioTypeRtc) {
        ESP_LOGE(TAG, "Pin %d isolateRtc not supported on digital GPIO", m_config.pin_num);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!m_inited) {
        ESP_LOGW(TAG, "Pin %d not inited (isolateRtc)", m_config.pin_num);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(
        rtc_gpio_isolate((gpio_num_t)m_config.pin_num),
        TAG, "Pin %d rtc_gpio_isolate failed: %s",
        m_config.pin_num, esp_err_to_name(err_rc_));

    ESP_LOGI(TAG, "Pin %d RTC pad isolated", m_config.pin_num);
    return ESP_OK;
}


/* ═══════════════════════════════════════════════════════════════════════════
   Status
   ═══════════════════════════════════════════════════════════════════════════ */

const gpio_cfg_t& GPIO::getConfig() const
{
    return m_config;
}

bool GPIO::isInited() const
{
    return m_inited;
}

gpio_drive_strength_t GPIO::getDriveStrength() const
{
    return m_drive;
}
