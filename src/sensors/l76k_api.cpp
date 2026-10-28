/**
 * @file        l76k_api.cpp
 * @brief       L76K GNSS driver implementation
 * @author      Mat RB
 * @date        2026-10-28
 * @version     1.0.0
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#include "sensors/l76k_api.hpp"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char* TAG = "L76K";

static constexpr char kHexDigits[] = "0123456789ABCDEF";

static bool isRmcTalker(const char* line)
{
    if ((line == nullptr) || (line[0] != '$')) {
        return false;
    }
    return (strncmp(&line[1], l76k_regs::kTalkerGnRmc, 5) == 0) ||
           (strncmp(&line[1], l76k_regs::kTalkerGpRmc, 5) == 0);
}

/**
 * @brief Convert NMEA ddmm.mmmm to decimal degrees.
 */
static float nmeaToDegrees(float ddmm)
{
    const int degrees = static_cast<int>(ddmm / 100.0f);
    const float minutes = ddmm - (static_cast<float>(degrees) * 100.0f);
    return static_cast<float>(degrees) + (minutes / 60.0f);
}

/**
 * @brief Advance to the next comma-separated NMEA field.
 *
 * @return Pointer to first char of the field, or nullptr at end.
 */
static const char* nextField(const char* p, const char*& field_start)
{
    if (p == nullptr) {
        field_start = nullptr;
        return nullptr;
    }
    field_start = p;
    while ((*p != '\0') && (*p != ',') && (*p != '*')) {
        p++;
    }
    if (*p == ',') {
        return p + 1;
    }
    return nullptr;
}

static size_t fieldLen(const char* start, const char* next)
{
    if (start == nullptr) {
        return 0U;
    }
    if (next == nullptr) {
        size_t n = 0U;
        while ((start[n] != '\0') && (start[n] != ',') && (start[n] != '*')) {
            n++;
        }
        return n;
    }
    /* next points past the comma that ended this field */
    if (next <= start) {
        return 0U;
    }
    return static_cast<size_t>((next - 1) - start);
}

L76k::L76k(Uart& uart, const l76k_cfg_t& cfg)
    : m_uart(uart)
    , m_cfg(cfg)
{
    if (m_cfg.rx_line_max > kLineBufCap) {
        m_cfg.rx_line_max = kLineBufCap;
    }
    if (m_cfg.rx_line_max < 32U) {
        m_cfg.rx_line_max = 32U;
    }
}

L76k::~L76k()
{
    if (m_inited) {
        (void)deinit();
    }
}

esp_err_t L76k::initOptionalGpio()
{
    if (m_cfg.pps_pin == l76k_regs::kPinUnused) {
        return ESP_OK;
    }

    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << static_cast<unsigned>(m_cfg.pps_pin);
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "PPS gpio_config failed");
    m_pps_owned = true;
    return ESP_OK;
}

esp_err_t L76k::deinitOptionalGpio()
{
    if (!m_pps_owned) {
        return ESP_OK;
    }
    gpio_reset_pin(static_cast<gpio_num_t>(m_cfg.pps_pin));
    m_pps_owned = false;
    return ESP_OK;
}

esp_err_t L76k::init()
{
    if (m_inited) {
        ESP_LOGW(TAG, "Already inited");
        return ESP_OK;
    }

    if (!m_uart.isInited()) {
        ESP_LOGE(TAG, "UART not inited");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(initOptionalGpio(), TAG, "optional GPIO failed");
    (void)m_uart.flushInput();

    /* Best-effort restore of default NMEA output (MediaTek PMTK). Harmless if ignored. */
    (void)sendCommand(l76k_regs::kPmtkNmeaReduction);
    vTaskDelay(pdMS_TO_TICKS(100));

    if (m_cfg.apply_nmea_cfg) {
        esp_err_t err = sendCommand(l76k_regs::kPmtkNmeaOutput);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "NMEA output cmd failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        err = sendCommand(l76k_regs::kPmtkPosFix1s);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Pos fix cmd failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    (void)m_uart.flushInput();

    m_inited = true;
    ESP_LOGI(TAG, "Init OK (pps_pin=%d)", (int)m_cfg.pps_pin);
    return ESP_OK;
}

esp_err_t L76k::deinit()
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Not inited (deinit)");
        return ESP_OK;
    }

    (void)deinitOptionalGpio();
    m_inited = false;
    ESP_LOGI(TAG, "Deinit OK");
    return ESP_OK;
}

esp_err_t L76k::sendCommand(const char* pmtk_body, uint32_t timeout_ms)
{
    if (!m_uart.isInited()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pmtk_body == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t body_len = strlen(pmtk_body);
    if ((body_len == 0U) || (body_len > l76k_regs::kMaxCmdBodyLen)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* XOR from first char after '$' through end of body (demo algorithm). */
    uint8_t check = static_cast<uint8_t>(pmtk_body[0]);
    for (size_t i = 1U; i < body_len; i++) {
        check ^= static_cast<uint8_t>(pmtk_body[i]);
    }

    char frame[l76k_regs::kMaxCmdBodyLen + 16U];
    const int n = snprintf(frame, sizeof(frame), "$%s*%c%c\r\n",
                           pmtk_body,
                           kHexDigits[(check >> 4) & 0x0FU],
                           kHexDigits[check & 0x0FU]);
    if ((n <= 0) || (static_cast<size_t>(n) >= sizeof(frame))) {
        return ESP_ERR_INVALID_SIZE;
    }

    return m_uart.write(reinterpret_cast<const uint8_t*>(frame),
                        static_cast<size_t>(n),
                        timeout_ms);
}

esp_err_t L76k::readLine(char* buf, size_t cap, size_t& out_len, uint32_t timeout_ms)
{
    out_len = 0U;
    if ((buf == nullptr) || (cap < 2U)) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t start = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    size_t idx = 0U;
    bool saw_dollar = false;

    while (true) {
        const uint32_t now = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if ((now - start) >= timeout_ms) {
            buf[idx] = '\0';
            out_len = idx;
            return ESP_ERR_TIMEOUT;
        }

        const uint32_t remain = timeout_ms - (now - start);
        const uint32_t slice =
            (remain > 50U) ? 50U : ((remain == 0U) ? 1U : remain);

        uint8_t byte = 0U;
        const esp_err_t err = m_uart.readByte(byte, slice);
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            return err;
        }

        const char c = static_cast<char>(byte);
        if (!saw_dollar) {
            if (c != '$') {
                continue;
            }
            saw_dollar = true;
            idx = 0U;
            buf[idx++] = c;
            continue;
        }

        if ((c == '\r') || (c == '\n')) {
            buf[idx] = '\0';
            out_len = idx;
            if (idx > 0U) {
                return ESP_OK;
            }
            saw_dollar = false;
            continue;
        }

        if (idx >= (cap - 1U)) {
            /* Overflow: resync on next '$'. */
            saw_dollar = false;
            idx = 0U;
            continue;
        }
        buf[idx++] = c;
    }
}

esp_err_t L76k::parseRmc(const char* line, l76k_rmc_t& out)
{
    out = {};
    if (!isRmcTalker(line)) {
        return ESP_ERR_NOT_FOUND;
    }

    /* Skip "$GNRMC" talker to first comma. */
    const char* p = strchr(line, ',');
    if (p == nullptr) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    p++;

    const char* f = nullptr;
    /* Field 1: UTC hhmmss.sss */
    p = nextField(p, f);
    const size_t time_len = fieldLen(f, p);
    if (time_len >= 6U) {
        char th[3] = {f[0], f[1], '\0'};
        char tm[3] = {f[2], f[3], '\0'};
        char ts[3] = {f[4], f[5], '\0'};
        out.utc_h = static_cast<uint8_t>(atoi(th));
        out.utc_m = static_cast<uint8_t>(atoi(tm));
        out.utc_s = static_cast<uint8_t>(atoi(ts));
    }

    /* Field 2: status A/V */
    p = nextField(p, f);
    if ((f != nullptr) && (fieldLen(f, p) > 0U) && (f[0] == 'A')) {
        out.fix_valid = true;
    }

    /* Field 3: latitude ddmm.mmmm */
    p = nextField(p, f);
    float lat_raw = 0.0f;
    if ((f != nullptr) && (fieldLen(f, p) > 0U)) {
        lat_raw = strtof(f, nullptr);
    }

    /* Field 4: N/S */
    p = nextField(p, f);
    if ((f != nullptr) && (fieldLen(f, p) > 0U)) {
        out.lat_hemi = static_cast<char>(toupper(static_cast<unsigned char>(f[0])));
    }

    /* Field 5: longitude */
    p = nextField(p, f);
    float lon_raw = 0.0f;
    if ((f != nullptr) && (fieldLen(f, p) > 0U)) {
        lon_raw = strtof(f, nullptr);
    }

    /* Field 6: E/W */
    p = nextField(p, f);
    if ((f != nullptr) && (fieldLen(f, p) > 0U)) {
        out.lon_hemi = static_cast<char>(toupper(static_cast<unsigned char>(f[0])));
    }

    out.lat_deg = nmeaToDegrees(lat_raw);
    out.lon_deg = nmeaToDegrees(lon_raw);
    if (out.lat_hemi == 'S') {
        out.lat_deg = -out.lat_deg;
    }
    if (out.lon_hemi == 'W') {
        out.lon_deg = -out.lon_deg;
    }

    return ESP_OK;
}

esp_err_t L76k::pollRmc(l76k_rmc_t& out, uint32_t timeout_ms)
{
    out = {};
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint32_t start = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    const size_t cap =
        (m_cfg.rx_line_max < kLineBufCap) ? m_cfg.rx_line_max : kLineBufCap;

    while (true) {
        const uint32_t now = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if ((now - start) >= timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        const uint32_t remain = timeout_ms - (now - start);
        const uint32_t slice =
            (remain > m_cfg.line_timeout_ms) ? m_cfg.line_timeout_ms : remain;

        size_t line_len = 0U;
        const esp_err_t line_err = readLine(m_line, cap, line_len, slice);
        if (line_err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (line_err != ESP_OK) {
            return line_err;
        }

        l76k_rmc_t parsed = {};
        const esp_err_t parse_err = parseRmc(m_line, parsed);
        if (parse_err == ESP_ERR_NOT_FOUND) {
            continue;
        }
        if (parse_err != ESP_OK) {
            continue;
        }

        m_last = parsed;
        out = parsed;
        return ESP_OK;
    }
}

esp_err_t L76k::readPps(bool& high) const
{
    high = false;
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (m_cfg.pps_pin == l76k_regs::kPinUnused) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    high = (gpio_get_level(static_cast<gpio_num_t>(m_cfg.pps_pin)) != 0);
    return ESP_OK;
}

bool L76k::isInited() const
{
    return m_inited;
}

const l76k_rmc_t& L76k::lastRmc() const
{
    return m_last;
}
