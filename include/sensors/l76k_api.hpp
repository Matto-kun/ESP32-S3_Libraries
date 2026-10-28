/**
 * @file        l76k_api.hpp
 * @brief       Quectel / Waveshare L76K (L76X) GNSS driver over UART
 * @author      Mat RB
 * @date        2026-10-28
 * @version     1.0.0
 *
 * @details
 *   Uses uart_api for NMEA I/O. Parses $GNRMC / $GPRMC for UTC time and
 *   WGS84 latitude / longitude. Sends PMTK commands with XOR checksum
 *   (Waveshare L76X demo framing). Optional PPS GPIO as digital input.
 *
 * @note pollRmc() and sendCommand() block the calling task. Do not call
 *       from an ISR.
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#ifndef L76K_API_HPP
#define L76K_API_HPP

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"
#include "uart_api.hpp"
#include "sensors/l76k_regs.hpp"

typedef struct {
    int8_t   pps_pin         = l76k_regs::kPinUnused;
    int8_t   force_pin       = l76k_regs::kPinUnused;
    int8_t   standby_pin     = l76k_regs::kPinUnused;
    uint32_t line_timeout_ms = l76k_regs::kDefaultLineTimeoutMs;
    size_t   rx_line_max     = l76k_regs::kDefaultRxLineMax;
    /** When true, send PMTK314/PMTK220 (MediaTek L76X). Leave false for Quectel LC76G. */
    bool     apply_nmea_cfg  = false;
} l76k_cfg_t;

/**
 * @brief One RMC fix sample (WGS84 decimal degrees).
 */
typedef struct {
    bool     fix_valid = false;
    uint8_t  utc_h     = 0U;
    uint8_t  utc_m     = 0U;
    uint8_t  utc_s     = 0U;
    float    lat_deg   = 0.0f;
    float    lon_deg   = 0.0f;
    char     lat_hemi  = 'N';
    char     lon_hemi  = 'E';
} l76k_rmc_t;

/**
 * @brief L76K GNSS module on a UART link.
 *
 * Hardware is untouched until init(). The destructor calls deinit() if active.
 */
class L76k
{
public:
    explicit L76k(Uart& uart, const l76k_cfg_t& cfg = {});
    ~L76k();

    esp_err_t init();
    esp_err_t deinit();

    /**
     * @brief Send a PMTK body (no '$', no checksum). Adds $*HH\\r\\n.
     */
    esp_err_t sendCommand(const char* pmtk_body,
                          uint32_t timeout_ms = l76k_regs::kDefaultCmdTimeoutMs);

    /**
     * @brief Read UART until an RMC sentence is parsed or timeout expires.
     *
     * @return ESP_OK on a parsed sentence (check fix_valid for A/V status)
     * @return ESP_ERR_TIMEOUT if no RMC arrived
     */
    esp_err_t pollRmc(l76k_rmc_t& out, uint32_t timeout_ms);

    esp_err_t readPps(bool& high) const;

    bool isInited() const;
    const l76k_rmc_t& lastRmc() const;

private:
    static constexpr size_t kLineBufCap = 160U;

    Uart&      m_uart;
    l76k_cfg_t m_cfg;
    bool       m_inited    = false;
    bool       m_pps_owned = false;
    l76k_rmc_t m_last      = {};
    char       m_line[kLineBufCap] = {};

    esp_err_t initOptionalGpio();
    esp_err_t deinitOptionalGpio();
    esp_err_t readLine(char* buf, size_t cap, size_t& out_len, uint32_t timeout_ms);
    esp_err_t parseRmc(const char* line, l76k_rmc_t& out);
};

#endif /* L76K_API_HPP */
