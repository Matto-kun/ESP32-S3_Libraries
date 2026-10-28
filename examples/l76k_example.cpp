/**
 * @file        l76k_example.cpp
 * @brief       Poll L76K RMC sentences over UART and log fix data
 * @author      Mat RB
 * @date        2026-10-28
 *
 * @details
 *   Pins from proto_esp32_s3/docs/pinout_gpio.json (example-local only):
 *   TX = GPIO 2, RX = GPIO 1, PPS = GPIO 42, baud = 9600.
 *   Console stays on UART0; GNSS uses UART1.
 */

#include "uart_api.hpp"
#include "sensors/l76k_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "L76K_EX";

/* MCU pins: TX drives module RX, RX reads module TX (pinout_gpio.json). */
static constexpr int8_t  kPinTx  = 2;
static constexpr int8_t  kPinRx  = 1;
static constexpr int8_t  kPinPps = 42;
static constexpr uint32_t kBaud  = 9600U;
static constexpr uint32_t kPollMs = 1000U;

extern "C" void app_main(void)
{
    uart_cfg_t uart_cfg = {};
    uart_cfg.port    = eUartPort1;
    uart_cfg.tx_pin  = kPinTx;
    uart_cfg.rx_pin  = kPinRx;
    uart_cfg.baud_hz = kBaud;

    Uart uart(uart_cfg);
    if (uart.init() != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed");
        return;
    }

    l76k_cfg_t gnss_cfg = {};
    gnss_cfg.pps_pin = kPinPps;
    gnss_cfg.apply_nmea_cfg = false;

    L76k gnss(uart, gnss_cfg);
    if (gnss.init() != ESP_OK) {
        ESP_LOGE(TAG, "L76K init failed");
        return;
    }

    ESP_LOGI(TAG, "Polling RMC every %lu ms", (unsigned long)kPollMs);

    while (true) {
        l76k_rmc_t rmc = {};
        const esp_err_t err = gnss.pollRmc(rmc, kPollMs);

        bool pps = false;
        (void)gnss.readPps(pps);

        if (err == ESP_OK) {
            ESP_LOGI(TAG,
                     "RMC valid=%d UTC=%02u:%02u:%02u lat=%.7f lon=%.7f pps=%d",
                     (int)rmc.fix_valid,
                     (unsigned)rmc.utc_h,
                     (unsigned)rmc.utc_m,
                     (unsigned)rmc.utc_s,
                     rmc.lat_deg,
                     rmc.lon_deg,
                     (int)pps);
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "No RMC (timeout) pps=%d", (int)pps);
        } else {
            ESP_LOGE(TAG, "pollRmc failed: %s", esp_err_to_name(err));
        }
    }
}
