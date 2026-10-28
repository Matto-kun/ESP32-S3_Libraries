/**
 * @file        uart_api.hpp
 * @brief       UART driver class for ESP32-S3
 * @author      Mat RB
 * @date        2026-10-28
 * @version     1.0.0
 *
 * @details
 *   Wraps the ESP-IDF UART driver behind a unified C++ interface.
 *   Supports UART ports 0–2. All ESP-IDF peripheral types are isolated
 *   to uart_api.cpp; only esp_err_t crosses this header boundary.
 *
 * @note write() / read() / readByte() block the calling task until the
 *       transfer completes or the timeout expires. Do not call from an ISR.
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#ifndef UART_API_HPP
#define UART_API_HPP

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"

static constexpr int8_t kUartPinUnused = -1;

static constexpr uint32_t kUartDefaultBaudHz = 115200U;

static constexpr int kUartDefaultRxBufBytes = 1024;

static constexpr int kUartDefaultTxBufBytes = 1024;

static constexpr uint32_t kUartDefaultTimeoutMs = 100U;

typedef enum {
    eUartPort0 = 0,
    eUartPort1 = 1,
    eUartPort2 = 2,
} uart_port_id_t;

/**
 * @brief UART driver configuration.
 *
 * Pin numbers and port must be set by the caller — no board defaults.
 * Unused RTS/CTS lines use kUartPinUnused.
 */
typedef struct {
    uart_port_id_t port         = eUartPort1;
    int8_t         tx_pin       = kUartPinUnused;
    int8_t         rx_pin       = kUartPinUnused;
    int8_t         rts_pin      = kUartPinUnused;
    int8_t         cts_pin      = kUartPinUnused;
    uint32_t       baud_hz      = kUartDefaultBaudHz;
    int            rx_buf_bytes = kUartDefaultRxBufBytes;
    int            tx_buf_bytes = kUartDefaultTxBufBytes;
} uart_cfg_t;

/**
 * @brief UART port for ESP32-S3.
 *
 * Hardware is untouched until init(). The destructor calls deinit() if active.
 */
class Uart
{
public:
    explicit Uart(const uart_cfg_t& config);
    Uart(uart_port_id_t port,
         int8_t tx_pin,
         int8_t rx_pin,
         uint32_t baud_hz = kUartDefaultBaudHz);
    ~Uart();

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t write(const uint8_t* data, size_t len,
                    uint32_t timeout_ms = kUartDefaultTimeoutMs);
    esp_err_t write(const char* text,
                    uint32_t timeout_ms = kUartDefaultTimeoutMs);

    esp_err_t read(uint8_t* data, size_t len, size_t& out_len,
                   uint32_t timeout_ms = kUartDefaultTimeoutMs);
    esp_err_t readByte(uint8_t& out_byte,
                       uint32_t timeout_ms = kUartDefaultTimeoutMs);

    int       available() const;
    esp_err_t flushInput();

    const uart_cfg_t& getConfig() const;
    bool              isInited() const;

private:
    uart_cfg_t m_config = {};
    bool       m_inited = false;
};

#endif /* UART_API_HPP */
