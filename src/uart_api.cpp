/**
 * @file        uart_api.cpp
 * @brief       UART driver implementation for ESP32-S3
 * @author      Mat RB
 * @date        2026-10-28
 * @version     1.0.0
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#include "uart_api.hpp"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static const char* TAG = "UART";

static constexpr size_t kUartPortCount = 3U;

static bool s_port_in_use[kUartPortCount] = {false, false, false};

static uart_port_t toEspPort(uart_port_id_t port)
{
    switch (port) {
    case eUartPort1:
        return UART_NUM_1;
    case eUartPort2:
        return UART_NUM_2;
    case eUartPort0:
    default:
        return UART_NUM_0;
    }
}

static size_t portIndex(uart_port_id_t port)
{
    switch (port) {
    case eUartPort1:
        return 1U;
    case eUartPort2:
        return 2U;
    case eUartPort0:
    default:
        return 0U;
    }
}

static TickType_t toTicks(uint32_t timeout_ms)
{
    if (timeout_ms == 0U) {
        return 0;
    }
    return pdMS_TO_TICKS(timeout_ms);
}

Uart::Uart(const uart_cfg_t& config)
    : m_config(config)
{
}

Uart::Uart(uart_port_id_t port, int8_t tx_pin, int8_t rx_pin, uint32_t baud_hz)
{
    m_config.port    = port;
    m_config.tx_pin  = tx_pin;
    m_config.rx_pin  = rx_pin;
    m_config.baud_hz = baud_hz;
}

Uart::~Uart()
{
    if (m_inited) {
        (void)deinit();
    }
}

esp_err_t Uart::init()
{
    if (m_inited) {
        ESP_LOGW(TAG, "Port %d already inited", (int)m_config.port);
        return ESP_OK;
    }

    if ((m_config.tx_pin == kUartPinUnused) || (m_config.rx_pin == kUartPinUnused)) {
        ESP_LOGE(TAG, "TX and RX pins are required");
        return ESP_ERR_INVALID_ARG;
    }

    if (m_config.baud_hz == 0U) {
        ESP_LOGE(TAG, "baud_hz must be non-zero");
        return ESP_ERR_INVALID_ARG;
    }

    if ((m_config.rx_buf_bytes <= 0) || (m_config.tx_buf_bytes < 0)) {
        ESP_LOGE(TAG, "Invalid buffer sizes rx=%d tx=%d",
                 m_config.rx_buf_bytes, m_config.tx_buf_bytes);
        return ESP_ERR_INVALID_ARG;
    }

    const size_t idx = portIndex(m_config.port);
    if (s_port_in_use[idx]) {
        ESP_LOGE(TAG, "Port %d already owned by another instance",
                 (int)m_config.port);
        return ESP_ERR_INVALID_STATE;
    }

    const uart_port_t esp_port = toEspPort(m_config.port);

    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate  = static_cast<int>(m_config.baud_hz);
    uart_cfg.data_bits  = UART_DATA_8_BITS;
    uart_cfg.parity     = UART_PARITY_DISABLE;
    uart_cfg.stop_bits  = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_RETURN_ON_ERROR(
        uart_driver_install(esp_port,
                            m_config.rx_buf_bytes,
                            m_config.tx_buf_bytes,
                            0,
                            nullptr,
                            0),
        TAG, "uart_driver_install failed");

    esp_err_t err = uart_param_config(esp_port, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        (void)uart_driver_delete(esp_port);
        return err;
    }

    const int rts = (m_config.rts_pin == kUartPinUnused)
                        ? UART_PIN_NO_CHANGE
                        : static_cast<int>(m_config.rts_pin);
    const int cts = (m_config.cts_pin == kUartPinUnused)
                        ? UART_PIN_NO_CHANGE
                        : static_cast<int>(m_config.cts_pin);

    err = uart_set_pin(esp_port,
                       static_cast<int>(m_config.tx_pin),
                       static_cast<int>(m_config.rx_pin),
                       rts,
                       cts);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        (void)uart_driver_delete(esp_port);
        return err;
    }

    s_port_in_use[idx] = true;
    m_inited = true;
    ESP_LOGI(TAG, "Port %d TX=%d RX=%d @ %lu baud",
             (int)m_config.port,
             (int)m_config.tx_pin,
             (int)m_config.rx_pin,
             (unsigned long)m_config.baud_hz);
    return ESP_OK;
}

esp_err_t Uart::deinit()
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Port %d not inited (deinit)", (int)m_config.port);
        return ESP_OK;
    }

    const uart_port_t esp_port = toEspPort(m_config.port);
    ESP_RETURN_ON_ERROR(uart_driver_delete(esp_port),
                        TAG, "uart_driver_delete failed");

    s_port_in_use[portIndex(m_config.port)] = false;
    m_inited = false;
    ESP_LOGI(TAG, "Port %d deinit OK", (int)m_config.port);
    return ESP_OK;
}

esp_err_t Uart::write(const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((data == nullptr) && (len > 0U)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len == 0U) {
        return ESP_OK;
    }

    const uart_port_t esp_port = toEspPort(m_config.port);
    const int written = uart_write_bytes(esp_port,
                                         reinterpret_cast<const char*>(data),
                                         len);
    if (written < 0) {
        return ESP_FAIL;
    }
    if (static_cast<size_t>(written) != len) {
        ESP_LOGE(TAG, "Port %d short write %d/%u",
                 (int)m_config.port, written, (unsigned)len);
        return ESP_ERR_INVALID_SIZE;
    }

    const esp_err_t err = uart_wait_tx_done(esp_port, toTicks(timeout_ms));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Port %d wait_tx_done: %s",
                 (int)m_config.port, esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t Uart::write(const char* text, uint32_t timeout_ms)
{
    if (text == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return write(reinterpret_cast<const uint8_t*>(text),
                 strlen(text),
                 timeout_ms);
}

esp_err_t Uart::read(uint8_t* data, size_t len, size_t& out_len,
                     uint32_t timeout_ms)
{
    out_len = 0U;
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((data == nullptr) || (len == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    const uart_port_t esp_port = toEspPort(m_config.port);
    const int n = uart_read_bytes(esp_port, data, len, toTicks(timeout_ms));
    if (n < 0) {
        return ESP_FAIL;
    }
    out_len = static_cast<size_t>(n);
    if (out_len == 0U) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t Uart::readByte(uint8_t& out_byte, uint32_t timeout_ms)
{
    size_t n = 0U;
    const esp_err_t err = read(&out_byte, 1U, n, timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (n != 1U) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

int Uart::available() const
{
    if (!m_inited) {
        return 0;
    }
    size_t cached = 0U;
    const esp_err_t err = uart_get_buffered_data_len(
        toEspPort(m_config.port), &cached);
    if (err != ESP_OK) {
        return 0;
    }
    return static_cast<int>(cached);
}

esp_err_t Uart::flushInput()
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    return uart_flush_input(toEspPort(m_config.port));
}

const uart_cfg_t& Uart::getConfig() const
{
    return m_config;
}

bool Uart::isInited() const
{
    return m_inited;
}
