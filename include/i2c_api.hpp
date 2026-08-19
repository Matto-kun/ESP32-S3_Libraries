/**
 * @file        i2c_api.hpp
 * @brief       I2C master driver class for ESP32-S3
 * @author      Mat RB
 * @date        2026-08-19
 * @version     1.0.0
 *
 * @details
 *   Wraps the ESP-IDF I2C master driver (driver/i2c_master.h) behind a
 *   unified C++ interface. Supports I2C port 0 and 1. Per-slave device
 *   handles are created on first use for each 7-bit address.
 *   All ESP-IDF peripheral types are isolated to i2c_api.cpp.
 *
 * @note write() / read() / writeRead() block the calling task until the
 *       transfer completes. Do not call from an ISR.
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#ifndef I2C_API_HPP
#define I2C_API_HPP

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"

static constexpr int8_t kI2cPinUnused = -1;

static constexpr uint32_t kI2cDefaultClockHz = 100000U;

static constexpr uint32_t kI2cDefaultTimeoutMs = 100U;

typedef enum {
    eI2cPort0 = 0,
    eI2cPort1 = 1,
} i2c_port_id_t;

typedef struct {
    i2c_port_id_t port                   = eI2cPort0;
    int8_t          sda_pin              = kI2cPinUnused;
    int8_t          scl_pin              = kI2cPinUnused;
    uint32_t        clk_hz               = kI2cDefaultClockHz;
    bool            enable_internal_pullup = true;
} i2c_cfg_t;

/**
 * @brief I2C master bus for ESP32-S3.
 *
 * Hardware is untouched until init(). The destructor calls deinit() if active.
 */
class I2C
{
public:
    explicit I2C(const i2c_cfg_t& config);
    I2C(i2c_port_id_t port,
        int8_t sda_pin,
        int8_t scl_pin,
        uint32_t clk_hz = kI2cDefaultClockHz);
    ~I2C();

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t write(uint8_t addr7, const uint8_t* data, size_t len,
                    uint32_t timeout_ms = kI2cDefaultTimeoutMs);
    esp_err_t read(uint8_t addr7, uint8_t* data, size_t len,
                   uint32_t timeout_ms = kI2cDefaultTimeoutMs);
    esp_err_t writeRead(uint8_t addr7,
                        const uint8_t* wr, size_t wr_len,
                        uint8_t* rd, size_t rd_len,
                        uint32_t timeout_ms = kI2cDefaultTimeoutMs);

    const i2c_cfg_t& getConfig() const;
    bool             isInited() const;

private:
    static constexpr size_t kMaxDevices = 8U;

    struct DeviceSlot {
        uint8_t addr7  = 0U;
        bool    in_use = false;
        void*   handle = nullptr;
    };

    i2c_cfg_t   m_config     = {};
    bool        m_inited     = false;
    bool        m_bus_owned  = false;
    void*       m_bus        = nullptr;
    DeviceSlot  m_devices[kMaxDevices] = {};

    esp_err_t getDevice(uint8_t addr7, void** out_handle);
    esp_err_t removeAllDevices();
};

#endif /* I2C_API_HPP */
