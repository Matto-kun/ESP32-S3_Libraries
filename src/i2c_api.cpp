/**
 * @file        i2c_api.cpp
 * @brief       I2C master implementation for ESP32-S3
 * @author      Mat RB
 * @date        2026-08-19
 * @version     1.0.0
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#include "i2c_api.hpp"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_check.h"

static const char* TAG = "I2C";

static constexpr size_t kI2cPortCount = 2U;

static uint8_t s_bus_refcount[kI2cPortCount] = {0U, 0U};

static i2c_port_num_t toEspPort(i2c_port_id_t port)
{
    return (port == eI2cPort1) ? I2C_NUM_1 : I2C_NUM_0;
}

static size_t portIndex(i2c_port_id_t port)
{
    return (port == eI2cPort1) ? 1U : 0U;
}

I2C::I2C(const i2c_cfg_t& config)
    : m_config(config)
{
}

I2C::I2C(i2c_port_id_t port, int8_t sda_pin, int8_t scl_pin, uint32_t clk_hz)
{
    m_config.port     = port;
    m_config.sda_pin  = sda_pin;
    m_config.scl_pin  = scl_pin;
    m_config.clk_hz   = clk_hz;
}

I2C::~I2C()
{
    if (m_inited) {
        (void)deinit();
    }
}

esp_err_t I2C::init()
{
    if (m_inited) {
        ESP_LOGW(TAG, "Port %d already inited", (int)m_config.port);
        return ESP_OK;
    }

    if ((m_config.sda_pin == kI2cPinUnused) || (m_config.scl_pin == kI2cPinUnused)) {
        ESP_LOGE(TAG, "SDA and SCL pins are required");
        return ESP_ERR_INVALID_ARG;
    }

    if (m_config.clk_hz == 0U) {
        ESP_LOGE(TAG, "clk_hz must be non-zero");
        return ESP_ERR_INVALID_ARG;
    }

    const size_t idx = portIndex(m_config.port);

    if (s_bus_refcount[idx] == 0U) {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port = toEspPort(m_config.port);
        bus_cfg.sda_io_num = static_cast<gpio_num_t>(m_config.sda_pin);
        bus_cfg.scl_io_num = static_cast<gpio_num_t>(m_config.scl_pin);
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7U;
        bus_cfg.flags.enable_internal_pullup = m_config.enable_internal_pullup ? 1U : 0U;

        i2c_master_bus_handle_t bus = nullptr;
        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus), TAG, "i2c_new_master_bus failed");
        m_bus = bus;
        m_bus_owned = true;
        s_bus_refcount[idx]++;
        ESP_LOGI(TAG, "Bus port %d SDA=%d SCL=%d @ %lu Hz",
                 (int)m_config.port, (int)m_config.sda_pin, (int)m_config.scl_pin,
                 (unsigned long)m_config.clk_hz);
    } else {
        ESP_LOGE(TAG, "Port %d bus already owned by another instance (not supported yet)",
                 (int)m_config.port);
        return ESP_ERR_INVALID_STATE;
    }

    m_inited = true;
    return ESP_OK;
}

esp_err_t I2C::removeAllDevices()
{
    esp_err_t last = ESP_OK;

    for (size_t i = 0U; i < kMaxDevices; i++) {
        if (!m_devices[i].in_use) {
            continue;
        }
        auto* handle = static_cast<i2c_master_dev_handle_t>(m_devices[i].handle);
        const esp_err_t err = i2c_master_bus_rm_device(handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "rm_device addr 0x%02X: %s",
                     (unsigned)m_devices[i].addr7, esp_err_to_name(err));
            last = err;
        }
        m_devices[i].in_use = false;
        m_devices[i].handle = nullptr;
        m_devices[i].addr7  = 0U;
    }

    return last;
}

esp_err_t I2C::deinit()
{
    if (!m_inited) {
        return ESP_OK;
    }

    (void)removeAllDevices();

    if (m_bus_owned && (m_bus != nullptr)) {
        const size_t idx = portIndex(m_config.port);
        ESP_RETURN_ON_ERROR(i2c_del_master_bus(static_cast<i2c_master_bus_handle_t>(m_bus)),
                            TAG, "i2c_del_master_bus failed");
        m_bus = nullptr;
        if (s_bus_refcount[idx] > 0U) {
            s_bus_refcount[idx]--;
        }
        m_bus_owned = false;
    }

    m_inited = false;
    return ESP_OK;
}

esp_err_t I2C::getDevice(uint8_t addr7, void** out_handle)
{
    if (out_handle == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0U; i < kMaxDevices; i++) {
        if (m_devices[i].in_use && (m_devices[i].addr7 == addr7)) {
            *out_handle = m_devices[i].handle;
            return ESP_OK;
        }
    }

    for (size_t i = 0U; i < kMaxDevices; i++) {
        if (m_devices[i].in_use) {
            continue;
        }

        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address  = addr7;
        dev_cfg.scl_speed_hz    = m_config.clk_hz;

        i2c_master_dev_handle_t dev = nullptr;
        ESP_RETURN_ON_ERROR(
            i2c_master_bus_add_device(static_cast<i2c_master_bus_handle_t>(m_bus),
                                      &dev_cfg, &dev),
            TAG, "add_device 0x%02X failed", (unsigned)addr7);

        m_devices[i].addr7  = addr7;
        m_devices[i].in_use = true;
        m_devices[i].handle = dev;
        *out_handle = dev;
        ESP_LOGD(TAG, "Added device 0x%02X", (unsigned)addr7);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Device table full (max %u)", (unsigned)kMaxDevices);
    return ESP_ERR_NO_MEM;
}

esp_err_t I2C::write(uint8_t addr7, const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (!m_inited) {
        ESP_LOGE(TAG, "Not inited");
        return ESP_ERR_INVALID_STATE;
    }
    if ((len > 0U) && (data == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    void* dev = nullptr;
    ESP_RETURN_ON_ERROR(getDevice(addr7, &dev), TAG, "getDevice");

    return i2c_master_transmit(static_cast<i2c_master_dev_handle_t>(dev),
                               data, len, static_cast<int>(timeout_ms));
}

esp_err_t I2C::read(uint8_t addr7, uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (!m_inited) {
        ESP_LOGE(TAG, "Not inited");
        return ESP_ERR_INVALID_STATE;
    }
    if ((len > 0U) && (data == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    void* dev = nullptr;
    ESP_RETURN_ON_ERROR(getDevice(addr7, &dev), TAG, "getDevice");

    return i2c_master_receive(static_cast<i2c_master_dev_handle_t>(dev),
                              data, len, static_cast<int>(timeout_ms));
}

esp_err_t I2C::writeRead(uint8_t addr7,
                         const uint8_t* wr, size_t wr_len,
                         uint8_t* rd, size_t rd_len,
                         uint32_t timeout_ms)
{
    if (!m_inited) {
        ESP_LOGE(TAG, "Not inited");
        return ESP_ERR_INVALID_STATE;
    }
    if ((wr_len > 0U) && (wr == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((rd_len > 0U) && (rd == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    void* dev = nullptr;
    ESP_RETURN_ON_ERROR(getDevice(addr7, &dev), TAG, "getDevice");

    return i2c_master_transmit_receive(static_cast<i2c_master_dev_handle_t>(dev),
                                       wr, wr_len, rd, rd_len,
                                       static_cast<int>(timeout_ms));
}

const i2c_cfg_t& I2C::getConfig() const
{
    return m_config;
}

bool I2C::isInited() const
{
    return m_inited;
}
