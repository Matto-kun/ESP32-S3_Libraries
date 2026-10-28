/**
 * @file        hmc5983_api.cpp
 * @brief       HMC5983 magnetometer implementation
 * @author      Mat RB
 * @date        2026-08-31
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#include "sensors/hmc5983_api.hpp"

#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "HMC5983";

Hmc5983::Hmc5983(I2C& bus, const hmc5983_cfg_t& cfg)
    : m_bus(bus)
    , m_cfg(cfg)
{
}

Hmc5983::Hmc5983(I2C& bus, uint8_t addr7)
    : m_bus(bus)
{
    m_cfg.addr7 = addr7;
}

esp_err_t Hmc5983::writeReg(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = {reg, val};
    return m_bus.write(m_cfg.addr7, buf, sizeof(buf));
}

esp_err_t Hmc5983::readReg(uint8_t reg, uint8_t& val)
{
    return m_bus.writeRead(m_cfg.addr7, &reg, 1U, &val, 1U, m_cfg.poll_timeout_ms);
}

esp_err_t Hmc5983::readRegs(uint8_t reg, uint8_t* buf, size_t len)
{
    return m_bus.writeRead(m_cfg.addr7, &reg, 1U, buf, len, m_cfg.poll_timeout_ms);
}

esp_err_t Hmc5983::verifyId()
{
    uint8_t id[3] = {0U, 0U, 0U};

    ESP_RETURN_ON_ERROR(readReg(hmc5983_regs::kRegIdA, id[0]), TAG, "read ID A");
    ESP_RETURN_ON_ERROR(readReg(hmc5983_regs::kRegIdB, id[1]), TAG, "read ID B");
    ESP_RETURN_ON_ERROR(readReg(hmc5983_regs::kRegIdC, id[2]), TAG, "read ID C");

    if ((id[0] != hmc5983_regs::kIdAExpected) ||
        (id[1] != hmc5983_regs::kIdBExpected) ||
        (id[2] != hmc5983_regs::kIdCExpected)) {
        ESP_LOGE(TAG, "ID mismatch: got %c%c%c expected H43",
                 (char)id[0], (char)id[1], (char)id[2]);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "ID OK (H43) addr 0x%02X", (unsigned)m_cfg.addr7);
    return ESP_OK;
}

esp_err_t Hmc5983::applyConfig()
{
    const uint8_t cra = hmc5983_regs::buildCra(
        m_cfg.temp_sensor, m_cfg.samples, m_cfg.odr, m_cfg.meas);
    const uint8_t crb = hmc5983_regs::buildCrb(m_cfg.gain);
    const uint8_t mode = hmc5983_regs::buildMode(m_cfg.mode);

    ESP_RETURN_ON_ERROR(writeReg(hmc5983_regs::kRegCra, cra), TAG, "write CRA");
    ESP_RETURN_ON_ERROR(writeReg(hmc5983_regs::kRegCrb, crb), TAG, "write CRB");
    ESP_RETURN_ON_ERROR(writeReg(hmc5983_regs::kRegMode, mode), TAG, "write Mode");

    m_lsb_per_gauss = hmc5983_regs::lsbPerGauss(m_cfg.gain);
    return ESP_OK;
}

esp_err_t Hmc5983::init()
{
    if (m_inited) {
        ESP_LOGW(TAG, "Already inited");
        return ESP_OK;
    }

    if (!m_bus.isInited()) {
        ESP_LOGE(TAG, "I2C bus not inited");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(verifyId(), TAG, "verifyId");
    ESP_RETURN_ON_ERROR(applyConfig(), TAG, "applyConfig");

    if (m_cfg.mode == hmc5983_regs::eHmc5983ModeContinuous) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    m_inited = true;
    return ESP_OK;
}

esp_err_t Hmc5983::deinit()
{
    if (!m_inited) {
        return ESP_OK;
    }

    const esp_err_t err = writeReg(hmc5983_regs::kRegMode,
                                   hmc5983_regs::buildMode(hmc5983_regs::eHmc5983ModeIdle));
    m_inited = false;
    return err;
}

esp_err_t Hmc5983::waitReady()
{
    uint8_t status = 0U;
    ESP_RETURN_ON_ERROR(readReg(hmc5983_regs::kRegStatus, status), TAG, "read Status");

    if ((status & hmc5983_regs::kStatusRdy) != 0U) {
        return ESP_OK;
    }

    if (m_cfg.mode == hmc5983_regs::eHmc5983ModeContinuous) {
        return ESP_ERR_NOT_FINISHED;
    }

    const TickType_t start = xTaskGetTickCount();
    const TickType_t limit = pdMS_TO_TICKS(m_cfg.poll_timeout_ms);

    while ((xTaskGetTickCount() - start) < limit) {
        ESP_RETURN_ON_ERROR(readReg(hmc5983_regs::kRegStatus, status), TAG, "read Status");
        if ((status & hmc5983_regs::kStatusRdy) != 0U) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    ESP_LOGE(TAG, "Status RDY timeout (%lu ms)",
             (unsigned long)m_cfg.poll_timeout_ms);
    return ESP_ERR_TIMEOUT;
}

esp_err_t Hmc5983::readRaw(hmc5983_raw_t& out)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    if (m_cfg.mode == hmc5983_regs::eHmc5983ModeSingle) {
        ESP_RETURN_ON_ERROR(
            writeReg(hmc5983_regs::kRegMode,
                     hmc5983_regs::buildMode(hmc5983_regs::eHmc5983ModeSingle)),
            TAG, "trigger single measure");
    }

    const esp_err_t ready = waitReady();
    if (ready != ESP_OK) {
        return ready;
    }

    uint8_t data[6] = {0U};
    ESP_RETURN_ON_ERROR(readRegs(hmc5983_regs::kRegDataX, data, sizeof(data)),
                       TAG, "read data");

    out.x = hmc5983_regs::assembleRaw(data[0], data[1]);
    out.z = hmc5983_regs::assembleRaw(data[2], data[3]);
    out.y = hmc5983_regs::assembleRaw(data[4], data[5]);

    if ((out.x == hmc5983_regs::kOverflowRaw) ||
        (out.y == hmc5983_regs::kOverflowRaw) ||
        (out.z == hmc5983_regs::kOverflowRaw)) {
        ESP_LOGW(TAG, "ADC overflow");
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t Hmc5983::readGauss(hmc5983_gauss_t& out)
{
    hmc5983_raw_t raw = {};
    const esp_err_t err = readRaw(raw);
    if (err != ESP_OK) {
        return err;
    }

    if (m_lsb_per_gauss == 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    const float scale = 1.0f / static_cast<float>(m_lsb_per_gauss);
    out.x = static_cast<float>(raw.x) * scale;
    out.y = static_cast<float>(raw.y) * scale;
    out.z = static_cast<float>(raw.z) * scale;
    return ESP_OK;
}

esp_err_t Hmc5983::readTemperature(float& celsius)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!m_cfg.temp_sensor) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[2] = {0U, 0U};
    ESP_RETURN_ON_ERROR(
        readRegs(hmc5983_regs::kRegTempH, data, sizeof(data)),
        TAG, "read temperature");

    const int16_t raw = hmc5983_regs::assembleRaw(data[0], data[1]);
    celsius = hmc5983_regs::temperatureCelsius(raw);
    return ESP_OK;
}

bool Hmc5983::isInited() const
{
    return m_inited;
}
