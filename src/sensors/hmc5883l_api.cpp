/**
 * @file        hmc5883l_api.cpp
 * @brief       HMC5883L magnetometer implementation
 * @author      Mat RB
 * @date        2026-08-19
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#include "sensors/hmc5883l_api.hpp"

#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "HMC5883L";

Hmc5883l::Hmc5883l(I2C& bus, const hmc5883l_cfg_t& cfg)
    : m_bus(bus)
    , m_cfg(cfg)
{
}

Hmc5883l::Hmc5883l(I2C& bus, uint8_t addr7)
    : m_bus(bus)
{
    m_cfg.addr7 = addr7;
}

esp_err_t Hmc5883l::writeReg(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = {reg, val};
    return m_bus.write(m_cfg.addr7, buf, sizeof(buf));
}

esp_err_t Hmc5883l::readReg(uint8_t reg, uint8_t& val)
{
    return m_bus.writeRead(m_cfg.addr7, &reg, 1U, &val, 1U, m_cfg.poll_timeout_ms);
}

esp_err_t Hmc5883l::readRegs(uint8_t reg, uint8_t* buf, size_t len)
{
    return m_bus.writeRead(m_cfg.addr7, &reg, 1U, buf, len, m_cfg.poll_timeout_ms);
}

esp_err_t Hmc5883l::verifyId()
{
    uint8_t id[3] = {0U, 0U, 0U};

    ESP_RETURN_ON_ERROR(readReg(hmc5883l_regs::kRegIdA, id[0]), TAG, "read ID A");
    ESP_RETURN_ON_ERROR(readReg(hmc5883l_regs::kRegIdB, id[1]), TAG, "read ID B");
    ESP_RETURN_ON_ERROR(readReg(hmc5883l_regs::kRegIdC, id[2]), TAG, "read ID C");

    if ((id[0] != hmc5883l_regs::kIdAExpected) ||
        (id[1] != hmc5883l_regs::kIdBExpected) ||
        (id[2] != hmc5883l_regs::kIdCExpected)) {
        ESP_LOGE(TAG, "ID mismatch: got %c%c%c expected H43",
                 (char)id[0], (char)id[1], (char)id[2]);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "ID OK (H43) addr 0x%02X", (unsigned)m_cfg.addr7);
    return ESP_OK;
}

esp_err_t Hmc5883l::applyConfig()
{
    const uint8_t cra  = hmc5883l_regs::buildCra(m_cfg.samples, m_cfg.odr);
    const uint8_t crb  = hmc5883l_regs::buildCrb(m_cfg.gain);
    const uint8_t mode = hmc5883l_regs::buildMode(m_cfg.mode);

    ESP_RETURN_ON_ERROR(writeReg(hmc5883l_regs::kRegCra, cra), TAG, "write CRA");
    ESP_RETURN_ON_ERROR(writeReg(hmc5883l_regs::kRegCrb, crb), TAG, "write CRB");
    ESP_RETURN_ON_ERROR(writeReg(hmc5883l_regs::kRegMode, mode), TAG, "write Mode");

    m_lsb_per_gauss = hmc5883l_regs::lsbPerGauss(m_cfg.gain);
    return ESP_OK;
}

esp_err_t Hmc5883l::init()
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

    if (m_cfg.mode == hmc5883l_regs::eHmcModeContinuous) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    m_inited = true;
    return ESP_OK;
}

esp_err_t Hmc5883l::deinit()
{
    if (!m_inited) {
        return ESP_OK;
    }

    const esp_err_t err = writeReg(hmc5883l_regs::kRegMode,
                                   hmc5883l_regs::buildMode(hmc5883l_regs::eHmcModeIdle));
    m_inited = false;
    return err;
}

esp_err_t Hmc5883l::waitReady()
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t limit = pdMS_TO_TICKS(m_cfg.poll_timeout_ms);

    while ((xTaskGetTickCount() - start) < limit) {
        uint8_t status = 0U;
        ESP_RETURN_ON_ERROR(readReg(hmc5883l_regs::kRegStatus, status), TAG, "read Status");
        if ((status & hmc5883l_regs::kStatusRdy) != 0U) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    ESP_LOGE(TAG, "Status RDY timeout (%lu ms)",
             (unsigned long)m_cfg.poll_timeout_ms);
    return ESP_ERR_TIMEOUT;
}

esp_err_t Hmc5883l::readRaw(hmc5883l_raw_t& out)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    if (m_cfg.mode == hmc5883l_regs::eHmcModeSingle) {
        ESP_RETURN_ON_ERROR(
            writeReg(hmc5883l_regs::kRegMode,
                     hmc5883l_regs::buildMode(hmc5883l_regs::eHmcModeSingle)),
            TAG, "trigger single measure");
    }

    ESP_RETURN_ON_ERROR(waitReady(), TAG, "waitReady");

    uint8_t data[6] = {0U};
    ESP_RETURN_ON_ERROR(readRegs(hmc5883l_regs::kRegDataX, data, sizeof(data)),
                       TAG, "read data");

    out.x = hmc5883l_regs::assembleRaw(data[0], data[1]);
    out.z = hmc5883l_regs::assembleRaw(data[2], data[3]);
    out.y = hmc5883l_regs::assembleRaw(data[4], data[5]);

    if ((out.x == hmc5883l_regs::kOverflowRaw) ||
        (out.y == hmc5883l_regs::kOverflowRaw) ||
        (out.z == hmc5883l_regs::kOverflowRaw)) {
        ESP_LOGW(TAG, "ADC overflow");
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t Hmc5883l::readGauss(hmc5883l_gauss_t& out)
{
    hmc5883l_raw_t raw = {};
    ESP_RETURN_ON_ERROR(readRaw(raw), TAG, "readRaw");

    if (m_lsb_per_gauss == 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    const float scale = 1.0f / static_cast<float>(m_lsb_per_gauss);
    out.x = static_cast<float>(raw.x) * scale;
    out.y = static_cast<float>(raw.y) * scale;
    out.z = static_cast<float>(raw.z) * scale;
    return ESP_OK;
}

bool Hmc5883l::isInited() const
{
    return m_inited;
}
