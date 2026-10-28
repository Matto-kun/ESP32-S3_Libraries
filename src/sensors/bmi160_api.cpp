/**
 * @file        bmi160_api.cpp
 * @brief       BMI160 IMU implementation
 * @author      Mat RB
 * @date        2026-10-28
 * @version     1.0.0
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#include "sensors/bmi160_api.hpp"

#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "BMI160";

Bmi160::Bmi160(I2C& bus, const bmi160_cfg_t& cfg)
    : m_bus(bus)
    , m_cfg(cfg)
{
}

Bmi160::Bmi160(I2C& bus, uint8_t addr7)
    : m_bus(bus)
{
    m_cfg.addr7 = addr7;
}

Bmi160::~Bmi160()
{
    if (m_inited) {
        (void)deinit();
    }
}

int16_t Bmi160::le16(const uint8_t* p)
{
    const uint16_t u =
        static_cast<uint16_t>(p[0]) |
        (static_cast<uint16_t>(p[1]) << 8);
    return static_cast<int16_t>(u);
}

esp_err_t Bmi160::writeReg(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = {reg, val};
    return m_bus.write(m_cfg.addr7, buf, sizeof(buf), m_cfg.timeout_ms);
}

esp_err_t Bmi160::readReg(uint8_t reg, uint8_t& val)
{
    return m_bus.writeRead(m_cfg.addr7, &reg, 1U, &val, 1U, m_cfg.timeout_ms);
}

esp_err_t Bmi160::readRegs(uint8_t reg, uint8_t* buf, size_t len)
{
    return m_bus.writeRead(m_cfg.addr7, &reg, 1U, buf, len, m_cfg.timeout_ms);
}

esp_err_t Bmi160::readChipId(uint8_t& out_id)
{
    out_id = 0U;
    if (!m_bus.isInited()) {
        return ESP_ERR_INVALID_STATE;
    }
    return readReg(bmi160_regs::kRegChipId, out_id);
}

esp_err_t Bmi160::softReset()
{
    /* Soft-reset may NACK; ignore transfer error then wait for POR. */
    (void)writeReg(bmi160_regs::kRegCmd, bmi160_regs::kCmdSoftReset);
    vTaskDelay(pdMS_TO_TICKS(bmi160_regs::kSoftResetDelayMs));
    vTaskDelay(pdMS_TO_TICKS(bmi160_regs::kPorSettleMs));
    return ESP_OK;
}

esp_err_t Bmi160::verifyId()
{
    uint8_t id = 0U;
    ESP_RETURN_ON_ERROR(readChipId(id), TAG, "read CHIP_ID");
    if (id != bmi160_regs::kChipIdExpected) {
        ESP_LOGE(TAG, "CHIP_ID 0x%02X (expected 0x%02X) addr 0x%02X",
                 (unsigned)id,
                 (unsigned)bmi160_regs::kChipIdExpected,
                 (unsigned)m_cfg.addr7);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "CHIP_ID OK (0xD1) addr 0x%02X", (unsigned)m_cfg.addr7);
    return ESP_OK;
}

esp_err_t Bmi160::applyConfig()
{
    ESP_RETURN_ON_ERROR(
        writeReg(bmi160_regs::kRegCmd, bmi160_regs::kCmdAccNormal),
        TAG, "accel normal");
    vTaskDelay(pdMS_TO_TICKS(bmi160_regs::kAccNormalDelayMs));

    ESP_RETURN_ON_ERROR(
        writeReg(bmi160_regs::kRegCmd, bmi160_regs::kCmdGyrNormal),
        TAG, "gyro normal");
    vTaskDelay(pdMS_TO_TICKS(bmi160_regs::kGyrNormalDelayMs));

    /* Retry gyro normal once if PMU still shows suspend. */
    uint8_t pmu = 0U;
    ESP_RETURN_ON_ERROR(readReg(bmi160_regs::kRegPmuStatus, pmu), TAG, "PMU_STATUS");
    if (((pmu >> 2) & 0x03U) != 1U) {
        ESP_LOGW(TAG, "Gyro PMU not normal (0x%02X) — retry CMD", (unsigned)pmu);
        ESP_RETURN_ON_ERROR(
            writeReg(bmi160_regs::kRegCmd, bmi160_regs::kCmdGyrNormal),
            TAG, "gyro normal retry");
        vTaskDelay(pdMS_TO_TICKS(bmi160_regs::kGyrNormalDelayMs));
        ESP_RETURN_ON_ERROR(readReg(bmi160_regs::kRegPmuStatus, pmu), TAG, "PMU retry");
    }
    ESP_LOGI(TAG, "PMU_STATUS=0x%02X", (unsigned)pmu);

    ESP_RETURN_ON_ERROR(
        writeReg(bmi160_regs::kRegAccConf, bmi160_regs::kConfOdr100Hz),
        TAG, "ACC_CONF");
    ESP_RETURN_ON_ERROR(
        writeReg(bmi160_regs::kRegAccRange,
                 static_cast<uint8_t>(m_cfg.accel_range)),
        TAG, "ACC_RANGE");
    ESP_RETURN_ON_ERROR(
        writeReg(bmi160_regs::kRegGyrConf, bmi160_regs::kConfOdr100Hz),
        TAG, "GYR_CONF");
    ESP_RETURN_ON_ERROR(
        writeReg(bmi160_regs::kRegGyrRange,
                 static_cast<uint8_t>(m_cfg.gyro_range)),
        TAG, "GYR_RANGE");

    m_accel_scale = 1.0f / bmi160_regs::accelLsbPerG(m_cfg.accel_range);
    m_gyro_scale  = 1.0f / bmi160_regs::gyroLsbPerDps(m_cfg.gyro_range);
    return ESP_OK;
}

esp_err_t Bmi160::init()
{
    if (m_inited) {
        ESP_LOGW(TAG, "Already inited");
        return ESP_OK;
    }

    if (!m_bus.isInited()) {
        ESP_LOGE(TAG, "I2C bus not inited");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(softReset(), TAG, "softReset");
    ESP_RETURN_ON_ERROR(verifyId(), TAG, "verifyId");
    ESP_RETURN_ON_ERROR(applyConfig(), TAG, "applyConfig");

    m_inited = true;
    ESP_LOGI(TAG, "Inited addr=0x%02X acc_range=0x%02X gyr_range=0x%02X",
             (unsigned)m_cfg.addr7,
             (unsigned)m_cfg.accel_range,
             (unsigned)m_cfg.gyro_range);
    return ESP_OK;
}

esp_err_t Bmi160::deinit()
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Not inited (deinit)");
        return ESP_OK;
    }

    (void)writeReg(bmi160_regs::kRegCmd, bmi160_regs::kCmdAccSuspend);
    (void)writeReg(bmi160_regs::kRegCmd, bmi160_regs::kCmdGyrSuspend);
    m_inited = false;
    ESP_LOGI(TAG, "Deinit OK");
    return ESP_OK;
}

esp_err_t Bmi160::readAccel(bmi160_vec3_t& out_g)
{
    out_g = {};
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[6] = {};
    ESP_RETURN_ON_ERROR(
        readRegs(bmi160_regs::kRegDataAcc, raw, sizeof(raw)),
        TAG, "read accel");

    out_g.x = static_cast<float>(le16(&raw[0])) * m_accel_scale;
    out_g.y = static_cast<float>(le16(&raw[2])) * m_accel_scale;
    out_g.z = static_cast<float>(le16(&raw[4])) * m_accel_scale;
    return ESP_OK;
}

esp_err_t Bmi160::readGyro(bmi160_vec3_t& out_dps)
{
    out_dps = {};
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[6] = {};
    ESP_RETURN_ON_ERROR(
        readRegs(bmi160_regs::kRegDataGyro, raw, sizeof(raw)),
        TAG, "read gyro");

    out_dps.x = static_cast<float>(le16(&raw[0])) * m_gyro_scale;
    out_dps.y = static_cast<float>(le16(&raw[2])) * m_gyro_scale;
    out_dps.z = static_cast<float>(le16(&raw[4])) * m_gyro_scale;
    return ESP_OK;
}

esp_err_t Bmi160::readImu(bmi160_vec3_t& accel_g, bmi160_vec3_t& gyro_dps)
{
    accel_g = {};
    gyro_dps = {};
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[12] = {};
    ESP_RETURN_ON_ERROR(
        readRegs(bmi160_regs::kRegDataGyro, raw, sizeof(raw)),
        TAG, "read imu burst");

    gyro_dps.x = static_cast<float>(le16(&raw[0])) * m_gyro_scale;
    gyro_dps.y = static_cast<float>(le16(&raw[2])) * m_gyro_scale;
    gyro_dps.z = static_cast<float>(le16(&raw[4])) * m_gyro_scale;
    accel_g.x  = static_cast<float>(le16(&raw[6])) * m_accel_scale;
    accel_g.y  = static_cast<float>(le16(&raw[8])) * m_accel_scale;
    accel_g.z  = static_cast<float>(le16(&raw[10])) * m_accel_scale;
    return ESP_OK;
}

bool Bmi160::isInited() const
{
    return m_inited;
}

uint8_t Bmi160::addr7() const
{
    return m_cfg.addr7;
}
