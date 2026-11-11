/**
 * @file        qmc5883p_api.cpp
 * @brief       QMC5883P magnetometer implementation
 * @author      Mat RB
 * @date        2026-11-10
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#include "sensors/qmc5883p_api.hpp"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "QMC5883P";
static constexpr int64_t kUsPerMs = 1000LL;
/** I2C transaction budget. This is not the DRDY wait. */
static constexpr uint32_t kRegIoTimeoutMs = 50U;


Qmc5883p::Qmc5883p(I2C& bus, const qmc5883p_cfg_t& cfg)
    : m_bus(bus)
    , m_cfg(cfg)
{
}

Qmc5883p::Qmc5883p(I2C& bus, uint8_t addr7)
    : m_bus(bus)
{
    m_cfg.addr7 = addr7;
}

Qmc5883p::~Qmc5883p()
{
    (void)deinit();
}

esp_err_t Qmc5883p::writeReg(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = {reg, val};
    return m_bus.write(m_cfg.addr7, buf, sizeof(buf), kRegIoTimeoutMs);
}

esp_err_t Qmc5883p::readReg(uint8_t reg, uint8_t& val)
{
    return m_bus.writeRead(m_cfg.addr7, &reg, 1U, &val, 1U, kRegIoTimeoutMs);
}

esp_err_t Qmc5883p::readRegs(uint8_t reg, uint8_t* buf, size_t len)
{
    return m_bus.writeRead(m_cfg.addr7, &reg, 1U, buf, len, kRegIoTimeoutMs);
}

esp_err_t Qmc5883p::verifyId()
{
    uint8_t id = 0U;
    ESP_RETURN_ON_ERROR(readReg(qmc5883p_regs::kRegChipId, id), TAG, "read chip ID");
    if (id != qmc5883p_regs::kChipId) {
        ESP_LOGE(TAG, "ID mismatch: got 0x%02X expected 0x80", static_cast<unsigned>(id));
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "ID OK (0x80) addr 0x%02X", static_cast<unsigned>(m_cfg.addr7));
    return ESP_OK;
}

esp_err_t Qmc5883p::enterSuspend()
{
    const uint8_t ctrl1 = qmc5883p_regs::buildCtrl1(
        qmc5883p_regs::eQmcModeSuspend, m_cfg.odr, m_cfg.osr1, m_cfg.osr2);
    ESP_RETURN_ON_ERROR(writeReg(qmc5883p_regs::kRegCtrl1, ctrl1), TAG, "write suspend");
    m_hw_mode = qmc5883p_regs::eQmcModeSuspend;
    return ESP_OK;
}

esp_err_t Qmc5883p::applyConfig()
{
    const uint8_t ctrl2 = qmc5883p_regs::buildCtrl2(m_cfg.range, m_cfg.set_reset);
    const uint8_t ctrl1 = qmc5883p_regs::buildCtrl1(
        m_cfg.mode, m_cfg.odr, m_cfg.osr1, m_cfg.osr2);
    const uint16_t lsb = qmc5883p_regs::lsbPerGauss(m_cfg.range);
    if (lsb == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    // Suspend first. A measure mode blocks the setup register writes.
    if (qmc5883p_regs::isMeasureMode(m_hw_mode)) {
        ESP_RETURN_ON_ERROR(enterSuspend(), TAG, "mode bridge");
    }

    ESP_RETURN_ON_ERROR(
        writeReg(qmc5883p_regs::kRegAxisSign, qmc5883p_regs::kAxisSignXyz),
        TAG, "write axis sign");
    ESP_RETURN_ON_ERROR(writeReg(qmc5883p_regs::kRegCtrl2, ctrl2), TAG, "write CTRL2");
    ESP_RETURN_ON_ERROR(writeReg(qmc5883p_regs::kRegCtrl1, ctrl1), TAG, "write CTRL1");
    m_hw_mode = m_cfg.mode;
    m_lsb_per_gauss = lsb;
    return ESP_OK;
}

esp_err_t Qmc5883p::init()
{
    if (m_inited) {
        ESP_LOGW(TAG, "Already inited");
        return ESP_OK;
    }
    if (!m_bus.isInited()) {
        ESP_LOGE(TAG, "I2C bus not inited");
        return ESP_ERR_INVALID_STATE;
    }
    if (m_cfg.addr7 > 0x7FU) {
        return ESP_ERR_INVALID_ARG;
    }

    m_hw_mode = qmc5883p_regs::eQmcModeSuspend;
    ESP_RETURN_ON_ERROR(verifyId(), TAG, "verifyId");
    ESP_RETURN_ON_ERROR(applyConfig(), TAG, "applyConfig");
    m_inited = true;
    return ESP_OK;
}

esp_err_t Qmc5883p::reconfigure(const qmc5883p_cfg_t& cfg)
{
    if (!m_bus.isInited()) {
        ESP_LOGE(TAG, "I2C bus not inited");
        return ESP_ERR_INVALID_STATE;
    }

    if (!m_inited) {
        m_cfg = cfg;
        return ESP_OK;
    }
    const qmc5883p_cfg_t saved = m_cfg;
    m_cfg = cfg;
    const esp_err_t err = applyConfig();
    if (err != ESP_OK) {
        m_cfg = saved;
        ESP_LOGE(TAG, "reconfigure failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t Qmc5883p::deinit()
{
    if (!m_inited) {
        return ESP_OK;
    }

    esp_err_t err = ESP_OK;
    if (m_bus.isInited()) {
        err = enterSuspend();
    } else {
        err = ESP_ERR_INVALID_STATE;
    }
    m_inited = false;
    m_hw_mode = qmc5883p_regs::eQmcModeSuspend;
    return err;
}

static void feedTaskWdt(void)
{
    const esp_err_t state = esp_task_wdt_status(nullptr);
    if (state != ESP_OK) {
        return;
    }
    const esp_err_t err = esp_task_wdt_reset();
    if (err == ESP_OK || err == ESP_ERR_NOT_FOUND || err == ESP_ERR_INVALID_STATE) {
        return;
    }
}

esp_err_t Qmc5883p::waitReady(uint8_t& status)
{
    // One tick is 10 ms. That delay drops samples at 200 Hz and at 1500 Hz.
    const int64_t start_us = esp_timer_get_time();
    const int64_t budget_us = static_cast<int64_t>(m_cfg.poll_timeout_ms) * kUsPerMs;

    while ((esp_timer_get_time() - start_us) < budget_us) {
        feedTaskWdt();
        ESP_RETURN_ON_ERROR(readReg(qmc5883p_regs::kRegStatus, status), TAG, "read status");
        if ((status & qmc5883p_regs::kStatusDrdy) != 0U) {
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "DRDY timeout (%lu ms)", static_cast<unsigned long>(m_cfg.poll_timeout_ms));
    return ESP_ERR_TIMEOUT;
}

esp_err_t Qmc5883p::readSample(qmc5883p_raw_t& out, bool& overflow)
{
    uint8_t status = 0U;
    ESP_RETURN_ON_ERROR(waitReady(status), TAG, "waitReady");

    uint8_t data[qmc5883p_regs::kDataLen] = {0U};
    ESP_RETURN_ON_ERROR(
        readRegs(qmc5883p_regs::kRegData, data, sizeof(data)),
        TAG, "read data");

    out.x = qmc5883p_regs::assembleLe(data[0], data[1]);
    out.y = qmc5883p_regs::assembleLe(data[2], data[3]);
    out.z = qmc5883p_regs::assembleLe(data[4], data[5]);
    overflow = (status & qmc5883p_regs::kStatusOvfl) != 0U;
    return ESP_OK;
}

esp_err_t Qmc5883p::readRaw(qmc5883p_raw_t& out)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (m_cfg.mode == qmc5883p_regs::eQmcModeSuspend) {
        return ESP_ERR_INVALID_STATE;
    }

    if (m_cfg.mode == qmc5883p_regs::eQmcModeSingle) {
        if (m_hw_mode != qmc5883p_regs::eQmcModeSuspend) {
            ESP_RETURN_ON_ERROR(enterSuspend(), TAG, "single bridge");
        }
        const uint8_t ctrl1 = qmc5883p_regs::buildCtrl1(
            qmc5883p_regs::eQmcModeSingle, m_cfg.odr, m_cfg.osr1, m_cfg.osr2);
        ESP_RETURN_ON_ERROR(writeReg(qmc5883p_regs::kRegCtrl1, ctrl1), TAG, "start single");
        m_hw_mode = qmc5883p_regs::eQmcModeSingle;
    }

    bool overflow = false;
    ESP_RETURN_ON_ERROR(readSample(out, overflow), TAG, "readSample");
    if (m_cfg.mode == qmc5883p_regs::eQmcModeSingle) {
        m_hw_mode = qmc5883p_regs::eQmcModeSuspend;
    }
    if (overflow) {
        ESP_LOGW(TAG, "data overflow");
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t Qmc5883p::readGauss(qmc5883p_gauss_t& out)
{
    qmc5883p_raw_t raw = {};
    const esp_err_t err = readRaw(raw);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_RESPONSE)) {
        return err;
    }
    if (m_lsb_per_gauss == 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    const float scale = 1.0f / static_cast<float>(m_lsb_per_gauss);
    out.x = static_cast<float>(raw.x) * scale;
    out.y = static_cast<float>(raw.y) * scale;
    out.z = static_cast<float>(raw.z) * scale;
    return err;
}

esp_err_t Qmc5883p::softReset()
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(
        writeReg(qmc5883p_regs::kRegCtrl2, qmc5883p_regs::kCtrl2SoftReset),
        TAG, "soft reset");
    vTaskDelay(pdMS_TO_TICKS(qmc5883p_regs::kSoftResetDelayMs));

    m_hw_mode = qmc5883p_regs::eQmcModeSuspend;
    ESP_RETURN_ON_ERROR(verifyId(), TAG, "ID after reset");
    return applyConfig();
}

esp_err_t Qmc5883p::selfTest(qmc5883p_self_test_t& delta)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    const qmc5883p_cfg_t saved = m_cfg;
    m_cfg.mode = qmc5883p_regs::eQmcModeContinuous;
    m_cfg.odr = qmc5883p_regs::eQmcOdr10Hz;
    m_cfg.osr1 = qmc5883p_regs::eQmcOsr1_8;
    m_cfg.osr2 = qmc5883p_regs::eQmcOsr2_1;

    esp_err_t err = applyConfig();
    qmc5883p_raw_t before = {};
    qmc5883p_raw_t after = {};
    bool overflow = false;
    if (err == ESP_OK) {
        err = readSample(before, overflow);
    }
    if (err == ESP_OK) {
        err = writeReg(qmc5883p_regs::kRegCtrl2, qmc5883p_regs::kCtrl2SelfTest);
    }
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(qmc5883p_regs::kSelfTestDelayMs));
        err = readSample(after, overflow);
    }

    uint8_t ctrl2 = 0xFFU;
    if (err == ESP_OK) {
        err = readReg(qmc5883p_regs::kRegCtrl2, ctrl2);
    }

    delta.x = static_cast<int32_t>(before.x) - static_cast<int32_t>(after.x);
    delta.y = static_cast<int32_t>(after.y) - static_cast<int32_t>(before.y);
    delta.z = static_cast<int32_t>(after.z) - static_cast<int32_t>(before.z);

    m_cfg = saved;
    m_hw_mode = qmc5883p_regs::eQmcModeSuspend;
    const esp_err_t restore = softReset();
    if (err != ESP_OK) {
        return err;
    }
    if (restore != ESP_OK) {
        return restore;
    }
    if ((ctrl2 & qmc5883p_regs::kCtrl2SelfTest) != 0U) {
        ESP_LOGW(TAG, "self-test bit stayed set");
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "self-test delta X=%ld Y=%ld Z=%ld",
             static_cast<long>(delta.x),
             static_cast<long>(delta.y),
             static_cast<long>(delta.z));
    return ESP_OK;
}

bool Qmc5883p::isInited() const
{
    return m_inited;
}
