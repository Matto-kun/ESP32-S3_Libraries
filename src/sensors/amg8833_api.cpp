/**
 * @file        amg8833_api.cpp
 * @brief       AMG8833 Grid-EYE implementation
 * @author      Mat RB
 * @date        2026-08-26
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#include "sensors/amg8833_api.hpp"

#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "AMG8833";

Amg8833::Amg8833(I2C& bus, const amg8833_cfg_t& cfg)
    : m_bus(bus)
    , m_cfg(cfg)
{
}

Amg8833::Amg8833(I2C& bus, uint8_t addr7)
    : m_bus(bus)
{
    m_cfg.addr7 = addr7;
}

esp_err_t Amg8833::writeReg(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = {reg, val};
    return m_bus.write(m_cfg.addr7, buf, sizeof(buf), m_cfg.timeout_ms);
}

esp_err_t Amg8833::readRegs(uint8_t reg, uint8_t* buf, size_t len)
{
    return m_bus.writeRead(m_cfg.addr7, &reg, 1U, buf, len, m_cfg.timeout_ms);
}

esp_err_t Amg8833::applyConfig()
{
    ESP_RETURN_ON_ERROR(
        writeReg(amg8833_regs::kRegPctl, amg8833_regs::kPctlNormal),
        TAG, "PCTL normal");

    ESP_RETURN_ON_ERROR(
        writeReg(amg8833_regs::kRegRst, amg8833_regs::kRstInitial),
        TAG, "RST initial");
    vTaskDelay(pdMS_TO_TICKS(2));

    ESP_RETURN_ON_ERROR(
        writeReg(amg8833_regs::kRegRst, amg8833_regs::kRstFlag),
        TAG, "RST flag");
    vTaskDelay(pdMS_TO_TICKS(2));

    ESP_RETURN_ON_ERROR(
        writeReg(amg8833_regs::kRegFpsc, amg8833_regs::fpsReg(m_cfg.fps)),
        TAG, "FPSC");

    ESP_RETURN_ON_ERROR(
        writeReg(amg8833_regs::kRegIntc, amg8833_regs::kIntcOff),
        TAG, "INTC off");

    /* AVE is reserved-access: unlock via 0x1F, write 0x07, then lock. */
    ESP_RETURN_ON_ERROR(writeReg(0x1FU, 0x50U), TAG, "AVE unlock 1");
    ESP_RETURN_ON_ERROR(writeReg(0x1FU, 0x45U), TAG, "AVE unlock 2");
    ESP_RETURN_ON_ERROR(writeReg(0x1FU, 0x57U), TAG, "AVE unlock 3");
    const uint8_t ave = m_cfg.moving_average ? amg8833_regs::kAveMovingOn : 0x00U;
    ESP_RETURN_ON_ERROR(writeReg(amg8833_regs::kRegAve, ave), TAG, "AVE");
    ESP_RETURN_ON_ERROR(writeReg(0x1FU, 0x00U), TAG, "AVE lock");

    return ESP_OK;
}

esp_err_t Amg8833::init()
{
    if (m_inited) {
        ESP_LOGW(TAG, "Already inited");
        return ESP_OK;
    }

    if (!m_bus.isInited()) {
        ESP_LOGE(TAG, "I2C bus not inited");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(applyConfig(), TAG, "applyConfig");

    float t_c = 0.0f;
    ESP_RETURN_ON_ERROR(readThermistor(t_c), TAG, "probe thermistor");

    m_inited = true;
    ESP_LOGI(TAG, "Inited addr=0x%02X thermistor=%.2f C",
             (unsigned)m_cfg.addr7, (double)t_c);
    return ESP_OK;
}

esp_err_t Amg8833::deinit()
{
    if (!m_inited) {
        return ESP_OK;
    }

    const esp_err_t err = writeReg(amg8833_regs::kRegPctl, amg8833_regs::kPctlSleep);
    m_inited = false;
    return err;
}

esp_err_t Amg8833::readThermistor(float& celsius)
{
    if (!m_bus.isInited()) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[2] = {0U, 0U};
    ESP_RETURN_ON_ERROR(readRegs(amg8833_regs::kRegTthL, raw, sizeof(raw)),
                       TAG, "read thermistor");
    celsius = amg8833_regs::thermistorToCelsius(raw[0], raw[1]);
    return ESP_OK;
}

esp_err_t Amg8833::readFrame(amg8833_frame_t& out)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[amg8833_regs::kPixelBytes] = {0U};
    ESP_RETURN_ON_ERROR(
        readRegs(amg8833_regs::kRegPixelBase, raw, sizeof(raw)),
        TAG, "read pixels");

    for (uint8_t i = 0U; i < amg8833_regs::kPixelCount; i++) {
        const uint8_t lo = raw[static_cast<size_t>(i) * 2U];
        const uint8_t hi = raw[static_cast<size_t>(i) * 2U + 1U];
        out.pixels[i] = amg8833_regs::pixelToCelsius(lo, hi);
    }
    return ESP_OK;
}

bool Amg8833::isInited() const
{
    return m_inited;
}

uint8_t Amg8833::addr7() const
{
    return m_cfg.addr7;
}
