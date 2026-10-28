/**
 * @file        hmc5983_api.hpp
 * @brief       Honeywell HMC5983 3-axis magnetometer driver
 * @author      Mat RB
 * @date        2026-08-31
 * @version     1.0.0
 *
 * @details
 *   I2C via i2c_api. Temp sensor (CRA7) enabled for factory compensation.
 *   Supports ODR up to 220 Hz. ID check expects "H43".
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#ifndef HMC5983_API_HPP
#define HMC5983_API_HPP

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "i2c_api.hpp"
#include "sensors/hmc5983_regs.hpp"

typedef struct {
    uint8_t addr7 = hmc5983_regs::kAddr7;
    bool temp_sensor = true;
    hmc5983_regs::SamplesAvg samples = hmc5983_regs::eHmc5983Samples1;
    hmc5983_regs::OutputRate odr = hmc5983_regs::eHmc5983Odr220Hz;
    hmc5983_regs::Gain gain = hmc5983_regs::eHmc5983Gain13Ga;
    hmc5983_regs::MeasMode meas = hmc5983_regs::eHmc5983MeasNormal;
    hmc5983_regs::OpMode mode = hmc5983_regs::eHmc5983ModeContinuous;
    uint32_t poll_timeout_ms = hmc5983_regs::kDefaultPollTimeoutMs;
} hmc5983_cfg_t;

typedef struct {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
} hmc5983_raw_t;

typedef struct {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
} hmc5983_gauss_t;

class Hmc5983
{
public:
    explicit Hmc5983(I2C& bus, const hmc5983_cfg_t& cfg = {});
    Hmc5983(I2C& bus, uint8_t addr7);

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t readRaw(hmc5983_raw_t& out);
    esp_err_t readGauss(hmc5983_gauss_t& out);
    esp_err_t readTemperature(float& celsius);

    bool isInited() const;

private:
    I2C&           m_bus;
    hmc5983_cfg_t  m_cfg;
    bool           m_inited = false;
    uint16_t       m_lsb_per_gauss = 1090U;

    esp_err_t writeReg(uint8_t reg, uint8_t val);
    esp_err_t readReg(uint8_t reg, uint8_t& val);
    esp_err_t readRegs(uint8_t reg, uint8_t* buf, size_t len);
    esp_err_t waitReady();
    esp_err_t verifyId();
    esp_err_t applyConfig();
};

#endif /* HMC5983_API_HPP */
