/**
 * @file        hmc5883l_api.hpp
 * @brief       Honeywell HMC5883L 3-axis magnetometer driver
 * @author      Mat RB
 * @date        2026-08-19
 * @version     1.0.0
 *
 * @details
 *   Uses i2c_api for register access. No DRDY GPIO — polls Status.RDY.
 *   Fails init if identification registers are not "H43".
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#ifndef HMC5883L_API_HPP
#define HMC5883L_API_HPP

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "i2c_api.hpp"
#include "sensors/hmc5883l_regs.hpp"

typedef struct {
    uint8_t addr7 = hmc5883l_regs::kAddr7;
    hmc5883l_regs::SamplesAvg samples = hmc5883l_regs::eHmcSamples8;
    hmc5883l_regs::OutputRate odr       = hmc5883l_regs::eHmcOdr15Hz;
    hmc5883l_regs::Gain gain            = hmc5883l_regs::eHmcGain13Ga;
    hmc5883l_regs::OpMode mode          = hmc5883l_regs::eHmcModeContinuous;
    uint32_t poll_timeout_ms            = hmc5883l_regs::kDefaultPollTimeoutMs;
} hmc5883l_cfg_t;

typedef struct {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
} hmc5883l_raw_t;

typedef struct {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
} hmc5883l_gauss_t;

class Hmc5883l
{
public:
    explicit Hmc5883l(I2C& bus, const hmc5883l_cfg_t& cfg = {});
    Hmc5883l(I2C& bus, uint8_t addr7);

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t readRaw(hmc5883l_raw_t& out);
    esp_err_t readGauss(hmc5883l_gauss_t& out);

    bool isInited() const;

private:
    I2C&            m_bus;
    hmc5883l_cfg_t  m_cfg;
    bool            m_inited = false;
    uint16_t        m_lsb_per_gauss = 1090U;

    esp_err_t writeReg(uint8_t reg, uint8_t val);
    esp_err_t readReg(uint8_t reg, uint8_t& val);
    esp_err_t readRegs(uint8_t reg, uint8_t* buf, size_t len);
    esp_err_t waitReady();
    esp_err_t verifyId();
    esp_err_t applyConfig();
};

#endif /* HMC5883L_API_HPP */
