/**
 * @file        bmi160_api.hpp
 * @brief       Bosch BMI160 6-axis IMU driver (accel + gyro) over I2C
 * @author      Mat RB
 * @date        2026-10-28
 * @version     1.0.0
 *
 * @details
 *   Uses i2c_api for register access. Verifies CHIP_ID 0xD1.
 *   Default 7-bit address 0x68 (SDO low); use 0x69 if SDO is high.
 *
 * @note readAccel() / readGyro() / readImu() block until the I2C transfer
 *       completes. Do not call from an ISR.
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#ifndef BMI160_API_HPP
#define BMI160_API_HPP

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"
#include "i2c_api.hpp"
#include "sensors/bmi160_regs.hpp"

typedef struct {
    uint8_t addr7 = bmi160_regs::kAddr7;
    bmi160_regs::AccelRange accel_range = bmi160_regs::eBmiAccRange4g;
    bmi160_regs::GyroRange  gyro_range  = bmi160_regs::eBmiGyrRange500dps;
    uint32_t timeout_ms = bmi160_regs::kDefaultTimeoutMs;
} bmi160_cfg_t;

typedef struct {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
} bmi160_vec3_t;

/**
 * @brief BMI160 IMU on an I2C bus.
 *
 * Hardware is untouched until init(). The destructor calls deinit() if active.
 */
class Bmi160
{
public:
    explicit Bmi160(I2C& bus, const bmi160_cfg_t& cfg = {});
    Bmi160(I2C& bus, uint8_t addr7);
    ~Bmi160();

    esp_err_t init();
    esp_err_t deinit();

    /**
     * @brief Read CHIP_ID without requiring full init (probe helper).
     */
    esp_err_t readChipId(uint8_t& out_id);

    esp_err_t readAccel(bmi160_vec3_t& out_g);
    esp_err_t readGyro(bmi160_vec3_t& out_dps);
    esp_err_t readImu(bmi160_vec3_t& accel_g, bmi160_vec3_t& gyro_dps);

    bool    isInited() const;
    uint8_t addr7() const;

private:
    I2C&         m_bus;
    bmi160_cfg_t m_cfg;
    bool         m_inited       = false;
    float        m_accel_scale  = 1.0f / 8192.0f;
    float        m_gyro_scale   = 1.0f / 65.6f;

    esp_err_t writeReg(uint8_t reg, uint8_t val);
    esp_err_t readReg(uint8_t reg, uint8_t& val);
    esp_err_t readRegs(uint8_t reg, uint8_t* buf, size_t len);
    esp_err_t softReset();
    esp_err_t verifyId();
    esp_err_t applyConfig();
    static int16_t le16(const uint8_t* p);
};

#endif /* BMI160_API_HPP */
