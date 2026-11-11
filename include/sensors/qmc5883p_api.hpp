/**
 * @file        qmc5883p_api.hpp
 * @brief       QST QMC5883P 3-axis magnetometer driver
 * @author      Mat RB
 * @date        2026-11-10
 * @version     1.0.0
 *
 * @details
 *   Uses i2c_api for register access. The 7-bit address is 0x2C.
 *   init() checks chip ID 0x80, writes the axis-sign byte, then the control registers.
 *   A read waits for the data-ready bit. That wait blocks the caller.
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#ifndef QMC5883P_API_HPP
#define QMC5883P_API_HPP

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "i2c_api.hpp"
#include "sensors/qmc5883p_regs.hpp"

typedef struct {
    uint8_t addr7 = qmc5883p_regs::kAddr7;
    qmc5883p_regs::Mode mode = qmc5883p_regs::eQmcModeNormal;
    qmc5883p_regs::Odr odr = qmc5883p_regs::eQmcOdr50Hz;
    qmc5883p_regs::Osr1 osr1 = qmc5883p_regs::eQmcOsr1_8;
    qmc5883p_regs::Osr2 osr2 = qmc5883p_regs::eQmcOsr2_8;
    qmc5883p_regs::Range range = qmc5883p_regs::eQmcRange8Ga;
    qmc5883p_regs::SetReset set_reset = qmc5883p_regs::eQmcSetResetOn;
    uint32_t poll_timeout_ms = qmc5883p_regs::kDefaultPollTimeoutMs;
} qmc5883p_cfg_t;

typedef struct {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
} qmc5883p_raw_t;

typedef struct {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
} qmc5883p_gauss_t;

/** Signed axis delta from the datasheet self-test sequence. */
typedef struct {
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
} qmc5883p_self_test_t;

class Qmc5883p
{
public:
    explicit Qmc5883p(I2C& bus, const qmc5883p_cfg_t& cfg = {});
    Qmc5883p(I2C& bus, uint8_t addr7);
    ~Qmc5883p();

    /**
     * @brief Check the chip ID and write the stored config.
     * @return ESP_OK, ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_ARG, ESP_ERR_NOT_FOUND, or an I2C error.
     * @note A second call returns ESP_OK and does not write the chip again.
     */
    esp_err_t init();

    /**
     * @brief Store a new config and write it when init() is done.
     * @param cfg New config. The address is used for later transfers.
     * @return ESP_OK, or the I2C error from the register writes.
     * @note Before init(), the config is stored and the chip is not written.
     */
    esp_err_t reconfigure(const qmc5883p_cfg_t& cfg);

    /**
     * @brief Put the chip in suspend mode.
     * @return ESP_OK when the chip was already stopped or the write succeeds.
     */
    esp_err_t deinit();

    /**
     * @brief Read one X/Y/Z sample.
     * @param[out] out Raw counts. The driver fills this before an overflow return.
     * @return ESP_OK, ESP_ERR_INVALID_STATE, ESP_ERR_TIMEOUT, ESP_ERR_INVALID_RESPONSE on overflow, or an I2C error.
     * @note The call blocks until the data-ready bit is set or poll_timeout_ms ends.
     *       The wait polls the status register. It does not sleep for a tick.
     *       Set poll_timeout_ms above one output period.
     */
    esp_err_t readRaw(qmc5883p_raw_t& out);

    /**
     * @brief Read one sample and scale it to gauss.
     * @param[out] out Magnetic field in gauss.
     * @return The same codes as readRaw().
     * @note The scale uses a float divide. LSB per gauss comes from the selected range.
     */
    esp_err_t readGauss(qmc5883p_gauss_t& out);

    /**
     * @brief Restore register defaults, then write the stored config again.
     * @return ESP_OK, or the chip-ID or I2C error.
     * @note The call blocks for the soft-reset delay.
     */
    esp_err_t softReset();

    /**
     * @brief Run the datasheet self-test and restore the stored config.
     * @param[out] delta Axis delta. X is before-after. Y and Z are after-before.
     * @return ESP_OK when the self-test bit clears. ESP_ERR_INVALID_RESPONSE when it stays set.
     * @note The datasheet does not publish a pass threshold for the delta.
     *       The call blocks for two samples, then soft-resets the chip.
     */
    esp_err_t selfTest(qmc5883p_self_test_t& delta);

    bool isInited() const;

private:
    I2C& m_bus;
    qmc5883p_cfg_t m_cfg;
    qmc5883p_regs::Mode m_hw_mode = qmc5883p_regs::eQmcModeSuspend;
    bool m_inited = false;
    uint16_t m_lsb_per_gauss = 3750U;

    esp_err_t writeReg(uint8_t reg, uint8_t val);
    esp_err_t readReg(uint8_t reg, uint8_t& val);
    esp_err_t readRegs(uint8_t reg, uint8_t* buf, size_t len);
    esp_err_t verifyId();
    esp_err_t applyConfig();
    esp_err_t enterSuspend();
    esp_err_t waitReady(uint8_t& status);
    esp_err_t readSample(qmc5883p_raw_t& out, bool& overflow);
};

#endif /* QMC5883P_API_HPP */
