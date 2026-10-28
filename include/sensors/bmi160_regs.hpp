/**
 * @file        bmi160_regs.hpp
 * @brief       Bosch BMI160 register map and scale helpers
 * @author      Mat RB
 * @date        2026-10-28
 *
 * @details
 *   Datasheet: BMI160. CHIP_ID (reg 0x00) is 0xD1.
 *   7-bit I2C address is 0x68 (SDO low) or 0x69 (SDO high) — not 0xD1.
 */

#ifndef BMI160_REGS_HPP
#define BMI160_REGS_HPP

#include <stdint.h>

namespace bmi160_regs {

static constexpr uint8_t kAddr7SdoLow  = 0x68U;
static constexpr uint8_t kAddr7SdoHigh = 0x69U;
static constexpr uint8_t kAddr7        = kAddr7SdoLow;

static constexpr uint8_t kChipIdExpected = 0xD1U;

static constexpr uint8_t kRegChipId    = 0x00U;
static constexpr uint8_t kRegErr       = 0x02U;
static constexpr uint8_t kRegPmuStatus = 0x03U;
static constexpr uint8_t kRegDataGyro  = 0x0CU; /* GYR_X_L … ACC_Z_H (12 bytes) */
static constexpr uint8_t kRegDataAcc   = 0x12U; /* ACC_X_L … ACC_Z_H (6 bytes) */
static constexpr uint8_t kRegAccConf   = 0x40U;
static constexpr uint8_t kRegAccRange  = 0x41U;
static constexpr uint8_t kRegGyrConf   = 0x42U;
static constexpr uint8_t kRegGyrRange  = 0x43U;
static constexpr uint8_t kRegCmd       = 0x7EU;

static constexpr uint8_t kCmdSoftReset   = 0xB6U;
static constexpr uint8_t kCmdAccNormal   = 0x11U;
static constexpr uint8_t kCmdGyrNormal   = 0x15U;
static constexpr uint8_t kCmdAccSuspend  = 0x10U;
static constexpr uint8_t kCmdGyrSuspend  = 0x14U;

/** ACC_CONF / GYR_CONF lower nibble ODR codes (datasheet). */
static constexpr uint8_t kOdr100Hz = 0x08U;
/** BWP normal + ODR 100 Hz (common bring-up value 0x28). */
static constexpr uint8_t kConfOdr100Hz = 0x28U;

static constexpr uint32_t kDefaultTimeoutMs     = 50U;
static constexpr uint32_t kSoftResetDelayMs     = 1U;
static constexpr uint32_t kPorSettleMs          = 50U;
static constexpr uint32_t kAccNormalDelayMs     = 5U;
static constexpr uint32_t kGyrNormalDelayMs     = 80U;

enum AccelRange : uint8_t {
    eBmiAccRange2g  = 0x03U,
    eBmiAccRange4g  = 0x05U,
    eBmiAccRange8g  = 0x08U,
    eBmiAccRange16g = 0x0CU,
};

enum GyroRange : uint8_t {
    eBmiGyrRange2000dps = 0x00U,
    eBmiGyrRange1000dps = 0x01U,
    eBmiGyrRange500dps  = 0x02U,
    eBmiGyrRange250dps  = 0x03U,
    eBmiGyrRange125dps  = 0x04U,
};

/**
 * @brief LSB per g for the selected accel range.
 */
static inline float accelLsbPerG(AccelRange range)
{
    switch (range) {
    case eBmiAccRange2g:
        return 16384.0f;
    case eBmiAccRange4g:
        return 8192.0f;
    case eBmiAccRange8g:
        return 4096.0f;
    case eBmiAccRange16g:
        return 2048.0f;
    default:
        return 8192.0f;
    }
}

/**
 * @brief LSB per deg/s for the selected gyro range.
 */
static inline float gyroLsbPerDps(GyroRange range)
{
    switch (range) {
    case eBmiGyrRange2000dps:
        return 16.4f;
    case eBmiGyrRange1000dps:
        return 32.8f;
    case eBmiGyrRange500dps:
        return 65.6f;
    case eBmiGyrRange250dps:
        return 131.2f;
    case eBmiGyrRange125dps:
        return 262.4f;
    default:
        return 65.6f;
    }
}

} /* namespace bmi160_regs */

#endif /* BMI160_REGS_HPP */
