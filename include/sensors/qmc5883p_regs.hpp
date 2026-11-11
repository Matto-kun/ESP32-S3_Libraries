/**
 * @file        qmc5883p_regs.hpp
 * @brief       QMC5883P register map (QST QMC5883P, Rev E)
 * @author      Mat RB
 * @date        2026-11-10
 *
 * @details     7-bit I2C address is 0x2C. Chip ID at 0x00 is 0x80.
 *              Field positions follow datasheet section 9.
 */

#ifndef QMC5883P_REGS_HPP
#define QMC5883P_REGS_HPP

#include <stdint.h>

namespace qmc5883p_regs {

static constexpr uint8_t kAddr7 = 0x2CU;
static constexpr uint8_t kChipId = 0x80U;

static constexpr uint8_t kRegChipId = 0x00U;
static constexpr uint8_t kRegData = 0x01U;
static constexpr uint8_t kRegStatus = 0x09U;
static constexpr uint8_t kRegCtrl1 = 0x0AU;
static constexpr uint8_t kRegCtrl2 = 0x0BU;

/** Datasheet section 7 writes 0x06 here before the control registers. */
static constexpr uint8_t kRegAxisSign = 0x29U;
static constexpr uint8_t kAxisSignXyz = 0x06U;

static constexpr uint8_t kStatusDrdy = 0x01U;
static constexpr uint8_t kStatusOvfl = 0x02U;

static constexpr uint8_t kCtrl2SelfTest = 0x40U;
static constexpr uint8_t kCtrl2SoftReset = 0x80U;

static constexpr uint8_t kDataLen = 6U;
static constexpr uint32_t kDefaultPollTimeoutMs = 200U;
static constexpr uint32_t kSoftResetDelayMs = 50U;
static constexpr uint32_t kSelfTestDelayMs = 20U;

/** Control register 1, MODE bits 1:0. */
enum Mode : uint8_t {
    eQmcModeSuspend = 0U,
    eQmcModeNormal = 1U,
    eQmcModeSingle = 2U,
    eQmcModeContinuous = 3U,
};

/** Control register 1, ODR bits 3:2. */
enum Odr : uint8_t {
    eQmcOdr10Hz = 0U,
    eQmcOdr50Hz = 1U,
    eQmcOdr100Hz = 2U,
    eQmcOdr200Hz = 3U,
};

/** Control register 1, OSR1 bits 5:4. Larger ratio means lower noise. */
enum Osr1 : uint8_t {
    eQmcOsr1_8 = 0U,
    eQmcOsr1_4 = 1U,
    eQmcOsr1_2 = 2U,
    eQmcOsr1_1 = 3U,
};

/** Control register 1, OSR2 bits 7:6. Datasheet name for the downsample depth. */
enum Osr2 : uint8_t {
    eQmcOsr2_1 = 0U,
    eQmcOsr2_2 = 1U,
    eQmcOsr2_4 = 2U,
    eQmcOsr2_8 = 3U,
};

/** Control register 2, RNG bits 3:2. */
enum Range : uint8_t {
    eQmcRange30Ga = 0U,
    eQmcRange12Ga = 1U,
    eQmcRange8Ga = 2U,
    eQmcRange2Ga = 3U,
};

/** Control register 2, SET/RESET bits 1:0. Value 3 matches "off". */
enum SetReset : uint8_t {
    eQmcSetResetOn = 0U,
    eQmcSetResetSetOnly = 1U,
    eQmcSetResetOff = 2U,
};

static inline uint8_t buildCtrl1(Mode mode, Odr odr, Osr1 osr1, Osr2 osr2)
{
    return static_cast<uint8_t>(
        (static_cast<uint8_t>(mode) & 0x03U) |
        (static_cast<uint8_t>(static_cast<uint8_t>(odr) & 0x03U) << 2U) |
        (static_cast<uint8_t>(static_cast<uint8_t>(osr1) & 0x03U) << 4U) |
        (static_cast<uint8_t>(static_cast<uint8_t>(osr2) & 0x03U) << 6U));
}

static inline uint8_t buildCtrl2(Range range, SetReset set_reset)
{
    return static_cast<uint8_t>(
        (static_cast<uint8_t>(set_reset) & 0x03U) |
        (static_cast<uint8_t>(static_cast<uint8_t>(range) & 0x03U) << 2U));
}

/** LSB per gauss from datasheet Table 2. 0 means the range value is not valid. */
static inline uint16_t lsbPerGauss(Range range)
{
    static constexpr uint16_t kTable[] = {1000U, 2500U, 3750U, 15000U};
    const uint8_t idx = static_cast<uint8_t>(range);
    return (idx < 4U) ? kTable[idx] : 0U;
}

static inline int16_t assembleLe(uint8_t lsb, uint8_t msb)
{
    return static_cast<int16_t>(
        (static_cast<uint16_t>(msb) << 8U) | static_cast<uint16_t>(lsb));
}

static inline bool isMeasureMode(Mode mode)
{
    return (mode == eQmcModeNormal) ||
           (mode == eQmcModeSingle) ||
           (mode == eQmcModeContinuous);
}

} /* namespace qmc5883p_regs */

#endif /* QMC5883P_REGS_HPP */
