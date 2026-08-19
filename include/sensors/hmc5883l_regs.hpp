/**
 * @file        hmc5883l_regs.hpp
 * @brief       HMC5883L register map and bit-field helpers (Honeywell)
 * @author      Mat RB
 * @date        2026-08-19
 *
 * @details     Datasheet: Honeywell HMC5883L (7-bit I2C 0x1E).
 */

#ifndef HMC5883L_REGS_HPP
#define HMC5883L_REGS_HPP

#include <stdint.h>

namespace hmc5883l_regs {

static constexpr uint8_t kAddr7 = 0x1EU;

static constexpr uint8_t kRegCra   = 0x00U;
static constexpr uint8_t kRegCrb   = 0x01U;
static constexpr uint8_t kRegMode  = 0x02U;
static constexpr uint8_t kRegDataX = 0x03U;
static constexpr uint8_t kRegStatus = 0x09U;
static constexpr uint8_t kRegIdA   = 0x0AU;
static constexpr uint8_t kRegIdB   = 0x0BU;
static constexpr uint8_t kRegIdC   = 0x0CU;

static constexpr uint8_t kIdAExpected = static_cast<uint8_t>('H');
static constexpr uint8_t kIdBExpected = static_cast<uint8_t>('4');
static constexpr uint8_t kIdCExpected = static_cast<uint8_t>('3');

static constexpr uint8_t kStatusRdy  = 0x01U;
static constexpr uint8_t kStatusLock = 0x02U;

static constexpr int16_t kOverflowRaw = static_cast<int16_t>(-4096);

static constexpr uint32_t kDefaultPollTimeoutMs = 50U;

enum SamplesAvg : uint8_t {
    eHmcSamples1 = 0U,
    eHmcSamples2 = 1U,
    eHmcSamples4 = 2U,
    eHmcSamples8 = 3U,
};

enum OutputRate : uint8_t {
    eHmcOdr075Hz = 0U,
    eHmcOdr1p5Hz = 1U,
    eHmcOdr3Hz   = 2U,
    eHmcOdr7p5Hz = 3U,
    eHmcOdr15Hz  = 4U,
    eHmcOdr30Hz  = 5U,
    eHmcOdr75Hz  = 6U,
};

enum Gain : uint8_t {
    eHmcGain088Ga = 0U,
    eHmcGain13Ga  = 1U,
    eHmcGain19Ga  = 2U,
    eHmcGain25Ga  = 3U,
    eHmcGain40Ga  = 4U,
    eHmcGain47Ga  = 5U,
    eHmcGain56Ga  = 6U,
    eHmcGain81Ga  = 7U,
};

enum OpMode : uint8_t {
    eHmcModeContinuous = 0U,
    eHmcModeSingle     = 1U,
    eHmcModeIdle       = 2U,
};

static inline uint8_t buildCra(SamplesAvg avg, OutputRate odr)
{
    return static_cast<uint8_t>(
        (static_cast<uint8_t>(avg) << 5U) |
        (static_cast<uint8_t>(odr) << 2U));
}

static inline uint8_t buildCrb(Gain gain)
{
    return static_cast<uint8_t>(static_cast<uint8_t>(gain) << 5U);
}

static inline uint8_t buildMode(OpMode mode)
{
    return static_cast<uint8_t>(mode & 0x03U);
}

static inline uint16_t lsbPerGauss(Gain gain)
{
    static constexpr uint16_t kTable[] = {
        1370U, 1090U, 820U, 660U, 440U, 390U, 330U, 230U
    };
    const uint8_t idx = static_cast<uint8_t>(gain);
    return (idx < 8U) ? kTable[idx] : kTable[1];
}

static inline int16_t assembleRaw(uint8_t msb, uint8_t lsb)
{
    return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8U) | lsb);
}

} /* namespace hmc5883l_regs */

#endif /* HMC5883L_REGS_HPP */
