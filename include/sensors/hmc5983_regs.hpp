/**
 * @file        hmc5983_regs.hpp
 * @brief       HMC5983 register map and bit-field helpers (Honeywell)
 * @author      Mat RB
 * @date        2026-08-31
 *
 * @details     Datasheet: Honeywell HMC5983 (7-bit I2C 0x1E).
 *              CRA7=TS enables temperature sensor for auto compensation.
 */

#ifndef HMC5983_REGS_HPP
#define HMC5983_REGS_HPP

#include <stdint.h>

namespace hmc5983_regs {

static constexpr uint8_t kAddr7 = 0x1EU;

static constexpr uint8_t kRegCra   = 0x00U;
static constexpr uint8_t kRegCrb   = 0x01U;
static constexpr uint8_t kRegMode  = 0x02U;
static constexpr uint8_t kRegDataX = 0x03U;
static constexpr uint8_t kRegStatus = 0x09U;
static constexpr uint8_t kRegIdA   = 0x0AU;
static constexpr uint8_t kRegIdB   = 0x0BU;
static constexpr uint8_t kRegIdC   = 0x0CU;
static constexpr uint8_t kRegTempH = 0x31U;
static constexpr uint8_t kRegTempL = 0x32U;

static constexpr uint8_t kIdAExpected = static_cast<uint8_t>('H');
static constexpr uint8_t kIdBExpected = static_cast<uint8_t>('4');
static constexpr uint8_t kIdCExpected = static_cast<uint8_t>('3');

static constexpr uint8_t kCraTsEnable = 0x80U;

static constexpr uint8_t kStatusRdy  = 0x01U;
static constexpr uint8_t kStatusLock = 0x02U;

static constexpr int16_t kOverflowRaw = static_cast<int16_t>(-4096);

static constexpr uint32_t kDefaultPollTimeoutMs = 50U;

enum SamplesAvg : uint8_t {
    eHmc5983Samples1 = 0U,
    eHmc5983Samples2 = 1U,
    eHmc5983Samples4 = 2U,
    eHmc5983Samples8 = 3U,
};

enum OutputRate : uint8_t {
    eHmc5983Odr075Hz = 0U,
    eHmc5983Odr1p5Hz = 1U,
    eHmc5983Odr3Hz   = 2U,
    eHmc5983Odr7p5Hz = 3U,
    eHmc5983Odr15Hz  = 4U,
    eHmc5983Odr30Hz  = 5U,
    eHmc5983Odr75Hz  = 6U,
    eHmc5983Odr220Hz = 7U,
};

enum Gain : uint8_t {
    eHmc5983Gain088Ga = 0U,
    eHmc5983Gain13Ga  = 1U,
    eHmc5983Gain19Ga  = 2U,
    eHmc5983Gain25Ga  = 3U,
    eHmc5983Gain40Ga  = 4U,
    eHmc5983Gain47Ga  = 5U,
    eHmc5983Gain56Ga  = 6U,
    eHmc5983Gain81Ga  = 7U,
};

enum MeasMode : uint8_t {
    eHmc5983MeasNormal   = 0U,
    eHmc5983MeasPosBias  = 1U,
    eHmc5983MeasNegBias  = 2U,
    eHmc5983MeasTempOnly = 3U,
};

enum OpMode : uint8_t {
    eHmc5983ModeContinuous = 0U,
    eHmc5983ModeSingle     = 1U,
    eHmc5983ModeIdle       = 2U,
};

/** CRA7=TS | MA1:0<<5 | DO2:0<<2 | MS1:0 */
static inline uint8_t buildCra(bool temp_sensor,
                                 SamplesAvg avg,
                                 OutputRate odr,
                                 MeasMode meas = eHmc5983MeasNormal)
{
    uint8_t v = static_cast<uint8_t>(
        (static_cast<uint8_t>(avg) << 5U) |
        (static_cast<uint8_t>(odr) << 2U) |
        (static_cast<uint8_t>(meas) & 0x03U));
    if (temp_sensor) {
        v = static_cast<uint8_t>(v | kCraTsEnable);
    }
    return v;
}

static inline uint8_t buildCrb(Gain gain)
{
    return static_cast<uint8_t>(static_cast<uint8_t>(gain) << 5U);
}

/** I2C continuous mode: clear SPI/LP/HS bits; mode in MR1:0. */
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

/** Temperature (C) = (raw / 128) + 25 per datasheet. */
static inline float temperatureCelsius(int16_t raw)
{
    return (static_cast<float>(raw) / 128.0f) + 25.0f;
}

} /* namespace hmc5983_regs */

#endif /* HMC5983_REGS_HPP */
