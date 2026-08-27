/**
 * @file        amg8833_regs.hpp
 * @brief       Panasonic AMG8833 (Grid-EYE) register map and converters
 * @author      Mat RB
 * @date        2026-08-26
 *
 * @details     Datasheet: Panasonic Grid-EYE AMG88xx. 7-bit I2C 0x69 (AD0 high)
 *              or 0x68 (AD0 low). 8x8 IR pixels @ 0.25 C/LSB; thermistor 0.0625 C/LSB.
 */

#ifndef AMG8833_REGS_HPP
#define AMG8833_REGS_HPP

#include <stdint.h>

namespace amg8833_regs {

static constexpr uint8_t kAddr7    = 0x69U;
static constexpr uint8_t kAddr7Alt = 0x68U;

static constexpr uint8_t kRegPctl  = 0x00U;
static constexpr uint8_t kRegRst   = 0x01U;
static constexpr uint8_t kRegFpsc  = 0x02U;
static constexpr uint8_t kRegIntc  = 0x03U;
static constexpr uint8_t kRegStat  = 0x04U;
static constexpr uint8_t kRegSclr  = 0x05U;
static constexpr uint8_t kRegAve   = 0x07U;
static constexpr uint8_t kRegTthL  = 0x0EU;
static constexpr uint8_t kRegTthH  = 0x0FU;
static constexpr uint8_t kRegPixelBase = 0x80U;

static constexpr uint8_t kPctlNormal = 0x00U;
static constexpr uint8_t kPctlSleep  = 0x10U;
static constexpr uint8_t kPctlStandby60 = 0x20U;
static constexpr uint8_t kPctlStandby10 = 0x21U;

static constexpr uint8_t kRstFlag   = 0x30U;
static constexpr uint8_t kRstInitial = 0x3FU;

static constexpr uint8_t kFpsc10Hz = 0x00U;
static constexpr uint8_t kFpsc1Hz  = 0x01U;

static constexpr uint8_t kIntcOff = 0x00U;

/** Twice moving average enable (bit 5). */
static constexpr uint8_t kAveMovingOn = 0x20U;

static constexpr uint8_t kPixelCount = 64U;
static constexpr uint8_t kPixelBytes = 128U; /* 64 * 2 */

static constexpr float kPixelLsbC      = 0.25f;
static constexpr float kThermistorLsbC = 0.0625f;

static constexpr uint32_t kDefaultTimeoutMs = 100U;

enum FrameRate : uint8_t {
    eAmgFps10Hz = 0U,
    eAmgFps1Hz  = 1U,
};

static inline uint8_t fpsReg(FrameRate fps)
{
    return (fps == eAmgFps1Hz) ? kFpsc1Hz : kFpsc10Hz;
}

/** 12-bit two's complement pixel word (low byte first). */
static inline int16_t pixelRawFromBytes(uint8_t lo, uint8_t hi)
{
    const uint16_t raw12 = static_cast<uint16_t>(
        (static_cast<uint16_t>(hi & 0x0FU) << 8U) | lo);
    if ((raw12 & 0x0800U) != 0U) {
        return static_cast<int16_t>(static_cast<int16_t>(raw12) - 4096);
    }
    return static_cast<int16_t>(raw12);
}

static inline float pixelToCelsius(uint8_t lo, uint8_t hi)
{
    return static_cast<float>(pixelRawFromBytes(lo, hi)) * kPixelLsbC;
}

/** Thermistor: 12-bit signed, low then high. */
static inline float thermistorToCelsius(uint8_t lo, uint8_t hi)
{
    const uint16_t raw12 = static_cast<uint16_t>(
        (static_cast<uint16_t>(hi & 0x0FU) << 8U) | lo);
    int16_t signed12 = 0;
    if ((raw12 & 0x0800U) != 0U) {
        signed12 = static_cast<int16_t>(-(static_cast<int16_t>(raw12 & 0x07FFU)));
    } else {
        signed12 = static_cast<int16_t>(raw12);
    }
    return static_cast<float>(signed12) * kThermistorLsbC;
}

} /* namespace amg8833_regs */

#endif /* AMG8833_REGS_HPP */
