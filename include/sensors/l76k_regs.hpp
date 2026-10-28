/**
 * @file        l76k_regs.hpp
 * @brief       L76K / L76X PMTK commands and NMEA RMC constants
 * @author      Mat RB
 * @date        2026-10-28
 *
 * @details
 *   Command set aligned with Waveshare L76X demos (MediaTek PMTK).
 *   Bodies omit leading '$' and trailing checksum; l76k_api adds both.
 */

#ifndef L76K_REGS_HPP
#define L76K_REGS_HPP

#include <stdint.h>
#include <stddef.h>

namespace l76k_regs {

static constexpr int8_t kPinUnused = -1;

static constexpr uint32_t kDefaultLineTimeoutMs = 2000U;
static constexpr uint32_t kDefaultCmdTimeoutMs  = 200U;
static constexpr size_t   kDefaultRxLineMax     = 128U;
static constexpr size_t   kMaxCmdBodyLen        = 96U;

/* Startup */
static constexpr const char* kPmtkHotStart      = "PMTK101";
static constexpr const char* kPmtkWarmStart     = "PMTK102";
static constexpr const char* kPmtkColdStart     = "PMTK103";
static constexpr const char* kPmtkFullColdStart = "PMTK104";

/* Fix interval */
static constexpr const char* kPmtkPosFix1s = "PMTK220,1000";

/* NMEA sentence rates: GLL off, RMC/VTG/GGA/GSA on */
static constexpr const char* kPmtkNmeaOutput =
    "PMTK314,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0";

static constexpr const char* kPmtkNmeaReduction = "PMTK314,-1";

static constexpr const char* kPmtkSyncPpsOn  = "PMTK255,1";
static constexpr const char* kPmtkSyncPpsOff = "PMTK255,0";

static constexpr const char* kTalkerGnRmc = "GNRMC";
static constexpr const char* kTalkerGpRmc = "GPRMC";

} /* namespace l76k_regs */

#endif /* L76K_REGS_HPP */
