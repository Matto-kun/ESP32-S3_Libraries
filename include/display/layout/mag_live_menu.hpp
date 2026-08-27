/**
 * @file        mag_live_menu.hpp
 * @brief       MAG live submenu: view mode, axis, unit, back (4 slots)
 * @author      Mat RB
 * @date        2026-08-26
 */

#ifndef MAG_LIVE_MENU_HPP
#define MAG_LIVE_MENU_HPP

#include "display/layout/screen_geom.hpp"

namespace display_layout {

/** Four equal slots across full bar width (Mode / Axis / Unit / Back). */
static constexpr uint8_t kMagLiveMenuCount = 4U;
static constexpr uint8_t kMagLiveMenuSelectCount = 4U;

enum MagLiveMenuId : uint8_t {
    eMagLiveMode = 0,
    eMagLiveAxis = 1,
    eMagLiveUnit = 2,
    eMagLiveBack = 3,
};

enum MagViewMode : uint8_t {
    eMagViewTime = 0,
    eMagViewHz   = 1,
};

static constexpr uint8_t kMagViewModeCount = 2U;

/**
 * Axis selection.
 * TIME view: ALL overlays X/Y/Z; X/Y/Z single trace.
 * Hz view: ABS = sqrt(x^2+y^2+z^2); X/Y/Z single channel (ALL maps to ABS).
 */
enum MagAxisMode : uint8_t {
    eMagAxisAll = 0, /**< TIME: all traces; Hz: treated as ABS */
    eMagAxisX   = 1,
    eMagAxisY   = 2,
    eMagAxisZ   = 3,
};

static constexpr uint8_t kMagAxisModeCount = 4U;

enum MagUnitMode : uint8_t {
    eMagUnitUT = 0, /**< microtesla */
    eMagUnitMT = 1, /**< millitesla */
    eMagUnitMG = 2, /**< milligauss */
    eMagUnitG  = 3, /**< gauss */
};

static constexpr uint8_t kMagUnitModeCount = 4U;

static inline Rect mag_live_menu_slot_rect(uint8_t index)
{
    const uint16_t slot_w = kScreenW / kMagLiveMenuCount;
    const uint16_t x0 = static_cast<uint16_t>(index * slot_w);
    const uint16_t x1 = static_cast<uint16_t>(
        (index == (kMagLiveMenuCount - 1U)) ? (kScreenW - 1U) : (x0 + slot_w - 1U));
    return Rect{x0, kBarY, x1, 239};
}

static inline const char* mag_view_label(uint8_t mode)
{
    switch (mode) {
        case eMagViewTime: return "TIME";
        case eMagViewHz:   return "Hz";
        default:           return "";
    }
}

static inline const char* mag_axis_label(uint8_t mode, uint8_t view_mode)
{
    if ((view_mode == eMagViewHz) && (mode == eMagAxisAll)) {
        return "ABS";
    }
    switch (mode) {
        case eMagAxisAll: return "ALL";
        case eMagAxisX:   return "X";
        case eMagAxisY:   return "Y";
        case eMagAxisZ:   return "Z";
        default:          return "";
    }
}

static inline const char* mag_unit_label(uint8_t mode)
{
    switch (mode) {
        case eMagUnitUT: return "uT";
        case eMagUnitMT: return "mT";
        case eMagUnitMG: return "mG";
        case eMagUnitG:  return "G";
        default:         return "";
    }
}

/** Multiply Gauss → display unit. */
static inline float mag_gauss_to_unit(float gauss, uint8_t unit)
{
    switch (unit) {
        case eMagUnitUT: return gauss * 100.0f;
        case eMagUnitMT: return gauss * 0.1f;
        case eMagUnitMG: return gauss * 1000.0f;
        case eMagUnitG:  return gauss;
        default:         return gauss * 100.0f;
    }
}

} /* namespace display_layout */

#endif /* MAG_LIVE_MENU_HPP */
