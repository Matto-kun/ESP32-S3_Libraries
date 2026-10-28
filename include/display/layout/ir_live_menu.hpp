/**
 * @file        ir_live_menu.hpp
 * @brief       IR live submenu: Filter, Level, Rot, Flip, Back
 * @author      Mat RB
 * @date        2026-08-26
 */

#ifndef IR_LIVE_MENU_HPP
#define IR_LIVE_MENU_HPP

#include "display/layout/screen_geom.hpp"

namespace display_layout {

static constexpr uint8_t kIrLiveMenuCount = 5U;
static constexpr uint8_t kIrLiveMenuSelectCount = 5U;

enum IrLiveMenuId : uint8_t {
    eIrLiveFilter = 0,
    eIrLiveLevel  = 1,
    eIrLiveRot    = 2,
    eIrLiveFlip   = 3,
    eIrLiveBack   = 4,
};

enum IrFilterMode : uint8_t {
    eIrFilterNear  = 0, /**< Nearest-neighbor upsample */
    eIrFilterBilin = 1, /**< Bilinear upsample */
    eIrFilterSharp = 2, /**< Bilinear + unsharp (uses Level) */
};

enum IrRotMode : uint8_t {
    eIrRot0   = 0,
    eIrRot90  = 1,
    eIrRot180 = 2,
    eIrRot270 = 3,
};

static constexpr uint8_t kIrFilterModeCount = 3U;
static constexpr uint8_t kIrSharpLevelCount = 4U; /* 0..3 */
static constexpr uint8_t kIrRotModeCount    = 4U;

static inline Rect ir_live_menu_slot_rect(uint8_t index)
{
    const uint16_t slot_w = kScreenW / kIrLiveMenuCount;
    const uint16_t x0 = static_cast<uint16_t>(index * slot_w);
    const uint16_t x1 = static_cast<uint16_t>(
        (index == (kIrLiveMenuCount - 1U)) ? (kScreenW - 1U) : (x0 + slot_w - 1U));
    return Rect{x0, kBarY, x1, 239};
}

static inline const char* ir_filter_label(uint8_t mode)
{
    switch (mode) {
        case eIrFilterNear:  return "NEAR";
        case eIrFilterBilin: return "BILIN";
        case eIrFilterSharp: return "SHARP";
        default:             return "";
    }
}

static inline const char* ir_level_label(uint8_t level)
{
    switch (level) {
        case 0U: return "L0";
        case 1U: return "L1";
        case 2U: return "L2";
        case 3U: return "L3";
        default: return "";
    }
}

static inline const char* ir_rot_label(uint8_t rot)
{
    switch (rot) {
        case eIrRot0:   return "R0";
        case eIrRot90:  return "R90";
        case eIrRot180: return "R180";
        case eIrRot270: return "R270";
        default:        return "";
    }
}

static inline const char* ir_flip_label(bool flip_on)
{
    return flip_on ? "FLIP" : "NORM";
}

} /* namespace display_layout */

#endif /* IR_LIVE_MENU_HPP */
