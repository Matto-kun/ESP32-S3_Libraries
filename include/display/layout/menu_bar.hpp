/**
 * @file        menu_bar.hpp
 * @brief       Bottom menu bar slots: BLE, WIFI, SENSOR + selectable forward
 * @author      Mat RB
 * @date        2026-08-12
 */

#ifndef MENU_BAR_HPP
#define MENU_BAR_HPP

#include "display/layout/screen_geom.hpp"

namespace display_layout {

/** Content slots (BLE / WIFI / SENSOR) — geometry only. */
static constexpr uint8_t kMenuCount = 3U;
/** Selectable positions including forward arrow (accept is a no-op for now). */
static constexpr uint8_t kMenuSelectCount = 4U;

enum MenuId : uint8_t {
    eMenuBle    = 0,
    eMenuWifi   = 1,
    eMenuSensor = 2,
    eMenuFwd    = 3,
};

static constexpr uint16_t kArrowW = 40U;
static constexpr uint16_t kSlotsW = kScreenW - kArrowW; /* 200 */

static inline Rect menu_slot_rect(uint8_t index)
{
    const uint16_t slot_w = kSlotsW / kMenuCount;
    const uint16_t x0 = static_cast<uint16_t>(index * slot_w);
    const uint16_t x1 = static_cast<uint16_t>(
        (index == (kMenuCount - 1U)) ? (kSlotsW - 1U) : (x0 + slot_w - 1U));
    return Rect{x0, kBarY, x1, 239};
}

static inline Rect menu_arrow_rect(void)
{
    return Rect{kSlotsW, kBarY, 239, 239};
}

static inline const char* menu_label(uint8_t index)
{
    switch (index) {
        case eMenuBle:    return "BLE";
        case eMenuWifi:   return "WIFI";
        case eMenuSensor: return "SENSOR";
        default:          return "";
    }
}

/** Full label for SENSOR (narrow slot uses short "SENSOR") */
static inline const char* menu_label_full(uint8_t index)
{
    switch (index) {
        case eMenuBle:    return "BLE";
        case eMenuWifi:   return "WIFI";
        case eMenuSensor: return "SENSOR";
        default:          return "";
    }
}

} /* namespace display_layout */

#endif /* MENU_BAR_HPP */
