/**
 * @file        sensor_menu.hpp
 * @brief       SENSOR submenu: IR, MAG, back + selectable forward
 * @author      Mat RB
 * @date        2026-08-12
 */

#ifndef SENSOR_MENU_HPP
#define SENSOR_MENU_HPP

#include "display/layout/menu_bar.hpp"

namespace display_layout {

/** Content slots (IR / MAG / BACK) — geometry only. */
static constexpr uint8_t kSensorMenuCount = 3U;
/** Selectable positions including forward arrow (accept is a no-op for now). */
static constexpr uint8_t kSensorMenuSelectCount = 4U;

enum SensorMenuId : uint8_t {
    eSensorIrCam = 0,
    eSensorMag   = 1,
    eSensorBack  = 2,
    eSensorFwd   = 3,
};

/** Same 3-slot + arrow geometry as the root menu bar. */
static inline Rect sensor_menu_slot_rect(uint8_t index)
{
    return menu_slot_rect(index);
}

static inline Rect sensor_menu_arrow_rect(void)
{
    return menu_arrow_rect();
}

/** Bar text for IR / MAG; BACK / FWD use icons (empty label). */
static inline const char* sensor_menu_label(uint8_t index)
{
    switch (index) {
        case eSensorIrCam: return "IR";
        case eSensorMag:   return "MAG";
        case eSensorBack:  return "";
        case eSensorFwd:   return "";
        default:           return "";
    }
}

} /* namespace display_layout */

#endif /* SENSOR_MENU_HPP */
