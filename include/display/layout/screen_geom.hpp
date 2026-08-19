/**
 * @file        screen_geom.hpp
 * @brief       240x240 screen geometry constants for display layout
 * @author      Mat RB
 * @date        2026-08-12
 */

#ifndef SCREEN_GEOM_HPP
#define SCREEN_GEOM_HPP

#include <stdint.h>

namespace display_layout {

static constexpr uint16_t kScreenW = 240U;
static constexpr uint16_t kScreenH = 240U;

static constexpr uint16_t kBarH           = 40U;
static constexpr uint16_t kBarY           = kScreenH - kBarH;          /* 200 */
static constexpr uint16_t kIconBandH      = 72U;
static constexpr uint16_t kIconBandY      = kBarY - kIconBandH;        /* 128 */
static constexpr uint16_t kIconPadBottom  = 8U;
/** Lowest Y for icon glyph pixels (keeps clear gap above the menu bar). */
static constexpr uint16_t kIconDrawY1     = kBarY - kIconPadBottom;    /* 192 */

/** Content / page area above the icon band */
static constexpr uint16_t kContentY0 = 0U;
static constexpr uint16_t kContentY1 = static_cast<uint16_t>(kIconBandY - 1U);

struct Rect {
    uint16_t x0;
    uint16_t y0;
    uint16_t x1;
    uint16_t y1;
};

static constexpr Rect kFullScreen = {0, 0, 239, 239};
static constexpr Rect kBarRect    = {0, kBarY, 239, 239};
static constexpr Rect kIconBand   = {0, kIconBandY, 239, static_cast<uint16_t>(kBarY - 1U)};
static constexpr Rect kContent    = {0, kContentY0, 239, kContentY1};

} /* namespace display_layout */

#endif /* SCREEN_GEOM_HPP */
