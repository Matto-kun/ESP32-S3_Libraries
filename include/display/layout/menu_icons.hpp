/**
 * @file        menu_icons.hpp
 * @brief       RGB565 icons for root (BLE/WIFI) and SENSOR submenu
 * @author      Mat RB
 * @date        2026-08-12
 *
 * @details
 *   Glyphs use fillRect / draw_diag. Icon-band pixels stay above
 *   kIconDrawY1. SENSOR submenu bar uses text for IR / MAG; BACK is an icon.
 */

#ifndef MENU_ICONS_HPP
#define MENU_ICONS_HPP

#include "display/display_api.hpp"
#include "display/layout/screen_geom.hpp"
#include "display/layout/menu_bar.hpp"
#include "display/layout/sensor_menu.hpp"

namespace display_layout {

/**
 * @brief Draw a horizontal span of 1-px steps approximating a diagonal.
 */
static inline esp_err_t draw_diag(Display& d, uint16_t x0, uint16_t y0,
                                  int16_t dx, int16_t dy, uint16_t len, uint16_t fg)
{
    for (uint16_t i = 0U; i < len; i++) {
        const uint16_t x = static_cast<uint16_t>(static_cast<int16_t>(x0) + (dx * static_cast<int16_t>(i)));
        const uint16_t y = static_cast<uint16_t>(static_cast<int16_t>(y0) + (dy * static_cast<int16_t>(i)));
        if (y > kIconDrawY1) {
            break;
        }
        const esp_err_t err = d.fillRect(x, y, static_cast<uint16_t>(x + 1U), y, fg);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

/**
 * @brief Classic Bluetooth rune (stem + upper/lower right chevrons).
 */
static inline esp_err_t draw_ble_rune(Display& d, uint16_t cx, uint16_t cy, uint16_t fg)
{
    /* Vertical stem */
    esp_err_t err = d.fillRect(static_cast<uint16_t>(cx - 1U), static_cast<uint16_t>(cy - 18U),
                               static_cast<uint16_t>(cx + 1U), static_cast<uint16_t>(cy + 18U), fg);
    if (err != ESP_OK) {
        return err;
    }

    err = draw_diag(d, cx, static_cast<uint16_t>(cy - 18U), 1, 1, 14U, fg);
    if (err != ESP_OK) {
        return err;
    }
    err = draw_diag(d, static_cast<uint16_t>(cx + 13U), static_cast<uint16_t>(cy - 5U), -1, 1, 14U, fg);
    if (err != ESP_OK) {
        return err;
    }

    err = draw_diag(d, cx, cy, 1, 1, 14U, fg);
    if (err != ESP_OK) {
        return err;
    }
    err = draw_diag(d, static_cast<uint16_t>(cx + 13U), static_cast<uint16_t>(cy + 13U), -1, 1, 14U, fg);
    if (err != ESP_OK) {
        return err;
    }

    err = draw_diag(d, cx, static_cast<uint16_t>(cy - 18U), -1, 1, 14U, fg);
    if (err != ESP_OK) {
        return err;
    }
    err = draw_diag(d, static_cast<uint16_t>(cx - 13U), static_cast<uint16_t>(cy - 5U), 1, 1, 14U, fg);
    if (err != ESP_OK) {
        return err;
    }
    err = draw_diag(d, cx, cy, -1, 1, 14U, fg);
    if (err != ESP_OK) {
        return err;
    }
    return draw_diag(d, static_cast<uint16_t>(cx - 13U), static_cast<uint16_t>(cy + 13U), 1, 1, 14U, fg);
}

/**
 * @brief Clear icon band and draw a root-mode icon (BLE / WIFI).
 * @note eMenuSensor clears the band only — submenu owns SENSOR icons.
 */
static inline esp_err_t draw_mode_icon(Display& d, uint8_t mode, uint16_t fg, uint16_t bg)
{
    esp_err_t err = d.fillRect(kIconBand.x0, kIconBand.y0, kIconBand.x1, kIconBand.y1, bg);
    if (err != ESP_OK) {
        return err;
    }

    const uint16_t cx = kScreenW / 2U;
    const uint16_t draw_h = static_cast<uint16_t>(kIconDrawY1 - kIconBandY);
    const uint16_t cy = static_cast<uint16_t>(kIconBandY + (draw_h / 2U));

    switch (mode) {
        case eMenuBle:
            return draw_ble_rune(d, cx, cy, fg);

        case eMenuWifi:
            err = d.fillRect(static_cast<uint16_t>(cx - 22U), static_cast<uint16_t>(cy - 18U),
                             static_cast<uint16_t>(cx + 22U), static_cast<uint16_t>(cy - 15U), fg);
            if (err != ESP_OK) {
                return err;
            }
            err = d.fillRect(static_cast<uint16_t>(cx - 14U), static_cast<uint16_t>(cy - 6U),
                             static_cast<uint16_t>(cx + 14U), static_cast<uint16_t>(cy - 3U), fg);
            if (err != ESP_OK) {
                return err;
            }
            return d.fillRect(static_cast<uint16_t>(cx - 4U), static_cast<uint16_t>(cy + 6U),
                              static_cast<uint16_t>(cx + 4U), static_cast<uint16_t>(cy + 14U), fg);

        case eMenuSensor:
        default:
            return ESP_OK;
    }
}

/**
 * @brief Compact IR thermal-camera glyph (body + lens + heat bars).
 */
static inline esp_err_t draw_ir_cam_glyph(Display& d, uint16_t cx, uint16_t cy,
                                         uint16_t half_w, uint16_t half_h,
                                         uint16_t fg, uint16_t bg)
{
    esp_err_t err = d.fillRect(static_cast<uint16_t>(cx - half_w), static_cast<uint16_t>(cy - half_h),
                               static_cast<uint16_t>(cx + half_w), static_cast<uint16_t>(cy + half_h), fg);
    if (err != ESP_OK) {
        return err;
    }
    /* Hollow interior (match slot / band background) */
    const uint16_t inset = (half_w > 4U) ? 2U : 1U;
    err = d.fillRect(static_cast<uint16_t>(cx - half_w + inset), static_cast<uint16_t>(cy - half_h + inset),
                     static_cast<uint16_t>(cx + half_w - inset), static_cast<uint16_t>(cy + half_h - inset),
                     bg);
    if (err != ESP_OK) {
        return err;
    }
    /* Lens */
    const uint16_t lr = (half_h > 6U) ? 4U : 2U;
    err = d.fillRect(static_cast<uint16_t>(cx - lr), static_cast<uint16_t>(cy - lr),
                     static_cast<uint16_t>(cx + lr), static_cast<uint16_t>(cy + lr), fg);
    if (err != ESP_OK) {
        return err;
    }
    /* Heat bars to the right of the body */
    const uint16_t bx = static_cast<uint16_t>(cx + half_w + 2U);
    err = d.fillRect(bx, static_cast<uint16_t>(cy - half_h),
                     static_cast<uint16_t>(bx + 1U), static_cast<uint16_t>(cy - 2U), fg);
    if (err != ESP_OK) {
        return err;
    }
    return d.fillRect(static_cast<uint16_t>(bx + 3U), static_cast<uint16_t>(cy - (half_h / 2U)),
                      static_cast<uint16_t>(bx + 4U), static_cast<uint16_t>(cy + half_h), fg);
}

/**
 * @brief Compact magnetometer glyph (U-magnet / field mark).
 */
static inline esp_err_t draw_mag_glyph(Display& d, uint16_t cx, uint16_t cy,
                                       uint16_t half_w, uint16_t half_h, uint16_t fg)
{
    /* Left pole */
    esp_err_t err = d.fillRect(static_cast<uint16_t>(cx - half_w), static_cast<uint16_t>(cy - half_h),
                               static_cast<uint16_t>(cx - half_w + 2U), static_cast<uint16_t>(cy + half_h), fg);
    if (err != ESP_OK) {
        return err;
    }
    /* Right pole */
    err = d.fillRect(static_cast<uint16_t>(cx + half_w - 2U), static_cast<uint16_t>(cy - half_h),
                     static_cast<uint16_t>(cx + half_w), static_cast<uint16_t>(cy + half_h), fg);
    if (err != ESP_OK) {
        return err;
    }
    /* Bottom bridge */
    err = d.fillRect(static_cast<uint16_t>(cx - half_w), static_cast<uint16_t>(cy + half_h - 2U),
                     static_cast<uint16_t>(cx + half_w), static_cast<uint16_t>(cy + half_h), fg);
    if (err != ESP_OK) {
        return err;
    }
    /* Field diamond above */
    return d.fillRect(static_cast<uint16_t>(cx - 2U), static_cast<uint16_t>(cy - half_h - 4U),
                      static_cast<uint16_t>(cx + 2U), static_cast<uint16_t>(cy - half_h), fg);
}

/**
 * @brief Compact back chevron (`<`) for bar / navigation (no icon-band Y clamp).
 */
static inline esp_err_t draw_back_glyph(Display& d, uint16_t cx, uint16_t cy,
                                        uint16_t half_h, uint16_t fg)
{
    for (uint16_t i = 0U; i < half_h; i++) {
        const uint16_t x = static_cast<uint16_t>(cx - i);
        esp_err_t err = d.fillRect(x, static_cast<uint16_t>(cy - half_h + i),
                                   static_cast<uint16_t>(x + 1U),
                                   static_cast<uint16_t>(cy - half_h + i), fg);
        if (err != ESP_OK) {
            return err;
        }
        err = d.fillRect(x, static_cast<uint16_t>(cy + half_h - i),
                         static_cast<uint16_t>(x + 1U),
                         static_cast<uint16_t>(cy + half_h - i), fg);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

/**
 * @brief Compact forward chevron (`>`) — mirror of draw_back_glyph.
 */
static inline esp_err_t draw_forward_glyph(Display& d, uint16_t cx, uint16_t cy,
                                           uint16_t half_h, uint16_t fg)
{
    for (uint16_t i = 0U; i < half_h; i++) {
        const uint16_t x = static_cast<uint16_t>(cx + i);
        esp_err_t err = d.fillRect(x, static_cast<uint16_t>(cy - half_h + i),
                                   static_cast<uint16_t>(x + 1U),
                                   static_cast<uint16_t>(cy - half_h + i), fg);
        if (err != ESP_OK) {
            return err;
        }
        err = d.fillRect(x, static_cast<uint16_t>(cy + half_h - i),
                         static_cast<uint16_t>(x + 1U),
                         static_cast<uint16_t>(cy + half_h - i), fg);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

/**
 * @brief Draw the forward-arrow slot (optional selection highlight).
 */
static inline esp_err_t draw_advance_arrow_slot(Display& d, const Rect& ar,
                                                uint16_t slot_bg, uint16_t arrow_fg)
{
    esp_err_t err = d.fillRect(ar.x0, ar.y0, ar.x1, ar.y1, slot_bg);
    if (err != ESP_OK) {
        return err;
    }
    const uint16_t cx = static_cast<uint16_t>((ar.x0 + ar.x1) / 2U);
    const uint16_t cy = static_cast<uint16_t>((ar.y0 + ar.y1) / 2U);
    return draw_forward_glyph(d, cx, cy, 10U, arrow_fg);
}

/**
 * @brief SENSOR submenu bar: IR/MAG text, back icon, selectable forward.
 */
static inline esp_err_t draw_sensor_menu_bar(Display& d, uint8_t selected,
                                            uint16_t bar_bg, uint16_t sel_bg,
                                            uint16_t text_fg, uint16_t arrow_fg)
{
    esp_err_t err = d.fillRect(kBarRect.x0, kBarRect.y0, kBarRect.x1, kBarRect.y1, bar_bg);
    if (err != ESP_OK) {
        return err;
    }

    for (uint8_t i = 0U; i < kSensorMenuCount; i++) {
        const Rect r = sensor_menu_slot_rect(i);
        if (i == selected) {
            err = d.fillRect(r.x0, r.y0, r.x1, r.y1, sel_bg);
            if (err != ESP_OK) {
                return err;
            }
        }

        if (i == eSensorBack) {
            const uint16_t cx = static_cast<uint16_t>((r.x0 + r.x1) / 2U);
            const uint16_t cy = static_cast<uint16_t>((r.y0 + r.y1) / 2U);
            err = draw_back_glyph(d, cx, cy, 10U, text_fg);
        } else {
            const uint16_t tx = static_cast<uint16_t>(r.x0 + 4U);
            const uint16_t ty = static_cast<uint16_t>(kBarY + 14U);
            err = d.drawString(tx, ty, sensor_menu_label(i), text_fg);
        }
        if (err != ESP_OK) {
            return err;
        }
    }

    const Rect ar = sensor_menu_arrow_rect();
    const uint16_t fwd_bg = (selected == eSensorFwd) ? sel_bg : bar_bg;
    return draw_advance_arrow_slot(d, ar, fwd_bg, arrow_fg);
}

/**
 * @brief Clear icon band and draw IR or MAG mode icon (BACK/FWD are navigation).
 */
static inline esp_err_t draw_sensor_mode_icon(Display& d, uint8_t sensor_id,
                                             uint16_t fg, uint16_t bg)
{
    esp_err_t err = d.fillRect(kIconBand.x0, kIconBand.y0, kIconBand.x1, kIconBand.y1, bg);
    if (err != ESP_OK) {
        return err;
    }

    if ((sensor_id == eSensorBack) || (sensor_id == eSensorFwd)) {
        return ESP_OK;
    }

    const uint16_t cx = kScreenW / 2U;
    const uint16_t draw_h = static_cast<uint16_t>(kIconDrawY1 - kIconBandY);
    const uint16_t cy = static_cast<uint16_t>(kIconBandY + (draw_h / 2U));

    if (sensor_id == eSensorIrCam) {
        return draw_ir_cam_glyph(d, cx, cy, 28U, 20U, fg, bg);
    }
    if (sensor_id == eSensorMag) {
        return draw_mag_glyph(d, cx, cy, 24U, 18U, fg);
    }
    return ESP_OK;
}

/**
 * @brief Draw bottom menu bar with selection highlight and selectable forward.
 */
static inline esp_err_t draw_menu_bar(Display& d, uint8_t selected,
                                      uint16_t bar_bg, uint16_t sel_bg,
                                      uint16_t text_fg, uint16_t arrow_fg)
{
    esp_err_t err = d.fillRect(kBarRect.x0, kBarRect.y0, kBarRect.x1, kBarRect.y1, bar_bg);
    if (err != ESP_OK) {
        return err;
    }

    for (uint8_t i = 0U; i < kMenuCount; i++) {
        const Rect r = menu_slot_rect(i);
        if (i == selected) {
            err = d.fillRect(r.x0, r.y0, r.x1, r.y1, sel_bg);
            if (err != ESP_OK) {
                return err;
            }
        }
        const uint16_t tx = static_cast<uint16_t>(r.x0 + 4U);
        const uint16_t ty = static_cast<uint16_t>(kBarY + 14U);
        err = d.drawString(tx, ty, menu_label(i), text_fg);
        if (err != ESP_OK) {
            return err;
        }
    }

    const Rect ar = menu_arrow_rect();
    const uint16_t fwd_bg = (selected == eMenuFwd) ? sel_bg : bar_bg;
    return draw_advance_arrow_slot(d, ar, fwd_bg, arrow_fg);
}

} /* namespace display_layout */

#endif /* MENU_ICONS_HPP */
