# AGENTS.md — ESP32-S3 Libraries

Persistent guidance for AI agents working in this repository.

## Mandatory skill

**Always** load and follow `embedded-systems-engineer` (`d:\AI_Workspaces\.cursor\skills\embedded-systems-engineer\SKILL.md`) before designing, writing, reviewing, or refactoring any firmware, driver, or example in this repo.

For module split / naming / include layers, also follow `embedded-modular-design` (`d:\AI_Workspaces\.cursor\skills\embedded-modular-design\SKILL.md`).

Do not invent ad-hoc driver layouts. Match the patterns below (reference implementation: `include/gpio_api.hpp` + `src/gpio_api.cpp`).

---

## Repo purpose

Reusable ESP32-S3 (ESP-IDF) peripheral driver libraries — not application firmware.

| Layer | Location | Role |
|-------|----------|------|
| Public API | `include/<name>_api.hpp` | Types, config, class / handlers; no ESP-IDF driver headers |
| Implementation | `src/<name>_api.cpp` | ESP-IDF / FreeRTOS calls, mapping helpers, logging |
| PlatformIO core lib | root `library.json` (`ESP32-S3_Libraries`) | **gpio_api + spi_api + i2c_api + uart_api + sd_api + ble_api**; sensors under `include/sensors/` (`hmc5883l`, `hmc5983`, `amg8833`, `bmi160`, `l76k`) — no ST7789 dependency |
| PlatformIO display lib | `display/library.json` (`esp32_s3_display`) | `display_api` / layout / ST7789 interface; depends on `ESP32-S3_Libraries` + app-provided `st7789` |
| Examples | `examples/` and `src/*_example.cpp` | App-side demos; product apps (e.g. proto_esp32_s3) consume via `lib_deps` |

**Sensor modules:** register maps in `include/sensors/*_regs.hpp`; drivers in `include/sensors/*_api.hpp` + `src/sensors/*_api.cpp`. Example firmware under `src/` is picked up by the repo's CMake GLOB but excluded from the PlatformIO library `srcFilter`.

**Consumers:** add `lib_deps = symlink://…/ESP32-S3_Libraries` and `symlink://…/ESP32-S3_Libraries/display`. Apps that use the display stack must vendor LibDriver ST7789 themselves (e.g. `lib/st7789` in the app). Do **not** put `st7789` on the root gpio/spi package.

Platform: ESP32-S3, ESP-IDF via PlatformIO (`platformio.ini`). Do not hardcode board pin numbers, clocks, or peripheral instances in library code — callers pass them via config.

---

## How to create a new module

### 1. File pair (required)

Always create a matching pair:

```
include/<peripheral>_api.hpp
src/<peripheral>_api.cpp
```

Never add a `.cpp` without a public `.hpp`. All wiring must work via `#include` (portable across PlatformIO / ESP-IDF / other IDEs).

### 2. Header contract (from `gpio_api.hpp`)

- File Doxygen block: `@file`, `@brief`, `@details`, `@platform`, `@version`.
- Include only: `<stdint.h>`, `<stdbool.h>`, `esp_err.h`, and other **project** headers. **Do not** include `driver/*.h` in the public header.
- Own the domain types: project enums / structs (`*_t`) with safe defaults. Map to ESP-IDF types only inside the `.cpp`.
- Prefer a config struct (`*_cfg_t`) with default member initializers; optional convenience constructors that fill the config.
- Public methods return `esp_err_t` (except trivial const getters). Document capability limits with `@note` and return `ESP_ERR_NOT_SUPPORTED` where hardware cannot do it.
- Explicit `init()` / `deinit()`: constructors must not touch hardware; destructor must safely tear down (ISR detach, then deinit) even if never inited.
- Keep ISR callbacks documented as ISR-context: no blocking, no malloc.
- Private state uses `m_` prefix; file-static / class-static shared resources (e.g. one-shot ISR service install) are allowed when hardware requires it.

### 3. Implementation contract (from `gpio_api.cpp`)

- Include order: project header → framework (`driver/*`, `esp_log.h`, `esp_check.h`, FreeRTOS as needed).
- Module `TAG` for `ESP_LOG*`.
- File-scope `static` mappers: project enum → ESP-IDF type (`toEsp*`, `toRtc*`). No raw IDF enums in the public API.
- Guard every mutating call: if not inited → `ESP_ERR_INVALID_STATE` (log + return). Idempotent `init`/`deinit`/`detach` may return `ESP_OK` with a warning.
- Prefer `ESP_RETURN_ON_ERROR(...)` for IDF calls; log pin/resource id and `esp_err_to_name`.
- Branch on hardware domain / capability; validate args early (`ESP_ERR_INVALID_ARG`).
- No dynamic allocation without bounds; no blocking work in ISRs; no floating-point unless justified.
- Do not embed application policy (button debounce UX, task loops, pin maps) inside the driver unless it is an optional, clearly documented feature of the peripheral API.

### 4. Size and cohesion

| Signal | Action |
|--------|--------|
| `.cpp` still one peripheral / one class and cohesive | Keep as one module (even if >300 lines) |
| Second peripheral, protocol, or unrelated concern appears | New `<name>_api` pair |
| Stateless helpers reused by multiple drivers | `util_<name>.hpp` / `util_<name>.cpp` |
| App wiring, FSM, MQTT, payloads | Do **not** put in this library repo — those belong in the product firmware (`app_*`, `state_*`) |

Library modules are `<peripheral>_api` (this repo). Product firmware uses `*_mod` / `app_*` / `state_*` / `util_*` per embedded-modular-design.

### 5. Include dependency rules

```
examples / app_main
        │
        ▼
  <peripheral>_api.hpp   ← public contract only (esp_err_t + project types)
        │
        ▼
  <peripheral>_api.cpp   ← driver/*.h, FreeRTOS, esp_log (implementation only)
```

- Drivers must not `#include` application or example headers.
- Drivers must not `#include` other `<peripheral>_api` headers unless extracting shared types into `types_<domain>.hpp`.
- No circular includes.

### 6. Checklist before finishing a module

- [ ] `embedded-systems-engineer` skill applied (timing, ISR, power, errors, no hardcoded HW)
- [ ] Matching `include/` + `src/` pair; header free of `driver/*.h`
- [ ] Config + `init()`/`deinit()`; safe destructor
- [ ] All public ops return / document `esp_err_t`; unsupported paths return `ESP_ERR_NOT_SUPPORTED`
- [ ] IDF type mapping confined to `.cpp`
- [ ] Logging + `ESP_RETURN_ON_ERROR` on IDF calls
- [ ] No board-specific pin/clock hardcodes in the library
- [ ] Example (if any) keeps pin numbers in the example only
- [ ] Brief note of flash/RAM/power or blocking behavior when non-obvious (e.g. debounced read blocking the caller)

---

## Reference: what `gpio_api` got right (copy this)

1. **HAL isolation** — `driver/gpio.h` / `driver/rtc_io.h` only in `.cpp`.
2. **Owned enums** — direction, pull, drive, ISR edge, domain; mapped with static helpers.
3. **Lazy hardware** — construct → `init()` → use; destructor cleans up.
4. **Capability honesty** — RTC OD / drive / wakeup paths return `ESP_ERR_NOT_SUPPORTED`.
5. **State guards** — `m_inited` / `m_isr_attached` checked before ops.
6. **Shared ISR service** — one-time `gpio_install_isr_service` behind a static guard.

## Anti-patterns (do not repeat)

| Anti-pattern | Fix |
|--------------|-----|
| `driver/*.h` in public header | Keep IDF types in `.cpp`; expose project enums + `esp_err_t` |
| Hardware touched in constructor | Config only in ctor; `init()` programs the peripheral |
| Swallowing IDF errors | Propagate `esp_err_t`; log with `TAG` and resource id |
| Board pins inside `*_api` | Pins/instances come from `*_cfg_t` or convenience args |
| App loops / product policy in the driver | Put demos under examples; keep the API peripheral-focused |
| `.cpp` without `.hpp` | Always ship the public header pair |
| New module without reading this file + `embedded-systems-engineer` | Stop; load the skill first |

---

## When editing existing modules

Preserve the public API unless the user explicitly requests a breaking change. Prefer additive methods, stronger validation, and clearer docs over renames. Match existing naming (`eGpio*`, `gpio_*_t`, `m_`, `k*` constants) within a module; for a **new** peripheral, mirror the same style with that peripheral’s prefix.
