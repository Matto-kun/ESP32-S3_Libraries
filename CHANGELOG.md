# Changelog

All notable changes to **ESP32-S3_Libraries** are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html) for the root `library.json` package.

## [Unreleased]

### Added

- `uart_api` — UART ports 0–2 with config pins, baud, and RX/TX buffers; IDF types confined to the `.cpp`
- `sensors/l76k_api` + `l76k_regs` — L76K / L76X GNSS over UART: PMTK command framing, `$GNRMC` / `$GPRMC` parse, optional PPS GPIO
- `examples/l76k_example.cpp` — UART1 GNSS RMC demo (pins local to the example)
- `sensors/bmi160_api` + `bmi160_regs` — Bosch BMI160 IMU (CHIP_ID `0xD1`, I2C `0x68`/`0x69`, accel g + gyro dps)
- `examples/bmi160_example.cpp` — I2C1 BMI160 sample demo (pins local to the example)
- Root `README.md` rewritten for packages, modules, and consumer setup

### Changed

- Root `library.json` `srcFilter` now includes `uart_api.cpp`, `sensors/l76k_api.cpp`, and `sensors/bmi160_api.cpp`
- `AGENTS.md` package table lists UART, SD, L76K, and BMI160 with the other core drivers

### Notes

- `l76k_cfg_t.apply_nmea_cfg` defaults to `false` (safer for Quectel LC76G). Init still sends a best-effort `PMTK314,-1` restore
- GPIO numbers for GNSS TX/RX are **MCU** pins (MCU TX drives module RX)
- BMI160 pinout `addr7` must be `0x68` or `0x69`; `0xD1` is CHIP_ID, not the bus address

## [1.0.0] — 2026-08-31

First packaged library release used by `proto_esp32_s3` and related apps.

### Added

- `gpio_api` — digital I/O, pulls, debounce, ISR service attach
- `spi_api` — SPI master (SPI2 / SPI3), shared host refcount, CS left to the caller
- `i2c_api` — I2C master ports 0/1 with per-address device handles
- `sd_api` — SDSPI + FatFS mount / format helpers
- Sensors:
  - `hmc5883l_api` / `hmc5883l_regs`
  - `hmc5983_api` / `hmc5983_regs`
  - `amg8833_api` / `amg8833_regs` (Grid-EYE 8×8)
- Display package `esp32_s3_display` (`display/library.json`):
  - `display_api`, ST7789 interface glue, layout / menu helpers
  - Depends on root `ESP32-S3_Libraries` and app-provided LibDriver `st7789`
- Examples for GPIO, HMC5883L, AMG8833, SD, and display smoke / menu
- `AGENTS.md` — module layout and HAL isolation rules for agents

### Design

- Public headers expose project types + `esp_err_t` only (no `driver/*.h` leak)
- Explicit `init()` / `deinit()`; constructors do not program hardware
- Pins, clocks, and host/port IDs come from caller config

[Unreleased]: https://github.com/Matto-kun/ESP32-S3_Libraries/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/Matto-kun/ESP32-S3_Libraries/releases/tag/v1.0.0
