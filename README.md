# ESP32-S3_Libraries

Reusable ESP32-S3 peripheral and sensor drivers for **ESP-IDF** via PlatformIO.

This repository is a **library**, not product firmware. Application projects (for example `proto_esp32_s3`) consume it through `lib_deps`.

## Requirements

| Item | Value |
|------|--------|
| MCU | ESP32-S3 |
| Framework | ESP-IDF (PlatformIO `framework = espidf`) |
| Platform | `espressif32` (tested with 7.0.1 / IDF 6.x) |
| Language | C++ |

Board pins, baud rates, and peripheral instances stay in the **application** config. Library modules take them through `*_cfg_t` structs.

## Packages

| Package | Path | Contents |
|---------|------|----------|
| `ESP32-S3_Libraries` | root [`library.json`](library.json) | `gpio_api`, `spi_api`, `i2c_api`, `uart_api`, `sd_api`, sensors |
| `esp32_s3_display` | [`display/library.json`](display/library.json) | `display_api`, layout helpers, ST7789 interface glue |
| `st7789` | app-vendored (e.g. `lib/st7789`) | LibDriver ST7789 — **not** in the root package |

## Core modules

| Module | Header | Role |
|--------|--------|------|
| GPIO | [`include/gpio_api.hpp`](include/gpio_api.hpp) | Digital I/O, pull, debounce, ISR attach |
| SPI | [`include/spi_api.hpp`](include/spi_api.hpp) | SPI master (host 2/3); CS owned by caller |
| I2C | [`include/i2c_api.hpp`](include/i2c_api.hpp) | I2C master ports 0/1 |
| UART | [`include/uart_api.hpp`](include/uart_api.hpp) | UART ports 0–2 |
| SD | [`include/sd_api.hpp`](include/sd_api.hpp) | SDSPI + FatFS mount |

Public headers do **not** include `driver/*.h`. ESP-IDF types stay in the `.cpp` files.

## Sensor modules

| Module | Headers | Bus |
|--------|---------|-----|
| HMC5883L | `include/sensors/hmc5883l_*.hpp` | I2C |
| HMC5983 | `include/sensors/hmc5983_*.hpp` | I2C |
| AMG8833 | `include/sensors/amg8833_*.hpp` | I2C |
| BMI160 | `include/sensors/bmi160_*.hpp` | I2C (accel + gyro; CHIP_ID `0xD1`) |
| L76K GNSS | `include/sensors/l76k_*.hpp` | UART (NMEA RMC + PMTK helpers) |

Register maps live in `*_regs.hpp`. Drivers live in `*_api.hpp` / `src/sensors/*_api.cpp`.

## Display package

Use `symlink://…/ESP32-S3_Libraries/display` and provide LibDriver `st7789` from the app.

- API: [`include/display/display_api.hpp`](include/display/display_api.hpp)
- Board wiring: [`include/display/display_board.hpp`](include/display/display_board.hpp)
- Layout / menus: [`include/display/layout/`](include/display/layout/)

## Consumer setup (PlatformIO)

```ini
lib_ldf_mode = deep+
lib_ignore = ST7789

lib_deps =
    symlink://../ESP32-S3_Libraries
    symlink://../ESP32-S3_Libraries/display
    symlink://lib/st7789
```

Omit the display lines if the app does not use the panel.

### Minimal usage

```cpp
#include "uart_api.hpp"
#include "sensors/l76k_api.hpp"

uart_cfg_t ucfg = {};
ucfg.port = eUartPort1;
ucfg.tx_pin = 2;
ucfg.rx_pin = 1;
ucfg.baud_hz = 9600U;

Uart uart(ucfg);
uart.init();

L76k gnss(uart, {});
gnss.init();

l76k_rmc_t rmc = {};
gnss.pollRmc(rmc, 2500U);
```

Always call `init()` before use. Destructors call `deinit()` when the object was initialized.

## Examples

Demo sources under [`examples/`](examples/) keep pin numbers in the example only:

| File | Topic |
|------|--------|
| `gpio_example.cpp` / `gpio_simple_example.cpp` | GPIO |
| `hmc5883l_example.cpp` | Magnetometer |
| `amg8833_example.cpp` | IR Grid-EYE |
| `bmi160_example.cpp` | IMU accel / gyro |
| `l76k_example.cpp` | GNSS RMC |
| `sd_example.cpp` | SD card |
| `display_driver_smoke.cpp` / `display_menu_example.cpp` | ST7789 UI |

## Design rules

- Config + explicit `init()` / `deinit()` (no hardware work in constructors)
- Public ops return `esp_err_t` where failure is possible
- No board pin hardcodes inside `*_api` modules
- Prefer one peripheral / sensor per `*_api` pair

Agent guidance: [`AGENTS.md`](AGENTS.md).

## Changelog

See [`CHANGELOG.md`](CHANGELOG.md).

## License

MIT — see [`LICENSE`](LICENSE).
