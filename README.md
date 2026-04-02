<div align="center">

# `fw_sensor_logger`

![MCU](https://img.shields.io/badge/MCU-STM32F446RETx-03a9f4?style=flat-square&logo=stmicroelectronics&logoColor=white)
![UART](https://img.shields.io/badge/UART-115200%208N1-4caf50?style=flat-square)
![I2C](https://img.shields.io/badge/I2C-BME280%200x76-ff9800?style=flat-square)
![ADC](https://img.shields.io/badge/ADC-Thermistor%20%7C%20LDR-9c27b0?style=flat-square)
![Watchdog](https://img.shields.io/badge/IWDG-Enabled-f44336?style=flat-square)

**STM32F446 firmware** that logs environmental sensor data to an internal ring buffer,  
exposes a UART command-line interface, and integrates a Bosch BME280 (I²C)  
together with ADC readings for a thermistor and photoresistor (LDR).

</div>

---

## 📋 Table of Contents

- [Target Hardware](#-target-hardware)
- [Features](#-features)
- [Repository Layout](#-repository-layout)
- [Toolchain & Build](#-toolchain--build)
- [Flashing & Debug](#-flashing--debug)
- [UART Commands](#-uart-commands)
- [BME280 I²C Address](#-bme280-ic-address)
- [Fault Policy](#-fault-policy)
- [License & Third-Party](#-license--third-party)

---

## 🔧 Target Hardware

| Item | Detail |
|------|--------|
| **MCU** | STM32F446RETx — LQFP64 (see `.ioc` → `Mcu.UserName`) |
| **UART** | USART2 · `PA2` = TX · `PA3` = RX *(ST-Link virtual COM on Nucleo)* |
| **I2C** | I2C1 · `PB8` = SCL · `PB9` = SDA *(BME280)* |
| **ADC1** | `PA1` = `ADC1_IN1` (thermistor) · `PA4` = `ADC1_IN4` (LDR) — 2 sequential conversions per `HAL_ADC_Start` |
| **Timer** | TIM2 · `Prescaler=8399` · `Period=999` *(exact Hz depends on APB timer clock)* |
| **Watchdog** | IWDG · `Prescaler=IWDG_PRESCALER_64` · `Reload=1499` *(order-of-second timeout with LSI — verify on hardware)* |

> Other pins (`PC13` user button, `PA5` LD2) are initialized in Cube; the sample application focuses on UART + sensors.

---

## ✨ Features

- **Multi-sensor sample** (`Sample` in `Core/Inc/app/app_types.h`): timestamp, thermistor °C, LDR %, optional BME280 T/P/RH when the bus read succeeds
- **Ring buffer** — 512 entries, overwrite-on-full policy, global overwrite counter via `error_stats.ring_overwrite`
- **BME280** via official Bosch driver (`Core/Src/bme280.c`) + app wrapper (`Core/Src/app/bme280_app.c`): I²C glue, forced-mode read, re-init after repeated failures (throttled), I²C fail counter via `error_stats.bme_i2c_fail`
- **UART CLI** (`Core/Src/app/cli_uart.c`): line-based commands, local echo, `status` prints the latest ring sample
- **Fault escalation** — after repeated failed BME re-inits, firmware may stop refreshing IWDG so the watchdog resets the system

---

## 📁 Repository Layout

| Path | Purpose |
|------|---------|
| `Core/Src/main.c` | Cube/HAL init and main loop — calls `app_init()` / `app_loop_tick()` |
| `Core/Src/app/` | **Application logic** (modular): orchestration, BME280, ring buffer, ADC path, UART CLI |
| `Core/Inc/app/` | Application headers — `app_types.h`, `app.h`, module APIs |
| `Core/Src/bme280.c` · `Core/Inc/bme280*.h` | Bosch BME280 vendor driver |
| `Drivers/` | STM32 HAL / CMSIS (ST) |
| `fw_sensor_logger_rebuild.ioc` | STM32CubeMX project — pins, clocks, peripherals |
| `Debug/` | Makefile-based build (`makefile`, `objects.list`, `Core/Src/app/subdir.mk`) |

---

## 🛠 Toolchain & Build

**Compiler:** GNU Arm Embedded (`arm-none-eabi-gcc`), as configured by STM32CubeIDE / GNU Tools for STM32.

Command-line build from repo root:

```bash
mingw32-make -C Debug -j all
```

Output artifact:

```
Debug/fw_sensor_logger_rebuild.elf
```

*(Plus generated `.map` / listing depending on post-build steps.)*

> **STM32CubeIDE:** Import the project and ensure `Core/Src/app/*.c` are part of the build. If the IDE project file does not yet list them, add the sources or regenerate from `.ioc` and merge carefully.

---

## ⚡ Flashing & Debug

Use **ST-Link** (on-board on Nucleo) or your usual programmer. Flash the generated ELF — convert to HEX/BIN if your tool requires it.

Connect a serial terminal to the ST-Link VCP on **USART2**:

| Setting | Value |
|---------|-------|
| Baud rate | `115200` |
| Data / Parity / Stop | `8N1` |
| Init reference | `MX_USART2_UART_Init` in `Core/Src/main.c` |

---

## 💬 UART Commands

Type commands followed by **Enter** (CR/LF). Characters are echoed.

| Command | Action |
|---------|--------|
| `start` | Start TIM2 interrupt — drives periodic sampling in the main loop |
| `stop` | Stop TIM2 interrupt — halts new sample ticks |
| `reset` | Re-initialize the ring buffer and clear the CLI's cached sample snapshot |
| `status` | Print the latest sample in the ring (thermistor, LDR, BME280 if last read OK) |
| `help` | List all available commands |

**Typical workflow:** send `start` → wait for data → send `status`.

> If the ring is empty, firmware prints a hint to run `start` first.

---

## 📡 BME280 I²C Address

Default in `Core/Inc/app/bme280_app.h`:

```c
BME280_I2C_ADDR_PRIM  // 0x76
```

> **SDO strapped to VDD?** Use `BME280_I2C_ADDR_SEC` (`0x77`) instead.

---

## ⚠️ Fault Policy

1. **Consecutive failures** — BME read/init failures increment internal counters. After **3 failures** within a **5 s throttle window**, firmware calls `bme280_app_init()` again.

2. **Fault latch** — if re-init still leaves the sensor not ready repeatedly, an internal fault latch may set. The main loop then **stops calling `HAL_IWDG_Refresh`**, allowing an IWDG watchdog reset.

See [`Core/Src/app/bme280_app.c`](Core/Src/app/bme280_app.c) and [`Core/Src/app/app.c`](Core/Src/app/app.c) for exact conditions and comments.

---

## 📄 License & Third-Party

| Component | License |
|-----------|---------|
| ST HAL / CMSIS | See ST license in `Drivers/` |
| Bosch BME280 C driver | See license headers in `Core/Src/bme280.c` and `Core/Inc/bme280*.h` |

---

<div align="center">

*This README describes the rebuild firmware layout — adjust board-specific wiring and Cube settings if you fork the project.*

</div>
