# Technical summary

One screen of orientation. Every statement here is expanded — and is
authoritative — in the linked document.

## What this repository is

**Robot firmware.** The robot runs it; student controllers run elsewhere and
connect over the network. Nothing in a student's project is compiled into the
firmware.

| Question | Answer | Details |
|---|---|---|
| How do I control the robot? | ASCII lines on serial or TCP `:4242` | [PROTOCOL.md](PROTOCOL.md) |
| How do I write a client? | any language; Python reference included | [languages/](languages/README.md), [examples/python](../examples/python/README.md) |
| What is the robot made of? | ESP32-S3, MPU9250, SSD1306 OLED, 2 servos | [HARDWARE.md](HARDWARE.md) |
| How is the firmware built? | PlatformIO, modular libraries | [ARCHITECTURE.md](ARCHITECTURE.md) |
| What is the course structure? | setup → interface → agent → training → evaluation | [PROJECT.md](PROJECT.md) |
| How is the IMU implemented? | PX4-derived filters, estimator, calibration | [imu/](imu/README.md) |

## Features

| Area | Capability |
|---|---|
| Sensing | 200 Hz IMU task: calibrated accel/gyro/mag, quaternion attitude, still detection, velocity/position estimate |
| Actuation | two servos, ramped motion, soft limits, motion health check |
| Interface | 9 commands over serial + TCP, one reply each, non-blocking I/O |
| Telemetry | 50-column CSV stream at up to 50 Hz, append-only format |
| Calibration | guided 3-stage procedure, stored per robot in NVS, verified at boot |
| Reliability | hardware watchdog, non-blocking boot, shared-bus mutex, OTA |
| Observability | OLED pages, structured `WARN,`/`ERROR,`/`PROTOCOL,` log lines |

## Library map

| Library | Responsibility | Depends on |
|---|---|---|
| `IMU` | MPU9250, filters, estimator, calibration | `I2CBus` |
| `ServoControl` | PWM, motion profile, limits, self-test | — |
| `Display` | SSD1306 drawing | `I2CBus` |
| `Network` | WiFi AP, OTA, robot identity (EEPROM) | — |
| `CommandServer` | protocol parsing, serial/TCP transports, streaming | all of the above |
| `HealthCheck` | boot servo verification | `ServoControl`, `IMU` |
| `I2CBus` | recursive mutex + bus bring-up | — |
| `SystemWatchdog` | `esp_task_wdt` subscription and feeding | — |

`src/main.cpp` wires them together and owns the boot order and the screen.

## Numbers worth knowing

| Setting | Value | Where |
|---|---|---|
| IMU sample rate | 200 Hz, core 1 | `IMU_TASK_*` in `config.h` |
| Telemetry rate | 10 Hz serial default, ≤ 50 Hz | `STREAM`, `PROTOCOL_MAX_STREAM_HZ` |
| Servo cruise / accel | 180 °/s, 600 °/s² | `SERVO_MAX_SPEED_DPS`, `SERVO_ACCEL_DPS2` |
| Watchdog | 3 s, panic + reset | `WATCHDOG_TIMEOUT_S` |
| I2C bus | 400 kHz, shared | `I2C_CLOCK_HZ` |
| TCP clients | 4 | `PROTOCOL_MAX_CLIENTS` |
| Line length | 96 bytes | `PROTOCOL_LINE_MAX` |

## Requirements

**Hardware** — ESP32-S3-DevKitM-1, MPU9250 (GY-91) at `0x68`, SSD1306 128×64 at
`0x3C`, 2 hobby servos on GPIO 6/7, external 5 V for the servos.

**Software** — PlatformIO; Python 3.8+ for the example client; a C++ compiler
for the host tests.

## Getting started

1. `pio run --target upload`
2. join `ESP32-AP-<n>` (password `12345678`)
3. `python examples/python/hello_robot.py --host 192.168.4.1`
4. if it reports `calibrated: False`, run `python calibrate.py`

## Testing

| Suite | Command | Covers |
|---|---|---|
| C++ host tests | `test/host/run.sh` | IMU math, filters, estimator, calibration fits, motion profile, protocol grammar |
| Python client tests | `python -m unittest discover -s examples/python/tests` | parsing, every client method, error and timeout paths |

## Status

Firmware feature-complete. See [CHANGELOG.md](../CHANGELOG.md). Mechanical and
electrical sources are being added under [hardware/](hardware/README.md).
