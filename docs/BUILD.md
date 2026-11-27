# Build Instructions

## Prerequisites

1. Install PlatformIO Core or PlatformIO IDE
   - **PlatformIO Core**: `pip install platformio`
   - **PlatformIO IDE**: Install VS Code extension "PlatformIO IDE"

## Building the Project

### Using PlatformIO Core (Command Line)

```bash
# Navigate to project directory
cd crawling-bot-v2

# Build the project
pio run

# Upload to ESP32-S3
pio run --target upload

# Monitor serial output
pio device monitor -b 115200

# Build, upload, and monitor in one command
pio run --target upload && pio device monitor -b 115200
```

### Using PlatformIO IDE (VS Code)

1. Open the project folder in VS Code
2. PlatformIO will automatically detect the project
3. Use the PlatformIO toolbar at the bottom:
   - Click "Build" (✓) to compile
   - Click "Upload" (→) to flash to board
   - Click "Serial Monitor" to view output

## First-Time Setup

The robot number (1–8) names the WiFi access point and the OTA hostname.

**It no longer prompts on first boot.** An erased or invalid EEPROM defaults the
robot to **1** and logs a warning, so the robot always comes up:

```text
WARN,ROBOT_NUM_INVALID,defaulting_to_1_use_SETID
```

To choose a number, send `SETID` and reboot:

```text
SETID 3
OK id=3
```

The number is written to EEPROM immediately; the access point is renamed at the
next boot (`ESP32-AP-3`, `ESP32-OTA-3`).

## Hardware connections

The pin map, wiring diagram, power rules and mechanical assembly order live in
[HARDWARE.md](HARDWARE.md) so there is one place to keep them correct. In short:
OLED and MPU9250 on the shared I2C bus (0x3C and 0x68), servos on **GPIO 7 and
6**, servos on their own 5 V supply, all grounds common.

## Troubleshooting

### Build Issues

1. **Platform not found**: Run `pio pkg install` to download ESP32 platform
2. **Library errors**: Dependencies are auto-installed from platformio.ini
3. **Compilation errors**: Ensure you have the latest PlatformIO version

### Runtime issues

| Symptom | Check |
|---|---|
| OLED blank, robot otherwise fine | I2C wiring, address 0x3C. A missing display is non-fatal by design. |
| `ERROR,IMU_INIT_FAILED` | IMU wiring, address 0x68, 3.3 V supply |
| OLED says `Uncalibrated - run CAL` | send `CAL` (see [CALIBRATION.md](CALIBRATION.md)) |
| `HEALTH,SERVO,...,code=1` | servo signal pin must be 6 or 7 (S3 PWM whitelist) |
| `HEALTH,SERVO,...,code=4` | servo has no power or is mechanically stuck |
| Legs remain parked after boot | `GET` reports `cal=`; boot parks them and skips the motion health check when `cal=0`. The `ACT` command itself is not calibration-interlocked; calibrate first and do not command motion beforehand. |
| No reply on TCP 4242 | joined the robot's WiFi? port blocked? try serial |
| `PROTOCOL,CLIENT,dropped,reason=stream_stalled` | the client stopped draining `STREAM` |
| Robot reboots repeatedly | a task stops feeding the 3 s watchdog; read the log for the last line before reset |
| OTA does not find the robot | you must be on `ESP32-AP-<n>`; mDNS name is `ESP32-OTA-<n>.local` |

## Development workflow

> **Students:** you do not flash the robot. The firmware is the platform you
> program against — see [PROTOCOL.md](PROTOCOL.md). This section is for
> maintainers.

1. Make code changes
2. Build: `pio run`
3. Run the host tests: `test/host/run.sh`
4. Fix any failures
5. Upload: `pio run --target upload`
6. Monitor: `pio device monitor -b 115200`
7. Test on hardware; for subsequent updates use OTA

Conventions, the release process and what must be updated when the protocol
changes: [DEVELOPMENT.md](DEVELOPMENT.md).

## OTA Updates (After Initial Upload)

1. Connect to robot's WiFi AP: `ESP32-AP-{robot_number}`
2. Password: `12345678`
3. Upload via OTA:
   ```bash
   pio run --target upload --upload-port ESP32-OTA-{robot_number}.local
   ```

## Calibration

Boot only *verifies* a stored calibration; it never blocks and never forces a
full run. If it is missing or fails, the robot parks the servos, shows
`Uncalibrated - run CAL` and accepts the `CAL` command.

```bash
cd examples/python && python calibrate.py
```

Full procedure, stage details and every failure message:
[CALIBRATION.md](CALIBRATION.md).

## Testing

Three layers, cheapest first:

| Layer | Command | Needs hardware |
|---|---|---|
| C++ host tests | `test/host/run.sh` | no |
| Python client tests | `python -m unittest discover -s examples/python/tests` | no |
| On-robot self-test | power cycle and watch the log | yes |

The on-robot self-test is the boot **health check**: it verifies the servos
electrically and by sweeping them while watching the gyro. It only runs when a
valid calibration is present, because it deliberately moves the legs. Codes and
their meaning: [SERVO_HEALTH.md](SERVO_HEALTH.md).

A good boot ends with:

```text
HEALTH,SUMMARY,ok=1,down=0,up=0
PROTOCOL,READY,serial=115200,tcp_port=4242,stream_hz=10
```
