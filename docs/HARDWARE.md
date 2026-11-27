# Hardware

The physical robot: what is on it, how it is wired, and where the mechanical and
electrical source files live.

> **Status of the mechanical/electrical files.** The 3D-printable body, the PCB
> and the bill of materials are being added. Each section below names the exact
> path the files will live at, so links from other documents stay valid once the
> files land. See [hardware/](hardware/README.md) for the directory and its
> conventions.

## Summary

| Item | Part | Interface |
|---|---|---|
| MCU | ESP32-S3-DevKitM-1 | USB-C, 2.4 GHz WiFi |
| IMU | MPU9250 (GY-91 breakout) | I2C `0x68` |
| Display | SSD1306 OLED 128×64 | I2C `0x3C` |
| Actuators | 2× hobby servo (leg down / leg up) | PWM |
| Power | USB-C or external 5 V; 12 V for the drive stage | — |

## Pin map

The authoritative values are in [include/config.h](../include/config.h). This
table documents them; if the two disagree, the header wins.

| Signal | GPIO | Notes |
|---|---|---|
| I2C SDA | 21 | shared by OLED and IMU |
| I2C SCL | 22 | shared by OLED and IMU |
| Servo, down leg | 7 | `SERVO_PIN_DOWN` |
| Servo, up leg | 6 | `SERVO_PIN_UP` |
| Servo power | 5 V external | **not** from the board's regulator |

> ⚠️ **Servo pins are 6 and 7, not 32/33.** Older course material said 32/33,
> which `ESP32Servo` 0.13.0 rejects on the S3 — attaching there produces
> `ATTACH_FAILED` (code 1) in the boot health check. See
> [SERVO_HEALTH.md](SERVO_HEALTH.md#pin-note-for-the-esp32-s3).

## Wiring

```text
                 ┌──────────────────────────┐
    USB-C ───────┤ ESP32-S3-DevKitM-1       │
                 │                          │
                 │  GPIO21 ●──── SDA ───────┼──┬── OLED  SDA
                 │  GPIO22 ●──── SCL ───────┼──┴── MPU9250 SCL
                 │                          │
                 │   GPIO7 ●────────────────┼──── down-leg servo signal
                 │   GPIO6 ●────────────────┼──── up-leg   servo signal
                 │                          │
                 │  3V3    ●────────────────┼──── OLED VCC, IMU VCC
                 │  GND    ●──── common ────┼──── all grounds
                 └──────────────────────────┘
                 external 5 V ──────────────┬──── servo V+ (both)
                                            └──── servo GND (common with board)
```

Rules that matter:

* **Servos get their own 5 V supply.** Current spikes from two servos moving
  together will brown out the board if they share the USB rail.
* **All grounds are common.** Board, servos and supply.
* The I2C bus is short and shared; both devices are on 3.3 V logic.
* Keep the IMU away from the servo wires, or magnetic calibration degrades.

## Bill of materials

> **Placeholder.** The component spreadsheet will live at
> [`hardware/bom/crawling-bot-v2-bom.xlsx`](hardware/bom/README.md).
> Until it is added, the summary table above is the authoritative parts list.

Planned content of that sheet: reference designator, part number, description,
quantity, package, supplier, unit price, notes. It will be the single source for
ordering and for the assembly guide.

## 3D-printable body

> **Placeholder.** Printable files will live under
> [`hardware/3d/`](hardware/3d/README.md).
>
> Expected layout:
>
> ```text
> hardware/3d/
>   stl/           printable, oriented and scaled
>   step/          editable CAD export
>   drawings/      dimensioned 2D drawings (PDF)
>   README.md      print settings, material, assembly order
> ```
>
> Print settings (layer height, walls, infill, supports, material) belong in that
> README, next to the files they apply to, so a reprint reproduces the original
> part.

## PCB

> **Placeholder.** Design files will live under
> [`hardware/pcb/`](hardware/pcb/README.md).
>
> Expected layout:
>
> ```text
> hardware/pcb/
>   crawling-bot-v2.kicad_pro / .kicad_sch / .kicad_pcb
>   gerbers/       fabrication output (zip)
>   bom/           assembly BOM in the fabricator's format
>   README.md      stackup, design rules, connectors, test points
> ```

## Images

| File | Shows |
|---|---|
| [`hardware/img/robot-overview.jpg`](hardware/img/README.md) | the assembled robot |
| [`hardware/img/wiring.jpg`](hardware/img/README.md) | harness and connectors |

> **Placeholder.** Add the photographs to [`hardware/img/`](hardware/img/README.md)
> using the names above; the README and this page already reference them.

## Assembly order

The firmware's boot sequence constrains the mechanical assembly:

1. Mount the servos and set the legs to the **park pose (180/180)** — the
   firmware parks there at power-on and cannot know where the horns are.
2. Route the servo wiring away from the IMU.
3. Connect the I2C devices.
4. Power on with the legs **off the ground** for the first boot.
5. Run `CAL` (see [CALIBRATION.md](CALIBRATION.md)) with the robot on a solid,
   level surface.

## Related reading

* [QUICKSTART.md](QUICKSTART.md) — first power-on
* [SERVO_SAFETY.md](SERVO_SAFETY.md) — what protects the mechanics
* [SERVO_HEALTH.md](SERVO_HEALTH.md) — diagnosing a dead servo
