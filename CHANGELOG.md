# Changelog

Firmware versions, newest first. Dates are release dates.

## Unreleased

### Changed

* **Repository scope.** This project is now the *robot firmware* only. Student
  work runs off-board as a client over serial/TCP rather than by editing and
  flashing the firmware; the `lib/Training` stubs were removed.
* Documentation restructured around the protocol: new
  [docs/PROTOCOL.md](docs/PROTOCOL.md), [docs/PROJECT.md](docs/PROJECT.md),
  [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md),
  [docs/HARDWARE.md](docs/HARDWARE.md), [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)
  and per-language guides under [docs/languages/](docs/languages/README.md).
* Persian doc copies (`*.fa.md`) removed so each fact has one home. They had
  drifted out of date against the firmware.

### Added

* **Reference Python client** in [examples/python](examples/python/README.md):
  typed `Snapshot`/`ImuFrame` results, deadlines, event/reply separation,
  streaming iterator, calibration with progress, plus runnable scripts and 20
  tests that run against a fake robot.
* **`crawlingbot` on PyPI.** The Python client is now a real distribution, so
  students install it with `pip install crawlingbot` instead of copying files.
  Packaging lives in [examples/python/pyproject.toml](examples/python/pyproject.toml);
  the release process is in [docs/PYPI.md](docs/PYPI.md), automated by
  [.github/workflows/publish-python-client.yml](.github/workflows/publish-python-client.yml)
  with trusted publishing.
* **Command protocol** on both USB serial and TCP port 4242:
  `PING`, `ID`, `SETID`, `GET`, `STREAM`, `ACT`, `STOP`, `PARK`, `CAL`.
* **Hardware documentation structure** under [docs/hardware/](docs/hardware/README.md)
  with placeholders for 3D print files, PCB design and the BOM spreadsheet.
* **`tools/robot_visualizer.py`**, a robot dashboard and command console: the
  twelve charts of the old IMU visualizer consolidated into six, plus a 3D body,
  heading dial, pass/fail health panel and a button for every protocol command.
  Reads serial, TCP, a recorded CSV, or synthetic `--demo` data. See
  [tools/README.md](tools/README.md).
* `CHANGELOG.md` (this file).

### Fixed

* Docs claimed the servos are on GPIO 32/33; the firmware uses 6/7, and 32/33
  are rejected by `ESP32Servo` on the S3. [docs/HARDWARE.md](docs/HARDWARE.md)
  now documents the real pin map and the failure mode.
* An `ACT` command with two angles could lose its second value in the protocol
  parser; covered by `test/host/test_command_parser.cpp`.
* TCP streaming could stall the loop because `WiFiClient::write()` blocks for
  seconds on a peer that stopped reading; writes are now non-blocking and a
  stalled client is dropped.

### Removed

* `lib/Training` placeholder library.
* The legacy single-key IMU console (`q`, then `1`–`6`); `CAL` replaces it.
* `*.fa.md` documentation mirrors (see *Changed*).

### Deprecated

* `tools/imu_visualizer.py` and `tools/imu_visualizer_v2.py` are superseded by
  `tools/robot_visualizer.py`. They still run and nothing depends on them; they
  are kept for their raw-vs-filtered gyro view and interactive sliders.

## 2026-11-26 — firmware baseline

Boot and reliability work that this release builds on:

* Non-blocking boot: an invalid EEPROM defaults to robot 1 instead of waiting on
  serial; a missing/invalid calibration parks the servos and shows
  `Uncalibrated - run CAL` instead of forcing a full recalibration.
* Shared I2C mutex between the OLED (core 0) and the 200 Hz IMU task (core 1).
* Hardware task watchdog (`esp_task_wdt`, 3 s, panic + reset) covering the loop
  task and the IMU task, suspended during guided calibration.
