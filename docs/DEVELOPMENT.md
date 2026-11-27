# Working on the firmware

Conventions, tooling and the release process for maintainers of this repository.

> If you are a student writing a client, you do not need this page. Start at
> [PROTOCOL.md](PROTOCOL.md) and [examples/python](../examples/python/README.md).

## Prerequisites

* PlatformIO (CLI or the VS Code extension)
* The ESP32-S3 toolchain (installed automatically by PlatformIO)
* A C++ compiler for the host tests (`c++` on the PATH)
* Python 3.8+ for the example client and the tools

## Everyday commands

```bash
pio run                       # build
pio run --target upload       # flash over USB
pio device monitor -b 115200  # see the log

test/host/run.sh                                           # C++ tests, ~1 s
cd examples/python && python -m unittest discover -s tests  # client tests

pio run --target upload --upload-port ESP32-OTA-<n>.local  # over the air
```

The Python client's packaging tools (`build`, `twine`, `pytest`) are not part of
a bare virtualenv. Install them into this repository's `env/` once:

```bash
cd examples/python && ../../env/bin/python -m pip install -e ".[dev]"
cd examples/python && ../../env/bin/python -m build   # then twine check dist/*
```

See [PYPI.md](PYPI.md) for the full release path.

## Layout rules

| Directory | Owns | Must not |
|---|---|---|
| `src/` | wiring the modules together, the UI loop | contain hardware drivers |
| `lib/<Name>/` | one responsibility, behind a small header | depend on `src/` |
| `include/` | configuration that a maintainer tunes | contain behaviour |
| `test/host/` | code that runs on a PC | require hardware |
| `examples/` | runnable client code | be built into the firmware |
| `docs/` | the documentation | duplicate the BOM or pin table |

Two conventions make the test story work:

1. **Keep pure logic free of Arduino.** Anything that can be unit tested
   (motion profiles, filter math, protocol grammar) lives in a file that does not
   include `Arduino.h`, so `test/host/run.sh` can compile it directly.
2. **One source of truth per fact.** Pin numbers live in `include/config.h`; the
   BOM lives in the spreadsheet; the protocol lives in `lib/CommandServer` and
   `docs/PROTOCOL.md`. Documents reference them instead of restating them.

## Adding a protocol command

1. Add the verb to `lib/CommandServer/CommandParser.{h,cpp}` and to the
   `Verb` enum; extend `test/host/test_command_parser.cpp`.
2. Add a handler and dispatch entry in `CommandServer.cpp`; keep replies to a
   single line and report failures as `ERR <code>,<detail>`.
3. Document it in `docs/PROTOCOL.md` (grammar, reply, error codes).
4. Wrap it in the Python client (`examples/python/crawlingbot/client.py`) and
   extend `examples/python/tests/fake_robot.py` and `test_client.py`.
5. If it changes the streaming format, append columns; never reorder them.

## Adding a client language

Create `docs/languages/<language>.md` with: connection snippet, send/receive
loop, one worked example (GET → ACT), error handling, and threading advice. Link
it from [languages/README.md](languages/README.md). Any snippet that could be
copy-pasted must run.

## Testing expectations

| Change | Must pass |
|---|---|
| any firmware change | `pio run` |
| math, filters, motion, protocol grammar | `test/host/run.sh` |
| client library | its unittest suite |
| docs only | link check by eye; no build required |

New behaviour needs a test at the lowest level that can express it. If a bug
escapes to hardware, add the smallest host test that would have caught it.

## Commit and review

* One logical change per commit; the message says *why*.
* A behaviour change updates the document that describes the behaviour in the
  same commit.
* Pin-map and BOM-affecting changes update [HARDWARE.md](HARDWARE.md) too, and
  are announced to the course staff before students flash.

## Release process

1. `pio run` and `test/host/run.sh` green, client tests green.
2. Update [CHANGELOG.md](../CHANGELOG.md).
3. Verify the protocol document matches the firmware (verbs, reply fields,
   error codes).
4. Tag the release; flash **one** robot and run the phase-0 gate from
   [PROJECT.md](PROJECT.md) before rolling out to the lab.

## Documentation map

| Page | Owner | Update when |
|---|---|---|
| [PROTOCOL.md](PROTOCOL.md) | interface | any command or field changes |
| [ARCHITECTURE.md](ARCHITECTURE.md) | firmware | tasks, boot order, failure modes change |
| [HARDWARE.md](HARDWARE.md) | hardware | pin map, parts or files change |
| [CALIBRATION.md](CALIBRATION.md) | firmware | the calibration procedure changes |
| [PROJECT.md](PROJECT.md) | course | phases or deliverables change |
| [imu/](imu/README.md) | firmware | the sensor pipeline changes |
