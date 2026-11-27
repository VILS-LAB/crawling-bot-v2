# Firmware architecture

How the firmware is put together: the boot sequence, the tasks, the shared
resources, the logging, and what happens when something fails. The IMU signal
chain has its own document ([imu/ARCHITECTURE.md](imu/ARCHITECTURE.md)); this
page covers everything around it.

## Design intent

The robot is a **service, not a library**. Students connect from outside; they
do not link against firmware code, and they do not reflash to change their
algorithm. Everything below follows from that:

1. The interface must be **stable and documented** → one line protocol
   ([PROTOCOL.md](PROTOCOL.md)).
2. The robot must **survive a client that misbehaves** → non-blocking I/O, a
   watchdog, per-client slots.
3. The robot must **never be left in a dangerous state** → servos park on an
   invalid calibration, ramped moves, soft limits.
4. The robot must be **diagnosable without a debugger** → structured log lines
   and an OLED that says what is wrong in words.

## Boot sequence

```text
Serial + watchdog
  │
  ├─ display.begin()            I2C bus up, OLED init (non-fatal on failure)
  ├─ network.begin()            EEPROM robot number, WiFi AP, mDNS
  ├─ network.startOTATask()     OTA on core 1
  ├─ ahrs.begin(n)              MPU9250 setup, load calibration from NVS
  ├─ servos.begin()             attach PWM, park at 180/180
  ├─ ahrs.calibrate()           VERIFY stored calibration (never a full run)
  │    └─ invalid → servos stay parked, "Uncalibrated - run CAL"
  ├─ ahrs.startTask()           IMU sampling task on core 1
  ├─ healthCheck.run()          only with a valid calibration (moves the legs!)
  └─ commands.begin()           TCP listener + serial reader
```

Two rules encoded in that order:

* **Nothing waits for a human.** An erased EEPROM defaults to robot 1; a missing
  calibration leaves the servos parked and continues. The robot is always reachable.
* **The servo self-test runs last.** It moves the legs, which would corrupt a
  calibration still being verified.

Failure handling per stage:

| Stage fails | Behaviour |
|---|---|
| OLED | logged, display stays blank, robot continues |
| WiFi AP | logged, OTA unavailable, serial still works |
| MPU9250 | logged, `CAL`/`GET` report unavailable, robot stays reachable |
| calibration | servos remain parked and `cal=0`; guided `CAL` is available (the `ACT` handler itself does not enforce calibration state) |
| IMU task | logged, loop task unsubscribed from the watchdog, robot halts visibly |
| servo attach | logged, health check reports code 1, remainder of boot continues |

## Tasks and cores

| Core | Runs | Why |
|---|---|---|
| 0 | Arduino `loop()` and command server | the loop must stay responsive to commands |
| 1 | IMU sampling + estimation (nominal 200 Hz), OTA handler | separate tasks; actual IMU rate is measured and may vary |

The IMU task publishes a snapshot under a spinlock; `loop()` copies it. Public
getters read only the copy, so no locks are needed in client-facing code.

## Shared resources

### I2C bus

The OLED and the MPU9250 share SDA/SCL but are driven from different tasks.
Every access takes a recursive mutex (`lib/I2CBus`) with a timeout: a display
transfer cannot interleave with a sensor read, and a stuck bus makes the caller
skip its transaction instead of blocking the loop.

Related detail: the SSD1306 library temporarily changes the bus clock. It is
constructed so both "during" and "after" are 400 kHz — otherwise every display
refresh would drop the bus to 100 kHz under the 200 Hz IMU reads.

### Servo state

Owned entirely by the loop task. `CommandServer::update()` and the health check
both run there, so servo calls need no locking. The IMU task never touches them.

### Serial port

Two writers: the command server (replies, telemetry) and the firmware's own boot
and diagnostic lines. They are both on the loop task, so lines never interleave
mid-frame. The IMU task does not print; it publishes fields instead.

## Concurrency limits

| Limit | Value | Consequence |
|---|---|---|
| TCP clients | `PROTOCOL_MAX_CLIENTS` (4) | a fifth connection is refused with `ERR busy` |
| line length | `PROTOCOL_LINE_MAX` (96) | longer lines are rejected and reported |
| stream rate | `PROTOCOL_MAX_STREAM_HZ` (50) | higher requests are clamped, not refused |
| command timeout | none on the robot | a client may think for as long as it likes |

Nothing on the robot is unbounded: every buffer is fixed at compile time, and
every socket write is non-blocking.

## Logging

Three line families, all ASCII, all greppable:

| Prefix | Meaning | Consumer |
|---|---|---|
| `OK` / `ERR` | reply to exactly one command | the client that asked |
| `IMU,` | telemetry frame (CSV) | `tools/robot_visualizer.py`, CSV logs |
| `CAL,` / `CALRESULT,` | calibration progress and outcome | the calibrating client |
| everything else | diagnostics: `WARN,`, `ERROR,`, `HEALTH,`, `PROTOCOL,` | a human reading the log |

`tools/robot_visualizer.py` consumes all four families at once: `IMU,` becomes
the charts, `HEALTH,`/`CAL,`/`PROTOCOL,` become its health panel, and `OK`/`ERR`
become the reply shown beside each console button. That is why the viewer can
report *why* something is wrong rather than only *that* it is.

The diagnostics are stable enough to grep. Examples:

```text
WARN,ROBOT_NUM_INVALID,defaulting_to_1_use_SETID
ERROR,IMU_INIT_FAILED
PROTOCOL,READY,serial=115200,tcp_port=4242,stream_hz=10
PROTOCOL,CLIENT,connected,ip=192.168.4.2
PROTOCOL,CLIENT,dropped,reason=stream_stalled
CAL,SELF_CHECK,failed,run_CAL
ROBOT,UNCALIBRATED,send_CAL_to_calibrate
```

> The display shows a short version of the same state. Pages: boot message,
> "Uncalibrated - run CAL", telemetry (attitude, acceleration, speed,
> temperature), OTA progress. A richer multi-page UI is planned; see
> [hardware/img/](hardware/img/README.md) for the page photographs.

## Failure modes

| Failure | Detection | Containment |
|---|---|---|
| Loop task hangs | 3 s hardware watchdog | panic + reset |
| IMU task hangs | same watchdog | panic + reset |
| Client stops reading | non-blocking write fails | that client is dropped, others unaffected |
| Client floods commands | fixed per-loop read | the robot stays responsive |
| Long calibration | watchdog suspended for the run | no spurious reset |
| I2C bus wedged | mutex timeout | the transfer is skipped, robot keeps running |
| OLED missing | init returns false | display is a no-op, robot boots |
| EEPROM erased | range check | default robot 1, warning printed |
| Calibration invalid | boot self-check | servos remain parked, `cal=0`; no `ACT` interlock is implemented |

## Where to change things

| Want to… | Edit |
|---|---|
| change pins, rates, limits | [include/config.h](../include/config.h) |
| add a protocol command | `lib/CommandServer` (parser + handler) |
| change servo behaviour | `lib/ServoControl` |
| change the attitude filter | `lib/IMU` (see [imu/](imu/README.md)) |
| change what the OLED shows | `src/main.cpp` |
| add a boot step | `setup()` in `src/main.cpp`, and update the table above |

## Related reading

* [imu/ARCHITECTURE.md](imu/ARCHITECTURE.md) — the sensor pipeline in depth
* [PROTOCOL.md](PROTOCOL.md) — the external interface
* [DEVELOPMENT.md](DEVELOPMENT.md) — how to build, test and release
