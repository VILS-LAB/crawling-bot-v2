# Project overview and phases

This page explains **what the robot is for** and **how the work is divided**. It
is deliberately separate from the technical documentation: [README.md](../README.md)
and [ARCHITECTURE.md](ARCHITECTURE.md) describe the machine,
[PROTOCOL.md](PROTOCOL.md) describes the interface, and this page describes the
course project that uses them.

> **Division of responsibility.** The firmware in this repository is the
> *platform*: it is maintained by the course staff and is expected to be stable.
> The *learning algorithm* is the student deliverable and runs off-board, in the
> student's language of choice, talking to the robot over WiFi or USB.

## Learning goal

Students implement reinforcement learning on real hardware. Because the firmware
is a fixed, documented service, the project focuses on the parts that matter for
the course:

* defining a **state** from noisy sensor data,
* defining an **action space** that respects the mechanics,
* designing a **reward** that produces the desired gait,
* handling **real-world stochasticity**: sensor noise, slipping, battery sag,
* running a training loop against a physical system, with logging and replay.

## Two roles

### The platform (this repository)

| Owned here | Notes |
|---|---|
| sensor sampling and attitude estimation | 200 Hz IMU task, PX4-derived filters |
| servo safety | ramped motion, soft limits, health check |
| the command protocol | serial + TCP, one reply per command |
| calibration and its persistence | guided procedure, NVS per robot |
| display, logging, watchdog | robot stays observable and recoverable |
| mechanical and electrical files | see [hardware/](hardware/README.md) |

### The student project

| Owned by students | Notes |
|---|---|
| client code | any language; Python reference client in [examples/python](../examples/python/README.md) |
| state/action/reward design | derived from `GET` / `STREAM` fields |
| learning algorithm | Q-learning, SARSA, DQN, policy gradient, … |
| experiment logging and analysis | the telemetry CSV is the raw material |
| final report and demo | |

## Phases

The phase structure is the course's, not the firmware's. The firmware supports
all of them without modification.

| Phase | Goal | What the robot provides | Done when |
|---|---|---|---|
| **0 — Setup** | robot boots, is reachable, is calibrated | non-blocking boot, AP + OTA, `CAL` | `hello_robot.py` prints a snapshot with `calibrated: True` |
| **1 — Interface** | read state, command actions, log data | `GET`, `STREAM`, `ACT`, `STOP`, `PARK` | a script logs 60 s of telemetry and replays a fixed action sequence |
| **2 — Agent** | define state/action/reward, implement the learner offline | telemetry CSV, deterministic command replies | the learner trains on recorded data |
| **3 — On-robot training** | close the loop on hardware | ramped actions, watchdog, `STOP` for safety | reward improves over episodes without damaging the robot |
| **4 — Evaluation** | measure and present | per-robot calibration, stable protocol | reproducible results and a working demo |

### Phase gates

* **Phase 0 gate** — the robot must report `cal=1`. A robot with an invalid
  calibration boots with the servos parked and refuses to move; this is
  intentional. See [CALIBRATION.md](CALIBRATION.md).
* **Phase 1 gate** — the client must survive a robot restart: reconnect, then
  re-read `ID` and `STREAM`. Clients that assume a permanent connection fail
  here, which is the point.
* **Phase 3 gate** — every training run must be interruptible. `STOP` freezes
  both legs immediately; a client that cannot reach it is not ready for
  hardware.

## Safety rules for hardware sessions

1. The legs are ramped and soft-limited, but they are still strong enough to
   pinch. Keep the robot on blocks during the first runs of a new policy.
2. `STOP` is the emergency command. Bind it to a key before anything else.
3. Do not run a training loop that cannot end: always cap episodes and total
   runtime in the client.
4. Battery sag changes servo behaviour. Watch the IMU temperature in `GET` and
   recharge before a long run.
5. Never flash firmware during a training run — use a separate robot or finish
   the run first.

## What students do **not** need to touch

`lib/`, `src/main.cpp`, `include/config.h` and the build configuration. If a
change to the platform seems necessary, it is a course-staff decision: open an
issue describing the interface gap rather than editing the firmware locally,
because every robot in the lab must run the same firmware for results to be
comparable.

## Related reading

* [README.md](../README.md) — firmware overview
* [PROTOCOL.md](PROTOCOL.md) — the interface students code against
* [HARDWARE.md](HARDWARE.md) — the physical robot
* [DEVELOPMENT.md](DEVELOPMENT.md) — working on the firmware itself
