# Robot command protocol

One ASCII line protocol, served on two transports at the same time:

| Transport | Address | Notes |
|---|---|---|
| USB serial | `115200` baud | the port that flashes/OTA too |
| TCP | `<robot ip>:4242` | AP mode, default `192.168.4.1:4242` |

Both speak exactly the same grammar, so anything written against the serial port
works unchanged over TCP and vice versa.

* Lines are `\n` terminated. A client may send `\r\n`; the `\r` is ignored.
* Commands and keywords are **case-insensitive** (`ping`, `Ping`, `PING`).
* Arguments are separated by one or more spaces or tabs.
* Every command answers with exactly **one** `OK ...` or `ERR ...` line.
* Telemetry frames (`IMU,...`) are *events*, not replies, and can be interleaved
  with replies at any time. A client must therefore accept that a line starting
  with `IMU,` or `CAL,` is not the answer to its command.

## Commands

| Command | Reply | Meaning |
|---|---|---|
| `PING` | `OK PONG <uptime_ms>` | liveness check, uptime since boot |
| `ID` | `OK id=<n> ssid=<ssid> ip=<ip>` | robot number and network identity |
| `SETID <1..8>` | `OK id=<n>` | store a new robot number in EEPROM |
| `GET` | `OK SNAP,...` | one-shot snapshot: attitude + servos |
| `STREAM <hz>` | `OK stream=<hz>` | telemetry rate, `0` = off, max `50` |
| `ACT <down> <up>` | `OK ACT d=<clamped> u=<clamped>` | move both legs |
| `STOP` | `OK STOP` | freeze both legs where they are |
| `PARK` | `OK PARK` | move both legs to 180/180 |
| `CAL [all\|gyro\|accel\|mag]` | `OK CAL sensors=<list>` | guided IMU calibration |

### `PING`

```
PING
OK PONG 123456
```

### `ID`

```
ID
OK id=3 ssid=ESP32-AP-3 ip=192.168.4.1
```

### `SETID`

```
SETID 4
OK id=4
SETID 9
ERR SETID_range,1_to_8
```

The new number is written to EEPROM immediately and is used by `ID` right away.
The access point (SSID) and the OTA hostname are built from the number at boot,
so they only change after a reset. Reconnect to `ESP32-AP-<new>` afterwards.

### `GET`

```
GET
OK SNAP,id=1,t=51402,roll=1.24,pitch=-0.31,yaw=97.05,ax=0.104,ay=0.021,az=9.807,gx=0.12,gy=-0.07,gz=0.03,mx=21.44,my=-8.02,mz=41.19,temp_c=31.25,still=1,trust=0.998,rate_hz=199.4,cal=1,d=180,dt=180,u=180,ut=180,act_moving=0
```

Field order is fixed; new fields are only appended.

| Field | Unit | Meaning |
|---|---|---|
| `id` | | robot number |
| `t` | ms | `millis()` when the snapshot was formatted |
| `roll`,`pitch`,`yaw` | deg | attitude, project convention |
| `ax`,`ay`,`az` | m/s² | calibrated body-frame acceleration (≈ 9.81 at rest) |
| `gx`,`gy`,`gz` | deg/s | bias-corrected angular rate |
| `mx`,`my`,`mz` | µT | calibrated magnetic field |
| `temp_c` | °C | IMU temperature |
| `still` | 0/1 | still detector result |
| `trust` | 0..1 | how much the accelerometer is currently fused |
| `rate_hz` | Hz | measured IMU sampling rate |
| `cal` | 0/1 | a valid calibration is loaded |
| `d`,`u` | deg | current leg angles |
| `dt`,`ut` | deg | commanded leg targets |
| `act_moving` | 0/1 | a ramp is still in progress |

### `STREAM`

```
STREAM 50
OK stream=50
IMU,51402,0.0050,0.1040,...
IMU,51422,0.0050,0.1035,...
STREAM 0
OK stream=0
```

* `STREAM` sets the rate **for the transport it arrived on**. The USB serial
  port streams at 10 Hz by default; every TCP client starts quiet.
* The rate is clamped to `PROTOCOL_MAX_STREAM_HZ` (50), which is the IMU task
  rate: asking for more just repeats samples.
* Frames keep the existing `IMU,` CSV layout, so
  `tools/imu_visualizer.py` and `tools/imu_visualizer_v2.py` work over TCP with
  no changes (point them at the socket instead of the serial port).
* A client that stops reading is disconnected rather than allowed to stall the
  robot. The byte stream is never cut mid-line.

### `ACT`

```
ACT 90 60
OK ACT d=90 u=60
ACT -20 300
OK ACT d=0 u=180
```

Angles are clamped to the soft limits in `include/config.h`
(`SERVO_DOWN_MIN_ANGLE` … `SERVO_UP_MAX_ANGLE`). The reply reports the
**clamped** target, so it is also how a client learns the limits. The move is
ramped: the reply means "accepted", not "arrived".

```
ACT 10
ERR ACT_usage,ACT_<down>_<up>
ACT abc 10
ERR ACT_usage,ACT_<down>_<up>
```

### `STOP` / `PARK`

```
STOP
OK STOP
PARK
OK PARK
```

`STOP` freezes both axes at their current angle (target := current). `PARK` is a
normal ramped move to 180/180, so it takes about 1.3 s at the default speed.

### `CAL`

```
CAL
OK CAL sensors=all
CAL,START,robot=1
CAL,gyro,starting
...
CAL,DONE,saved=1
CALRESULT,ok=1,sensors=all
```

* `CAL` with no argument or `all` runs gyro → accel → mag in the PX4 order.
* `CAL gyro`, `CAL accel`, `CAL mag` run one stage.
* The `OK CAL ...` line means "started". The final `CALRESULT,ok=<0|1>` line is
  the outcome; intermediate `CAL,...` lines report the current stage (on TCP
  they go to the client that asked, on serial they were already being printed).
* The robot must stay still for the gyro stage, be flipped to six positions for
  the accel stage, and be rotated slowly for 20 s for the mag stage. Follow the
  OLED prompts.
* The calibration runs on the main task and blocks command handling while it is
  in progress. Other TCP clients see no replies until it finishes.
* Send `x` on serial during any stage to cancel.
* A successful run is stored in NVS, keyed by robot number, and the robot starts
  using it immediately (no reboot required).
* `cal=1` in `GET` only becomes true once the accelerometer **and** magnetometer
  stages are both stored. A lone `CAL gyro` on a blank robot therefore leaves the
  robot reported as uncalibrated; run `CAL` (all stages) to finish it.

## Error replies

| Reply | Cause |
|---|---|
| `ERR unknown_command,use_...` | the verb is not in the table |
| `ERR line_too_long` | the line exceeded `PROTOCOL_LINE_MAX` (96) bytes |
| `ERR <CMD>_usage,...` | wrong number or type of arguments |
| `ERR SETID_range,1_to_8` | robot number outside the valid range |
| `ERR STREAM_range,0_to_50` | negative or non-numeric rate |
| `ERR CAL_usage,...` | unknown calibration stage |
| `ERR CAL_imu_unavailable` | the MPU9250 did not initialise |
| `ERR ACT_servos_disabled` and friends | firmware built with `ENABLE_SERVOS=0` |
| `ERR busy` | no free TCP client slot (`PROTOCOL_MAX_CLIENTS`) |

## Boot behaviour

Boot does not depend on any of this working:

* An erased or corrupt EEPROM defaults the robot to **1** and prints
  `WARN,ROBOT_NUM_INVALID,...`; it no longer waits for serial input. Use
  `SETID` to fix the number.
* A stored calibration is verified with a short self-check. If it is missing or
  fails, the robot parks the servos, boots **uncalibrated**, shows
  `Uncalibrated - run CAL` on the OLED and prints
  `ROBOT,UNCALIBRATED,send_CAL_to_calibrate`. `GET` reports `cal=0`.
* The legacy single-key IMU menu (`q` then `1`…`6`) is gone; `CAL` replaces it.

## Trying it by hand

```bash
# serial
pio device monitor -b 115200
PING

# TCP from the connected laptop
python3 - <<'EOF'
import socket

s = socket.create_connection(("192.168.4.1", 4242), timeout=5)
stream = s.makefile("r")          # one buffered reader for the whole session

def command(text):
    s.sendall(text.encode() + b"\n")
    while True:
        line = stream.readline().strip()
        if not line or line.startswith(("IMU,", "CAL,")):
            continue              # telemetry / calibration event, not a reply
        return line

for cmd in ("PING", "ID", "GET", "ACT 90 60", "STREAM 2"):
    print(cmd, "->", command(cmd))

s.close()
EOF
```

## Implementation notes

* `lib/CommandServer` owns both parsers, the TCP listener and the stream timer.
  `AHRS`, `ServoControl` and `Network` are only called through their public API,
  so the protocol layer can be replaced without touching the drivers.
* Socket writes use `send(..., MSG_DONTWAIT)`. `WiFiClient::write()` retries for
  up to ~10 s on a stalled peer, which would starve the loop and trip the task
  watchdog.
* The task watchdog (`WATCHDOG_TIMEOUT_S`, default 3 s) covers the Arduino loop
  and the IMU task. `CAL`, the health check and the blocking servo helpers
  suspend or explicitly feed it.
