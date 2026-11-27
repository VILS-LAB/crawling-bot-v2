# Quick start

From a fresh board to your first command in about ten minutes.

## 1. Wire it

| Device | Pins |
|---|---|
| OLED + IMU (I2C) | SDA → GPIO 21, SCL → GPIO 22, VCC → 3V3, GND → GND |
| Servo, down leg | signal → GPIO 7, V+ → external 5 V |
| Servo, up leg | signal → GPIO 6, V+ → external 5 V |

All grounds common. Put the legs in the **park pose (180/180)** before power-on.
Full detail and the pin rationale: [HARDWARE.md](HARDWARE.md).

## 2. Flash it

```bash
git clone https://github.com/VILS-LAB/crawling-bot-v2.git
cd crawling-bot-v2
pio run --target upload
pio device monitor -b 115200
```

Later updates go over the air — no cable:

```bash
pio run --target upload --upload-port ESP32-OTA-1.local
```

See [BUILD.md](BUILD.md) for troubleshooting.

## 3. Join its WiFi

| | |
|---|---|
| SSID | `ESP32-AP-<n>` |
| Password | `12345678` |
| Robot IP | `192.168.4.1` |

The number `<n>` is the robot's identity. A brand-new board defaults to `1`; to
change it later, send `SETID <1..8>` and reboot.

## 4. Say hello

No SDK, no build step — the reference client is plain Python. Install it from
PyPI:

```bash
pip install "crawlingbot[serial]"   # drop [serial] if you only use Wi-Fi
python -c "from crawlingbot import Robot; print(Robot.tcp('192.168.4.1').ping())"
```

Or run the bundled script straight from the repository:

```bash
cd examples/python
python hello_robot.py --host 192.168.4.1
```

```text
connected to tcp://192.168.4.1:4242
uptime      : 51402 ms
robot id    : 1 (ESP32-AP-1)
attitude    : roll +1.24  pitch -0.31  yaw +97.05 deg
calibrated  : True   still: True
legs        : down 180 (target 180), up 180 (target 180)
```

Or by hand, from any terminal:

```text
$ nc 192.168.4.1 4242
PING
OK PONG 51402
GET
OK SNAP,id=1,t=51402,roll=1.24,...,cal=1,d=180,dt=180,u=180,ut=180,act_moving=0
```

## 5. Calibrate

If `calibrated` printed `False`, the robot has parked its servos and will not
move until it has a valid calibration:

```bash
python calibrate.py
```

Follow the OLED prompts: keep still, six orientations, then rotate slowly for
20 s. Details and failure modes: [CALIBRATION.md](CALIBRATION.md).

## What the robot does at boot

```text
Crawling Bot V2 / Starting...
Initializing IMU...
Attaching servos...
Checking IMU           <- verifying the stored calibration (2 s)
Health check...        <- moves the legs, only with a valid calibration
Ready
```

Serial tells the same story, and is the place to look when something is wrong:

```text
AP SSID: ESP32-AP-1
IP Address: 192.168.4.1
OTA Hostname: ESP32-OTA-1
CAL,LOADED,robot=1
CAL,SELF_CHECK,ok
HEALTH,SERVO,down,pin=7,code=0,status=OK
HEALTH,SUMMARY,ok=1,down=0,up=0
PROTOCOL,READY,serial=115200,tcp_port=4242,stream_hz=10
```

With an invalid calibration you instead get:

```text
CAL,SELF_CHECK,failed,run_CAL
ROBOT,UNCALIBRATED,send_CAL_to_calibrate
HEALTH,SERVO,SKIPPED,reason=uncalibrated
```

and the OLED shows `Uncalibrated - run CAL`. That is not a fault: the robot is
protecting the legs until it knows its own orientation.

## Next steps

| Want to… | Go to |
|---|---|
| write your own client | [PROTOCOL.md](PROTOCOL.md), [languages/](languages/README.md) |
| see runnable examples | [EXAMPLES.md](EXAMPLES.md) |
| understand the course phases | [PROJECT.md](PROJECT.md) |
| fix a servo that reports a code | [SERVO_HEALTH.md](SERVO_HEALTH.md) |
| understand the firmware | [ARCHITECTURE.md](ARCHITECTURE.md) |
