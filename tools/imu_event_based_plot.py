#!/usr/bin/env python3
"""
Crawling Bot V2 - EVENT view  (separate from imu_visualizer.py)
================================================================

The visualizer shows the robot against TIME. This tool shows it against EVENTS:
it ignores the time axis and only records a new row when the robot's STATE really
changes.  Sitting untouched for 5 minutes = zero rows.  Then something happens:

    1 event = the robot leaves its settled state ..... (gyro / accel / tilt / arm command)
              ... something happens ................... (peak gyro, peak accel)
              ... it settles again ................... (quiet for settle_s seconds)

and one row is stored: how long it stayed still BEFORE (dwell), how long the event lasted,
the arm commands before -> after, roll/pitch/heading before -> after, the change (delta),
the dead-reckoned move, and the peak gyro / accel.  That is a (state, action, next state)
table, which is exactly what you want for RL later.

    python imu_events.py --demo                         # fake robot, no hardware
    python imu_events.py logs/imu_2026-11-23_15-00-00.csv   # analyse a finished log (--log of the visualizer)
    python imu_events.py --follow                       # live: tails the NEWEST log in logs/
                                                        #   (run `imu_visualizer.py --log` at the same time)
    python imu_events.py --follow logs/some_log.csv     # live: tails one specific log
    python imu_events.py --port auto                    # live: straight from the ESP32 (close the visualizer first,
                                                        #   a serial port can only be opened by one program)
    python imu_events.py FILE --no-plot                 # only print the table and write the CSV

Change any setting without editing the file:

    python imu_events.py --demo --set gyro_dps=10 settle_s=2 keep_att_deg=off

Everything you will want to tweak is in section 1 (Config).  Rows are also saved to
events/events_<name>.csv (turn off with save_csv = False).

ARM ANGLES:  the firmware has to print the commanded servo angles as two extra columns
AFTER the 40 IMU columns (names in Config.arm_fields).  Without them everything still works,
the arm columns are just blank and an arm command alone cannot start an event.
If you record with `imu_visualizer.py --log` and want the arm angles inside that log, add
"down_cmd", "up_cmd" at the end of DEFAULT_FIELDS in imu_visualizer.py as well.
"""
from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

NAN = float("nan")


# =============================================================================
# 1. CONFIG  -  the knobs.  (any of them can also be set with  --set name=value)
# =============================================================================
@dataclass
class Config:
    # ---- connection / files (only used by --port / --follow) ----------------
    port: str = "auto"
    baud: int = 115200
    line_prefix: str = "IMU"
    log_dir: str = "logs"                # --follow (no name) tails the newest imu_*.csv in this folder
    # Firmware columns with the COMMANDED servo angles in degrees: (down leg, up leg).
    arm_fields: tuple = ("down_cmd", "up_cmd")

    # ---- 1. what counts as "something is happening" -------------------------
    gyro_dps: float = 6.0        # |filtered gyro| above this = the robot is rotating
    acc_ms2: float = 0.6         # |filtered linear acceleration| above this = it is accelerating
    tilt_deg: float = 2.0        # roll/pitch moved this far from the last settled state. Catches slow
                                 #   tilts that stay under gyro_dps.  Keep it >= keep_att_deg.
                                 #   (yaw is not used here on purpose: a gyro bias makes yaw creep)
    arm_change_deg: float = 0.05 # any change of a commanded angle bigger than this = an action
    arm_hold_s: float = 1.5      # after an arm command the event stays open at least this long
    use_firmware_moving: bool = False   # True = use the firmware's `moving` flag instead of gyro/acc

    # ---- 2. when is the new state "settled" ---------------------------------
    settle_s: float = 1.0        # quiet for this long = the event is over
    snapshot_s: float = 0.5      # the stored "state" is the average of the last snapshot_s seconds

    # ---- 3. which events are worth a row (set a value to None / "--set x=off" to disable it)
    #      An event is kept if ANY of these is true.  Everything else is treated as a blip.
    keep_att_deg: float | None = 1.5          # roll or pitch changed at least this much
    keep_heading_deg: float | None = 3.0      # heading changed at least this much
    keep_peak_gyro_dps: float | None = 40.0   # it was shaken / kicked hard even if it ended up where it was
    keep_arm_deg: float | None = 1.0          # an arm command changed by at least this much

    # ---- frame (same meaning as in imu_visualizer.py) -----------------------
    heading_offset_deg: float = 270.0    # heading = (heading_offset_deg - yaw) mod 360
    yaw_sign: float = 1.0                # -1 if the heading turns the wrong way

    # ---- display / output ---------------------------------------------------
    table_rows: int = 14                 # rows shown in the table (newest first)
    plot_last_n: int = 60                # events drawn in the four plots
    refresh_s: float = 0.4               # screen refresh
    save_csv: bool = True
    out_dir: str = "events"
    demo_speed: float = 1.0              # --demo only: 2.0 = twice as fast
    figsize: tuple = (16, 9)

    def __post_init__(self):
        self.snapshot_s = min(self.snapshot_s, self.settle_s)


# =============================================================================
# 2. DATA FORMAT  (same 40 IMU columns as imu_visualizer.py)
# =============================================================================
DEFAULT_FIELDS = (
    "t_ms", "dt",
    "ax", "ay", "az",
    "lax", "lay", "laz",
    "flax", "flay", "flaz",
    "gx", "gy", "gz",
    "gbx", "gby", "gbz",
    "fgx", "fgy", "fgz",
    "magx", "magy", "magz",
    "roll", "pitch", "yaw",
    "vx", "vy", "vz",
    "px", "py", "pz",
    "temp_c", "moving", "still",
    "still_failure_mask",
    "stdA", "trust",
    "rollG", "pitchG",
    "ebx", "eby", "ebz",
    "acc_norm_g",
    "mag_norm",
    "rate_hz",
    "accel_rms_g",
    "mag_rms_ut",
    "mag_fit",
    "est_init",
)
# the only columns this tool needs (everything else is skipped while parsing = fast)
NEEDED = ("t_ms", "roll", "pitch", "yaw", "fgx", "fgy", "fgz", "flax", "flay", "flaz",
          "px", "py", "pz", "temp_c", "moving")


def wrap180(x):
    return (x + 180.0) % 360.0 - 180.0


def norm3(s, a, b, c):
    x, y, z = s.get(a, NAN), s.get(b, NAN), s.get(c, NAN)
    if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
        return math.sqrt(x * x + y * y + z * z)
    return NAN


def big(x, thr):
    return thr is not None and math.isfinite(x) and abs(x) >= thr


class LineParser:
    """text line -> dict {name: float, 't': seconds}.   Understands the live firmware lines
    ('IMU,<t_ms>,...') and the CSV logs of the visualizer ('#t_ms,...' header + 'IMU,...' rows)."""

    def __init__(self, cfg):
        self.prefix = cfg.line_prefix
        self.wanted = set(NEEDED) | set(cfg.arm_fields)
        fields = list(DEFAULT_FIELDS) + [n for n in cfg.arm_fields if n not in DEFAULT_FIELDS]
        self.rejected, self.reason, self.last_reject = 0, None, ""
        self._set_fields(fields, header=False)

    def _set_fields(self, names, header):
        self.fields = [n.strip() for n in names]
        self.idx = [(i, n) for i, n in enumerate(self.fields) if n in self.wanted]
        # without a header we only insist on the 40 IMU columns (arm columns are optional)
        self.min_cols = len(self.fields) if header else len(DEFAULT_FIELDS)

    def feed(self, line, host_t):
        self.reason = None
        line = line.strip()
        if not line:
            return None
        if line[0] != "#" and self.prefix:
            if not line.startswith(self.prefix + ","):
                return self._reject(f"no '{self.prefix},' prefix", line)
            line = line[len(self.prefix) + 1:]
            if not line:
                return None
        if line[0] == "#" or line.startswith("t_ms"):             # header line
            self._set_fields(line.lstrip("#").split(","), header=True)
            return None
        parts = line.split(",")
        if len(parts) < self.min_cols:
            return self._reject(f"got {len(parts)} values, expected {self.min_cols}", line)
        s = {}
        try:
            for i, n in self.idx:
                if i < len(parts):
                    s[n] = float(parts[i])
        except ValueError:
            return self._reject("a value is not a number", line)
        t_ms = s.pop("t_ms", None)
        s["t"] = host_t if t_ms is None else t_ms * 1e-3
        return s

    def _reject(self, why, line):
        self.reason = why
        self.rejected += 1
        self.last_reject = f"{why}:  {line[:50]}"
        return None


# =============================================================================
# 3. THE EVENT DETECTOR  (pure logic, no GUI)
# =============================================================================
SNAP_BASE = ("roll", "pitch", "yaw", "px", "py", "pz", "temp_c")
ANGLES = ("roll", "pitch", "yaw")
TRIGGER_ORDER = ("tilt", "arm", "gyro", "acc", "fw-moving")

EVENT_COLUMNS = ("n", "t_start", "t_end", "dur", "dwell", "trigger",
                 "down0", "up0", "down1", "up1",
                 "roll0", "pitch0", "heading0", "E0", "N0",
                 "roll1", "pitch1", "heading1", "E1", "N1",
                 "d_roll", "d_pitch", "d_heading", "dE", "dN", "dist",
                 "peak_gyro", "peak_acc")


class EventDetector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.arm = tuple(cfg.arm_fields[:2])                     # (down leg, up leg)
        self.fields = SNAP_BASE + self.arm
        self.i_roll, self.i_pitch = self.fields.index("roll"), self.fields.index("pitch")
        self.n = 0                                               # number of events so far
        self.arm_seen = False
        self.reset()

    def reset(self):
        self.hist = deque()                    # (t, values) of the last snapshot_s seconds
        self.ref, self.ref_t = None, None      # last settled state and when it began
        self.origin = None                     # (px, py) of the very first settled state
        self.seg = None                        # the event that is in progress
        self.quiet_t = None                    # since when has the current event been quiet
        self.t0, self.t_last = None, None
        self.arm_t = -math.inf                 # time of the last arm command change
        self.prev_arm = [NAN] * len(self.arm)

    # ------------------------------------------------------------------ input
    def feed(self, s):
        """Feed one sample (dict). Returns the list of events that finished with this sample."""
        t = s["t"]
        if not math.isfinite(t):
            return []
        cfg, out = self.cfg, []
        if self.t_last is not None and t < self.t_last - 1.0:    # ESP32 rebooted / another log started
            out += self.finalize()
            self.reset()
        self.t_last = t
        if self.t0 is None:
            self.t0 = t
        vec = tuple(s.get(k, NAN) for k in self.fields)
        self.hist.append((t, vec))
        while t - self.hist[0][0] > cfg.snapshot_s:
            self.hist.popleft()

        # --- is something happening right now? ---
        for i, name in enumerate(self.arm):
            v = s.get(name, NAN)
            if math.isfinite(v):
                self.arm_seen = True
                p = self.prev_arm[i]
                if math.isfinite(p) and abs(v - p) >= cfg.arm_change_deg:
                    self.arm_t = t
                self.prev_arm[i] = v
        why = []
        gyro, acc = norm3(s, "fgx", "fgy", "fgz"), norm3(s, "flax", "flay", "flaz")
        if cfg.use_firmware_moving:
            if s.get("moving", NAN) >= 0.5:
                why.append("fw-moving")
        else:
            if gyro > cfg.gyro_dps:
                why.append("gyro")
            if acc > cfg.acc_ms2:
                why.append("acc")
        if t - self.arm_t < cfg.arm_hold_s:
            why.append("arm")

        # --- start of an event ---
        if self.seg is None:
            if self.ref is None:                                 # first settled state after start-up
                if not why and t - self.hist[0][0] >= 0.9 * cfg.snapshot_s:
                    self._set_ref(self._snap(), self.t0)
            else:
                w = list(why)
                if self._tilt(vec) > cfg.tilt_deg:
                    w.append("tilt")
                if w:
                    self.seg = {"t": t, "why": set(w), "pg": 0.0, "pa": 0.0}
                    self.quiet_t = None

        # --- during / end of an event ---
        if self.seg is not None:
            seg = self.seg
            seg["why"].update(why)
            if math.isfinite(gyro) and gyro > seg["pg"]:
                seg["pg"] = gyro
            if math.isfinite(acc) and acc > seg["pa"]:
                seg["pa"] = acc
            if why:
                self.quiet_t = None
            else:
                if self.quiet_t is None:
                    self.quiet_t = t
                if t - self.quiet_t >= cfg.settle_s:
                    ev = self._close(self.quiet_t)
                    if ev:
                        out.append(ev)
        return out

    def finalize(self):
        """End of the data: close the event that is still open."""
        if self.seg is None or self.t_last is None:
            return []
        ev = self._close(self.t_last)
        return [ev] if ev else []

    def status(self):
        """(kind, seconds, detail) for the status line."""
        if self.t_last is None:
            return "WAITING FOR DATA", 0.0, ""
        if self.seg is not None:
            why = "+".join(k for k in TRIGGER_ORDER if k in self.seg["why"])
            return "MOVING", self.t_last - self.seg["t"], why
        if self.ref is None:
            return "WARMING UP", 0.0, ""
        return "STILL", self.t_last - self.ref_t, ""

    # --------------------------------------------------------------- internals
    def _set_ref(self, st, t):
        self.ref, self.ref_t = st, t
        if self.origin is None and math.isfinite(st["px"]) and math.isfinite(st["py"]):
            self.origin = (st["px"], st["py"])

    def _snap(self):
        """The settled state = mean over the last snapshot_s seconds (angles averaged around
        their last value so 359/1 deg does not become 180; arm commands = last value)."""
        arr = np.array([v for _, v in self.hist], float)
        out = {}
        for j, name in enumerate(self.fields):
            col = arr[:, j]
            col = col[np.isfinite(col)]
            if col.size == 0:
                out[name] = NAN
            elif name in self.arm:
                out[name] = float(col[-1])
            elif name in ANGLES:
                out[name] = float(col[-1] + np.mean(wrap180(col - col[-1])))
            else:
                out[name] = float(np.mean(col))
        return out

    def _tilt(self, vec):
        d = 0.0
        for i, name in ((self.i_roll, "roll"), (self.i_pitch, "pitch")):
            x, y = vec[i], self.ref[name]
            if math.isfinite(x) and math.isfinite(y):
                d = max(d, abs(wrap180(x - y)))
        return d

    def _heading(self, yaw):
        if not math.isfinite(yaw):
            return NAN
        return (self.cfg.heading_offset_deg - self.cfg.yaw_sign * yaw) % 360.0

    def _en(self, st):
        """(east, north) in metres relative to the first settled position. world X = north, Y = west."""
        if self.origin is None or not math.isfinite(st["px"]) or not math.isfinite(st["py"]):
            return NAN, NAN
        return -(st["py"] - self.origin[1]), st["px"] - self.origin[0]

    def _arm(self, st, i):
        return st.get(self.arm[i], NAN) if i < len(self.arm) else NAN

    def _close(self, end_t):
        seg, st0, st1 = self.seg, self.ref, self._snap()
        self.seg, self.quiet_t = None, None
        ev = self._make(seg, st0, st1, end_t)
        if self._worth_keeping(ev):
            self.n += 1
            ev["n"] = self.n
            self._set_ref(st1, end_t)
            return ev
        if "tilt" in seg["why"]:          # a discarded tilt must not re-trigger forever
            self.ref = st1
        return None

    def _make(self, seg, st0, st1, end_t):
        h0, h1 = self._heading(st0["yaw"]), self._heading(st1["yaw"])
        e0, n0 = self._en(st0)
        e1, n1 = self._en(st1)
        return {
            "n": 0, "t_start": seg["t"] - self.t0, "t_end": end_t - self.t0,
            "dur": end_t - seg["t"], "dwell": seg["t"] - self.ref_t,
            "trigger": "+".join(k for k in TRIGGER_ORDER if k in seg["why"]),
            "down0": self._arm(st0, 0), "up0": self._arm(st0, 1),
            "down1": self._arm(st1, 0), "up1": self._arm(st1, 1),
            "roll0": st0["roll"], "pitch0": st0["pitch"], "heading0": h0, "E0": e0, "N0": n0,
            "roll1": st1["roll"], "pitch1": st1["pitch"], "heading1": h1, "E1": e1, "N1": n1,
            "d_roll": wrap180(st1["roll"] - st0["roll"]), "d_pitch": wrap180(st1["pitch"] - st0["pitch"]),
            "d_heading": wrap180(h1 - h0), "dE": e1 - e0, "dN": n1 - n0, "dist": math.hypot(e1 - e0, n1 - n0),
            "peak_gyro": seg["pg"], "peak_acc": seg["pa"],
        }

    def _worth_keeping(self, ev):
        c = self.cfg
        return (big(ev["d_roll"], c.keep_att_deg) or big(ev["d_pitch"], c.keep_att_deg)
                or big(ev["d_heading"], c.keep_heading_deg)
                or big(ev["peak_gyro"], c.keep_peak_gyro_dps)
                or big(ev["down1"] - ev["down0"], c.keep_arm_deg)
                or big(ev["up1"] - ev["up0"], c.keep_arm_deg))


# =============================================================================
# 4. DATA SOURCES  (background threads that push parsed samples into a queue)
# =============================================================================
class Reader(threading.Thread):
    def __init__(self, q, cfg):
        super().__init__(daemon=True)
        self.q, self.cfg = q, cfg
        self.parser = LineParser(cfg)
        self.stop_evt = threading.Event()
        self.state = "starting"

    def stop(self):
        self.stop_evt.set()

    def handle_line(self, line):
        s = self.parser.feed(line, time.monotonic())
        if s is not None:
            self.q.put(s)


def find_port():
    from serial.tools import list_ports
    hints = ("usbmodem", "usbserial", "ttyacm", "ttyusb", "slab", "ch340", "cp210", "com")
    for p in list_ports.comports():
        if any(h in (p.device + " " + (p.description or "")).lower() for h in hints):
            return p.device
    return None


class SerialReader(Reader):
    def run(self):
        try:
            import serial
        except ImportError:
            self.state = "pyserial missing: pip install pyserial"
            return
        while not self.stop_evt.is_set():
            port = self.cfg.port if self.cfg.port != "auto" else find_port()
            if not port:
                self.state = "no serial port found"
                time.sleep(1.0)
                continue
            try:
                with serial.Serial(port, self.cfg.baud, timeout=0.05) as ser:
                    self.state = f"serial {port} @ {self.cfg.baud}"
                    buf = b""
                    while not self.stop_evt.is_set():
                        chunk = ser.read(max(1, ser.in_waiting))
                        if not chunk:
                            continue
                        buf += chunk
                        *lines, buf = buf.split(b"\n")
                        for ln in lines:
                            self.handle_line(ln.decode("ascii", "ignore"))
                        if len(buf) > 65536:
                            buf = b""
            except (serial.SerialException, OSError) as e:
                self.state = f"serial error: {str(e)[:40]}"
                time.sleep(1.0)


class FollowReader(Reader):
    """Like `tail -f` on a CSV log that imu_visualizer.py --log is writing right now."""

    def __init__(self, q, cfg, spec):
        super().__init__(q, cfg)
        self.spec = spec                                 # "latest" or a path

    def _pick_log(self):
        if self.spec != "latest":
            return Path(self.spec)
        files = sorted(Path(self.cfg.log_dir).glob("imu_*.csv"), key=lambda p: p.stat().st_mtime)
        return files[-1] if files else None

    def run(self):
        cur = f = None
        pending = ""
        while not self.stop_evt.is_set():
            line = f.readline() if f else ""
            if line:
                pending += line
                if pending.endswith("\n"):                # the writer may still be in the middle of a line
                    self.handle_line(pending)
                    pending = ""
                continue
            target = self._pick_log()                       # idle: look for a new / newer log
            if target is not None and target != cur and target.exists():
                if f:
                    f.close()
                cur, f, pending = target, open(target, errors="ignore"), ""
                self.parser = LineParser(self.cfg)        # new file -> new header
                self.state = f"following {cur.name}"
                continue
            if cur is None:
                self.state = f"waiting for a log in {self.cfg.log_dir}/ ..."
            self.stop_evt.wait(0.1)
        if f:
            f.close()


# Fake robot: a 40 s script that repeats. (start s, duration s, d_roll, d_pitch, d_yaw in deg,
# down cmd, up cmd, travel in m).  None = arm command unchanged.
DEMO_SCRIPT = (
    (5.0, 1.5, 0, 0, 0, 60, 120, 0.00),       # arm command only: the robot does not react
    (9.0, 2.0, 4, -9, 15, None, None, 0.12),  # it tilts, turns and crawls forward
    (16.0, 1.0, 0, 0, 40, 120, 60, 0.05),     # arm command + a quick turn
    (24.0, 2.5, -4, 9, -55, None, None, 0.10),
    (33.0, 1.5, 0, 0, 0, 90, 90, 0.00),       # arms to the 90/90 test pose
)
DEMO_CYCLE = 40.0


def demo_stream(rate=100.0):
    rng = np.random.default_rng(3)
    dt = 1.0 / rate
    roll = pitch = yaw = px = py = 0.0
    down = up = 90.0
    n = 0
    while True:
        t = n * dt
        n += 1
        ph = t % DEMO_CYCLE
        g, v, acc, moving = [0.0, 0.0, 0.0], 0.0, 0.0, 0.0
        for ts, dur, dr, dp, dy, ad, au, dist in DEMO_SCRIPT:
            if ad is not None and ts <= ph < ts + 1.5 * dt:
                down, up = float(ad), float(au)
            if ts <= ph < ts + dur:
                u = (ph - ts) / dur
                sm = 6.0 * u * (1.0 - u) / dur                    # smooth 0 -> 1 ramp, rate profile
                g[0] += dr * sm
                g[1] += dp * sm
                g[2] += dy * sm
                v += dist * sm
                acc += dist * 6.0 * (1.0 - 2.0 * u) / (dur * dur)
                moving = 1.0
        roll += g[0] * dt
        pitch += g[1] * dt
        yaw += g[2] * dt
        h = math.radians((270.0 - yaw) % 360.0)                   # same heading rule as the visualizer
        px += math.cos(h) * v * dt
        py -= math.sin(h) * v * dt
        z = rng.normal(0.0, 1.0, 6)
        yield {"t": t, "roll": roll, "pitch": pitch, "yaw": yaw,
               "fgx": g[0] + 0.3 * z[0], "fgy": g[1] + 0.3 * z[1], "fgz": g[2] + 0.3 * z[2],
               "flax": 0.05 * z[3], "flay": acc + 0.05 * z[4], "flaz": 0.05 * z[5],
               "px": px, "py": py, "pz": 0.0, "temp_c": 30.0, "moving": moving,
               "down_cmd": down, "up_cmd": up}


class DemoReader(Reader):
    def run(self):
        self.state = "demo data"
        t0 = time.monotonic()
        for s in demo_stream():
            delay = s["t"] / self.cfg.demo_speed - (time.monotonic() - t0)
            if delay > 0 and self.stop_evt.wait(delay):
                return
            if self.stop_evt.is_set():
                return
            self.q.put(s)


def analyse_file(path, cfg):
    """Run a finished log through the detector in one go."""
    parser, det = LineParser(cfg), EventDetector(cfg)
    events, samples = [], 0
    for ln in Path(path).read_text(errors="ignore").splitlines():
        s = parser.feed(ln, samples * 0.01)
        if s is not None:
            samples += 1
            events += det.feed(s)
    events += det.finalize()
    return det, events, samples, parser


class EventWriter:
    def __init__(self, folder, name):
        Path(folder).mkdir(parents=True, exist_ok=True)
        self.path = Path(folder) / name
        self.f = open(self.path, "w", newline="")
        self.f.write(",".join(EVENT_COLUMNS) + "\n")
        self.f.flush()

    def write(self, ev):
        cells = [v if isinstance(v, str) else (str(v) if isinstance(v, int) else f"{v:.4f}")
                 for v in (ev[c] for c in EVENT_COLUMNS)]
        self.f.write(",".join(cells) + "\n")
        self.f.flush()

    def close(self):
        self.f.close()


# =============================================================================
# 5. TEXT TABLE
# =============================================================================
def fmt_t(sec):
    if sec is None or not math.isfinite(sec):
        return "-"
    if sec < 60:
        return f"{sec:.1f}s"
    m, s = divmod(int(round(sec)), 60)
    if m < 60:
        return f"{m}m{s:02d}s"
    h, m = divmod(m, 60)
    return f"{h}h{m:02d}m"


def fmt_arm(a, b):
    f = lambda x: "--" if not math.isfinite(x) else f"{x:.0f}"
    return f"{f(a):>3}→{f(b):<3}"


TABLE_HEADER = (f"{'#':>4} {'start':>7} {'dwell':>7} {'dur':>7} {'trigger':<14} {'down °':>7} {'up °':>7} "
                f"{'Δroll':>6} {'Δpitch':>6} {'Δhead':>7} {'ΔE m':>6} {'ΔN m':>6} {'dist':>5} {'gyro':>5} {'acc':>5}")


def row_text(e):
    return (f"{e['n']:>4} {fmt_t(e['t_start']):>7} {fmt_t(e['dwell']):>7} {fmt_t(e['dur']):>7} "
            f"{e['trigger']:<14.14} {fmt_arm(e['down0'], e['down1']):>7} {fmt_arm(e['up0'], e['up1']):>7} "
            f"{e['d_roll']:+6.1f} {e['d_pitch']:+6.1f} {e['d_heading']:+7.1f} {e['dE']:+6.2f} {e['dN']:+6.2f} "
            f"{e['dist']:5.2f} {e['peak_gyro']:5.0f} {e['peak_acc']:5.1f}")


# =============================================================================
# 6. THE WINDOW
# =============================================================================
X_C, Y_C, Z_C = "#d62728", "#2ca02c", "#1f77b4"


class App:
    def __init__(self, cfg, det, events, reader=None, writer=None, mode="LIVE", label="", samples=0):
        self.cfg, self.det, self.events = cfg, det, events
        self.reader, self.writer, self.mode, self.label = reader, writer, mode, label
        self.q = reader.q if reader else None
        self.rx_total, self.rx_rate, self._rx_mark = samples, 0.0, (time.monotonic(), samples)
        self.last_rx = None
        self.dirty = True
        self._build_figure()

    def _build_figure(self):
        cfg = self.cfg
        plt.rcParams.update({"font.size": 8, "axes.titlesize": 9.5, "axes.titleweight": "bold"})
        self.fig = fig = plt.figure(figsize=cfg.figsize)
        try:
            fig.canvas.manager.set_window_title("Crawling Bot V2 - event view")
        except Exception:
            pass
        gs = fig.add_gridspec(2, 4, left=0.05, right=0.985, top=0.955, bottom=0.085, hspace=0.30,
                              wspace=0.28, height_ratios=[0.78, 1.0])
        axt = fig.add_subplot(gs[0, :])
        axt.axis("off")
        self.t_status = axt.text(0.0, 1.0, "", transform=axt.transAxes, va="top", fontsize=11,
                                 fontweight="bold", family="monospace")
        self.t_table = axt.text(0.0, 0.88, "", transform=axt.transAxes, va="top", fontsize=8.5,
                                family="monospace", linespacing=1.3)
        self.ax_att = fig.add_subplot(gs[1, 0])
        self.ax_chg = fig.add_subplot(gs[1, 1])
        self.ax_time = fig.add_subplot(gs[1, 2])
        self.ax_pos = fig.add_subplot(gs[1, 3])
        fig.text(0.05, 0.012,
                 f"event = leaves its settled state (gyro > {cfg.gyro_dps:g} °/s | accel > {cfg.acc_ms2:g} m/s² | "
                 f"tilt > {cfg.tilt_deg:g}° | arm command)  and ends after {cfg.settle_s:g} s of quiet.   "
                 "Δ = after − before.   E/N = dead reckoning (drifts, treat as rough).",
                 fontsize=8, color="0.4")
        fig.canvas.mpl_connect("close_event", lambda _e: self.shutdown())

    # -------------------------------------------------------------- drawing
    def _setup(self, ax, title, xlabel, ylabel):
        ax.cla()
        ax.set_title(title, loc="left")
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)

    @staticmethod
    def _ticks(ax, ns):
        step = max(1, math.ceil(len(ns) / 25))
        ax.set_xticks(np.arange(0, len(ns), step))
        ax.set_xticklabels([str(n) for n in ns[::step]], fontsize=7)

    def _path_plot(self, ax, xs, ys, ids):
        ax.plot(xs, ys, "-", color="0.6", lw=1)
        ax.scatter(xs, ys, c=np.arange(len(xs)), cmap="viridis", s=36, zorder=3)
        ax.scatter(xs[-1:], ys[-1:], s=120, facecolors="none", edgecolors="#d62728", lw=2, zorder=4)
        if len(xs) <= 40:
            for x, y, i in zip(xs, ys, ids):
                ax.annotate(str(i), (x, y), textcoords="offset points", xytext=(4, 4), fontsize=7)

    def _draw_plots(self):
        P = self.events[-self.cfg.plot_last_n:]
        s1 = (self.ax_att, "Attitude after each event", "roll (°)", "pitch (°)")
        s2 = (self.ax_chg, "Change in each event (°)", "event #", "")
        s3 = (self.ax_time, "Time before (grey) and during (orange) each event", "event #", "seconds (log)")
        s4 = (self.ax_pos, "Position after each event (rough)", "E (m)", "N (m)")
        for a in (s1, s2, s3, s4):
            self._setup(*a)
        if not P:
            for ax in (self.ax_att, self.ax_chg, self.ax_time, self.ax_pos):
                ax.text(0.5, 0.5, "no events yet", transform=ax.transAxes, ha="center", color="0.5")
            return
        ns = [e["n"] for e in P]
        xs = np.arange(len(P))

        ids = [P[0]["n"] - 1] + ns                                   # point 0 = the state before the first plotted event
        self._path_plot(self.ax_att, [P[0]["roll0"]] + [e["roll1"] for e in P],
                        [P[0]["pitch0"]] + [e["pitch1"] for e in P], ids)

        w = 0.27
        for k, (key, color, lab) in enumerate((("d_roll", X_C, "Δroll"), ("d_pitch", Y_C, "Δpitch"),
                                               ("d_heading", Z_C, "Δheading"))):
            self.ax_chg.bar(xs + (k - 1) * w, np.nan_to_num([e[key] for e in P]), w, color=color, label=lab)
        self.ax_chg.axhline(0, color="k", lw=0.6)
        self.ax_chg.legend(loc="upper left", fontsize=7, ncol=3, framealpha=0.75)
        self._ticks(self.ax_chg, ns)

        floor = 0.01
        dwell = np.maximum(np.nan_to_num([e["dwell"] for e in P]), floor)
        dur = np.maximum(np.nan_to_num([e["dur"] for e in P]), floor)
        self.ax_time.bar(xs - 0.2, dwell, 0.4, color="0.65", label="still before")
        self.ax_time.bar(xs + 0.2, dur, 0.4, color="#e08a00", label="event lasted")
        self.ax_time.set_yscale("log")
        self.ax_time.set_ylim(floor, max(dwell.max(), dur.max()) * 4)
        self.ax_time.legend(loc="upper left", fontsize=7, framealpha=0.75)
        self._ticks(self.ax_time, ns)

        self._path_plot(self.ax_pos, [P[0]["E0"]] + [e["E1"] for e in P],
                        [P[0]["N0"]] + [e["N1"] for e in P], ids)
        self.ax_pos.set_aspect("equal", adjustable="datalim")

    def _update_table(self):
        rows = self.events[-self.cfg.table_rows:][::-1]
        if rows:
            text = TABLE_HEADER + "\n" + "\n".join(row_text(e) for e in rows)
        else:
            text = "no events yet - waiting for the robot to do something ..."
            p = self.reader.parser if self.reader else None
            if p and p.rejected:
                text += f"\n\n{p.rejected} lines received but REJECTED, last one:\n{p.last_reject}"
        self.t_table.set_text(text)

    def _update_status(self):
        now = time.monotonic()
        kind, secs, detail = self.det.status()
        if kind == "STILL":
            sit = f"STILL for {fmt_t(secs)}"
        elif kind == "MOVING":
            sit = f"MOVING {fmt_t(secs)}  [{detail}]"
        else:
            sit = kind
        if self.mode == "FILE":
            head, color = f"● FILE {self.label}   {self.rx_total} samples", "#1f77b4"
        else:
            live = self.last_rx is not None and now - self.last_rx < 1.0
            state = self.reader.state if self.reader else ""
            head = f"● {self.mode if live else 'NO DATA'} {state[:34]}   rx {self.rx_rate:3.0f} Hz"
            color = "#2ca02c" if live else "#d62728"
            sit = sit if live else f"last state: {sit}"
        extra = "" if self.det.arm_seen else "   (no arm columns in the data)"
        self.t_status.set_text(f"{head}   |   {sit}   |   {len(self.events)} events{extra}")
        self.t_status.set_color(color)

    # ----------------------------------------------------------- main loop
    def tick(self):
        if self.q is not None:
            deadline = time.monotonic() + 0.15                      # never block the window for long
            while time.monotonic() < deadline:
                try:
                    s = self.q.get_nowait()
                except queue.Empty:
                    break
                self.rx_total += 1
                self.last_rx = time.monotonic()
                for ev in self.det.feed(s):
                    self._add(ev)
            now = time.monotonic()
            t0, n0 = self._rx_mark
            if now - t0 > 1.0:
                self.rx_rate = (self.rx_total - n0) / (now - t0)
                self._rx_mark = (now, self.rx_total)
        if self.dirty:
            self.dirty = False
            self._draw_plots()
            self._update_table()
        self._update_status()
        self.fig.canvas.draw_idle()

    def _tick_safe(self):
        try:
            self.tick()
        except Exception:
            import traceback
            traceback.print_exc()

    def _add(self, ev):
        self.events.append(ev)
        self.dirty = True
        print(row_text(ev))
        if self.writer:
            self.writer.write(ev)

    def run(self):
        if self.reader is None:                                     # finished log: draw once
            self.tick()
        else:
            self.timer = self.fig.canvas.new_timer(interval=int(self.cfg.refresh_s * 1000))
            self.timer.add_callback(self._tick_safe)
            self.timer.start()
        plt.show()

    def shutdown(self):
        if getattr(self, "_closed", False):
            return
        self._closed = True
        if self.reader:
            self.reader.stop()
            for ev in self.det.finalize():
                self._add(ev)
        if self.writer:
            self.writer.close()
            print(f"\nevents saved: {self.writer.path}")


# =============================================================================
# 7. MAIN
# =============================================================================
def apply_overrides(cfg, items):
    for item in items or []:
        key, _, val = item.partition("=")
        if not hasattr(cfg, key):
            sys.exit(f"unknown setting '{key}'  (see section 1 of the file)")
        cur = getattr(cfg, key)
        if val.lower() in ("none", "off"):
            new = None
        elif isinstance(cur, bool):
            new = val.lower() in ("1", "true", "yes", "on")
        elif cur is None or isinstance(cur, float):
            new = float(val)
        elif isinstance(cur, int):
            new = int(val)
        elif isinstance(cur, tuple):
            new = tuple(x.strip() for x in val.split(","))
        else:
            new = val
        setattr(cfg, key, new)
    cfg.__post_init__()


def main():
    ap = argparse.ArgumentParser(description="Crawling Bot V2 - event view (separate from imu_visualizer.py)")
    ap.add_argument("file", nargs="?", help="a finished CSV log to analyse (written by imu_visualizer.py --log)")
    ap.add_argument("--follow", nargs="?", const="latest", metavar="CSV",
                    help="live: tail a log that imu_visualizer.py --log is writing (default: newest in logs/)")
    ap.add_argument("--port", nargs="?", const="auto", metavar="PORT", help="live: read the ESP32 directly")
    ap.add_argument("--baud", type=int)
    ap.add_argument("--demo", action="store_true", help="fake data, no robot needed")
    ap.add_argument("--no-plot", action="store_true", help="only print the table / write the CSV")
    ap.add_argument("--set", nargs="+", metavar="NAME=VALUE", help="override Config values, e.g. gyro_dps=10")
    a = ap.parse_args()

    cfg = Config()
    if a.port:
        cfg.port = a.port
    if a.baud:
        cfg.baud = a.baud
    apply_overrides(cfg, a.set)

    def new_writer(name):
        return EventWriter(cfg.out_dir, name) if cfg.save_csv else None

    print(f"thresholds: gyro > {cfg.gyro_dps:g} °/s | accel > {cfg.acc_ms2:g} m/s² | tilt > {cfg.tilt_deg:g}° | "
          f"arm cmd | settle {cfg.settle_s:g} s")
    print(TABLE_HEADER)

    # ---- a finished log: analyse it in one go ----
    if a.file and not (a.demo or a.follow or a.port):
        path = Path(a.file)
        if not path.exists():
            sys.exit(f"file not found: {path}")
        det, events, samples, parser = analyse_file(path, cfg)
        writer = new_writer(f"events_{path.stem}.csv")
        for ev in events:
            print(row_text(ev))
            if writer:
                writer.write(ev)
        print(f"\n{samples} samples, {len(events)} events"
              + (f", {parser.rejected} lines rejected (last: {parser.last_reject})" if parser.rejected else "")
              + ("" if det.arm_seen else "   [no arm columns in this log]"))
        if writer:
            writer.close()
            print(f"events saved: {writer.path}")
        if not a.no_plot:
            App(cfg, det, events, None, None, "FILE", path.name, samples).run()
        return

    # ---- live sources ----
    q = queue.SimpleQueue()
    if a.demo:
        reader, mode = DemoReader(q, cfg), "DEMO"
    elif a.follow:
        reader, mode = FollowReader(q, cfg, a.follow), "FOLLOW"
    elif a.port:
        reader, mode = SerialReader(q, cfg), "LIVE"
    else:
        ap.print_help()
        return
    if a.no_plot:
        sys.exit("--no-plot only makes sense together with a log file")
    det = EventDetector(cfg)
    writer = new_writer(f"events_{datetime.now():%Y-%m-%d_%H-%M-%S}.csv")
    app = App(cfg, det, [], reader, writer, mode)
    reader.start()
    app.run()


if __name__ == "__main__":
    sys.exit(main())