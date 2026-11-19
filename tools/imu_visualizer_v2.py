#!/usr/bin/env python3
"""
Crawling Bot V2 - live IMU visualizer  (matplotlib)
===================================================

    python imu_visualizer.py --demo                      # fake robot, no hardware needed
    python imu_visualizer.py --port auto                 # find the ESP32 automatically
    python imu_visualizer.py --port /dev/cu.usbmodem1101 --baud 115200 --log
    python imu_visualizer.py /dev/cu.usbmodem1101        # port as first argument also works
    python imu_visualizer.py --replay logs/imu_2026-11-07_21-00-00.csv

Keys (click the window first):
    space  pause / resume the display (data keeps being recorded)
    [  ]   shorter / longer time window  (5 10 20 30 60 120 s, see Config.window_steps)
    m      cycle Y-range preset: overview -> detail -> auto
    z      zero the top-down position (ΔX/ΔY start again from 0)

Robot frame: X = right, Y = front, Z = up (right-handed). Heading dial: 0 deg = world +Y
(the way the robot faced when yaw was 0), increasing clockwise as seen from above.
Test: turn the robot LEFT by hand -> the heading must DEcrease and the icon must turn left.
If it turns the wrong way, set Config.euler_signs = (1, 1, -1).

Everything you will want to tweak is in section 1 (Config) and section 3 (PANELS).

------------------------------------------------------------------------------
WHAT THE ESP32 PRINTS  (one line per sample, newline terminated)
------------------------------------------------------------------------------
    IMU,<t_ms>,<39 more numbers>        <- 'IMU' is Config.line_prefix; other lines are ignored

The column names/order are DEFAULT_FIELDS below (section 2) = the FIELDS list of your
first visualizer. If your firmware changes, edit that one list - or print a header line
once at boot (and e.g. every 5 s):

    IMU,t_ms,dt,ax,ay,az,...            (or the same without 'IMU,', or starting with '#')

Columns the visualizer does not know are ignored; columns you do not send show as NaN.
If lines arrive but are rejected, the status panel says why (wrong prefix / wrong
number of columns / not a number) instead of staying silently empty.

Firmware tips for fast, smooth plots (the "other half" of speed):
  * your lines are ~280 bytes. Print every 10-20 ms (50-100 Hz), not every 100 ms.
    The filters can keep running at 200 Hz inside the ESP32.
  * build the whole line with ONE snprintf() and send it with ONE Serial.write();
    no Serial.flush(), no delay() in the sending path.
  * 280 B x 100 Hz = 28 KB/s. A real UART at 115200 baud carries only ~11.5 KB/s, so
    use 921600 on both sides. /dev/cu.usbmodem* (ESP32-S3 native USB) ignores baud.
  * keep t_ms = millis() as the first column: the x-axis is then real time.
"""
from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
import time
import textwrap
import traceback
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgba
from matplotlib.patches import Circle, Polygon
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# =============================================================================
# 1. CONFIG  -  the knobs. Change values here (or override with command-line flags)
# =============================================================================
@dataclass
class Config:
    # ---- connection --------------------------------------------------------
    port: str = "auto"            # "auto", or "/dev/cu.usbmodem1101", "COM5", ...
    baud: int = 115200            # must match Serial.begin() on a UART; ignored by USB-CDC
    line_prefix: str = "IMU"      # only lines starting with "IMU," are data ("" = no prefix)
    auto_zero_on_connect: bool = True   # ΔX/ΔY start at 0 when the first sample arrives

    # ---- time axis ---------------------------------------------------------
    window_steps: tuple = (5, 10, 20, 30, 60, 120)   # seconds; [ and ] move along this list
    window_index: int = 3                            # start at window_steps[3] = 30 s

    # ---- speed -------------------------------------------------------------
    fps: int = 30                 # screen refresh rate (what you SEE)
    max_points: int = 1000        # max points drawn per line (longer windows are decimated
                                  # with a min/max envelope, so spikes are never lost)
    use_blit: bool = True         # redraw only the moving lines (fast). If the window ever
                                  # looks glitchy on your machine set False (slower, safest).
    max_rate_hz: int = 250        # buffer size = buffer_s * max_rate_hz samples
    buffer_s: float = 120.0       # how much history is kept in RAM (>= largest window)

    # ---- fixed Y ranges ----------------------------------------------------
    # Symmetric limits (+-value). None = auto-scale. Key 'm' cycles through the presets.
    #   acc  -> all three linear-acceleration charts      (m/s²)
    #   gyro -> gyroscope chart                           (deg/s)
    #   vel  -> velocity chart                            (m/s)
    #   att  -> attitude chart                            (deg)
    range_presets: dict = field(default_factory=lambda: {
        "overview": {"acc": 6.0, "gyro": 300.0, "vel": 1.0, "att": 45.0},
        "detail":   {"acc": 1.0, "gyro": 30.0,  "vel": 0.2, "att": 10.0},   # for "is it really still?"
        "auto":     None,
    })
    range_preset: str = "overview"

    # ---- top-down displacement --------------------------------------------
    trail_s: float = 120.0                 # how many seconds of path to draw
    topdown_min_half_span_m: float = 0.5   # never zoom in closer than +-0.5 m

    # ---- 3D robot ----------------------------------------------------------
    box_size: tuple = (0.9, 1.4, 0.4)      # X, Y, Z size of the drawn box (forward = +Y, longest)
    euler_signs: tuple = (1, 1, 1)         # flip a 1 to -1 if roll / pitch / yaw rotates the wrong way

    # ---- logging -----------------------------------------------------------
    log: bool = False                      # also record everything to logs/imu_<time>.csv
    log_dir: str = "logs"

    figsize: tuple = (16, 9)


# =============================================================================
# 2. DATA FORMAT
# =============================================================================
# The 40 values after "IMU," (same names/order as the FIELDS list of your first visualizer).
DEFAULT_FIELDS = (
    "t_ms", "dt",                          # millis() on the ESP32, loop dt in seconds
    "ax", "ay", "az",                      # body-frame accel, calibrated (m/s²)  -> |a| ~ 9.81 at rest
    "lax", "lay", "laz",                   # linear accel (gravity removed), unfiltered (m/s²)
    "flax", "flay", "flaz",                # linear accel, filtered
    "gx", "gy", "gz",                      # gyro, unfiltered (deg/s)
    "gbx", "gby", "gbz",                   # gyro bias (deg/s)
    "fgx", "fgy", "fgz",                   # gyro, filtered (deg/s)
    "magx", "magy", "magz",                # magnetometer
    "roll", "pitch", "yaw",                # attitude (deg)
    "vx", "vy", "vz",                      # velocity (m/s)
    "px", "py", "pz",                      # position (m)
    "temp_c", "moving", "still",           # °C, 0/1, 0/1
    "still_failure_mask",                  # bit i set = still-check number i is failing
    "stdA", "trust",                       # g, 0..1
    "rollG", "pitchG",                     # gyro-only shadow attitude (deg)
)
FAIL_BITS = 8
DERIVED = (("speed", "a_norm", "drift_roll", "drift_pitch")      # computed here
           + tuple(f"fb{i}" for i in range(FAIL_BITS)))          # fb<i> = bit i of still_failure_mask
CHANNELS = ("t",) + DEFAULT_FIELDS[1:] + DERIVED      # 't' is in seconds
CH = {name: i for i, name in enumerate(CHANNELS)}
N_CH = len(CHANNELS)


def make_colmap(names):
    """column index -> (channel index, scale) or None if we do not know that column"""
    cm = []
    for n in names:
        n = n.strip()
        if n == "t_ms":
            cm.append((CH["t"], 1e-3))
        elif n in CH and n not in ("t",) + DERIVED:
            cm.append((CH[n], 1.0))
        else:
            cm.append(None)
    return cm


def finish_row(row):
    v = row[[CH["vx"], CH["vy"], CH["vz"]]]
    row[CH["speed"]] = math.sqrt(float(np.nansum(v * v))) if np.isfinite(v).any() else np.nan
    b = row[[CH["ax"], CH["ay"], CH["az"]]]
    row[CH["a_norm"]] = math.sqrt(float(np.sum(b * b))) if np.isfinite(b).all() else np.nan
    for g, att, dst in (("rollG", "roll", "drift_roll"), ("pitchG", "pitch", "drift_pitch")):
        d = row[CH[g]] - row[CH[att]]
        row[CH[dst]] = ((d + 180.0) % 360.0) - 180.0 if np.isfinite(d) else np.nan
    m = row[CH["still_failure_mask"]]
    if np.isfinite(m):
        for i in range(FAIL_BITS):
            row[CH[f"fb{i}"]] = (int(m) >> i) & 1
    return row


class LineParser:
    def __init__(self, prefix=""):
        self.prefix = prefix
        self.colmap = make_colmap(DEFAULT_FIELDS)
        self.last_reason = None            # why the last line was rejected (None = fine / header)

    def feed(self, line: str, host_t: float):
        self.last_reason = None
        line = line.strip()
        if not line:
            return None
        if line[0] != "#" and self.prefix:                      # data lines must carry the prefix
            if not line.startswith(self.prefix + ","):
                self.last_reason = f"line does not start with '{self.prefix},'"
                return None
            line = line[len(self.prefix) + 1:]
        if line[0] == "#" or line.startswith("t_ms"):           # header line
            self.colmap = make_colmap(line.lstrip("#").split(","))
            return None
        parts = line.split(",")
        if len(parts) != len(self.colmap):                      # boot message / half line / wrong layout
            self.last_reason = f"got {len(parts)} values, expected {len(self.colmap)}"
            return None
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            self.last_reason = "a value is not a number"
            return None
        row = np.full(N_CH, np.nan)
        for v, m in zip(vals, self.colmap):
            if m is not None:
                row[m[0]] = v * m[1]
        if np.isnan(row[0]):                                    # no t_ms column -> PC clock
            row[0] = host_t
        return finish_row(row)


class Ring:
    """Fixed-size ring buffer, shape (channels, capacity). Only used from the GUI thread."""

    def __init__(self, cap):
        self.buf = np.full((N_CH, cap), np.nan)
        self.cap, self.n = cap, 0

    def clear(self):
        self.buf[:] = np.nan
        self.n = 0

    def push(self, row):
        self.buf[:, self.n % self.cap] = row
        self.n += 1

    def last_time(self):
        return None if self.n == 0 else float(self.buf[0, (self.n - 1) % self.cap])

    def window(self, t_min, rows=None):
        size = min(self.n, self.cap)
        if size == 0:
            return None
        idx = np.arange(self.n - size, self.n) % self.cap
        i0 = int(np.searchsorted(self.buf[0, idx], t_min, side="left"))
        i0 = min(i0, size - 1)
        if rows is None:
            return self.buf[:, idx[i0:]]
        return self.buf[np.ix_(rows, idx[i0:])]


# =============================================================================
# 3. PANELS  -  what is drawn on the left. Add / remove / edit entries here.
# =============================================================================
X_C, Y_C, Z_C = "#d62728", "#2ca02c", "#1f77b4"      # same colours as the 3D axes: X red, Y green, Z blue
FAINT = dict(alpha=0.30, lw=0.9)


# Names for the bits of still_failure_mask. Your firmware sets bit i when check number i fails
# (e.g. mask 4 = bit 2). Rename them to what they mean in your code, e.g. {0: "gyro", 1: "accel", ...}
STILL_FAIL_BITS = {0: "bit0", 1: "bit1", 2: "bit2", 3: "bit3", 4: "bit4", 5: "bit5"}


@dataclass
class Line:
    field: str                 # channel name (see DEFAULT_FIELDS / DERIVED)
    label: str                 # legend text; start with "_" to hide it from the legend
    color: str
    alpha: float = 1.0
    lw: float = 1.3
    ls: str = "-"
    step: bool = False         # draw as a step (for 0/1 signals)
    scale: float = 1.0         # drawn value = value * scale + offset
    offset: float = 0.0


@dataclass
class Panel:
    title: str
    ylabel: str
    pos: tuple                 # (row, col) on the left grid (4 rows x 2 cols)
    lines: list
    range_key: str | None = None      # which Config.range_presets entry sets its Y limits
    ylim: tuple | None = None         # or fixed limits that ignore presets
    colspan: int = 1
    yticks: tuple | None = None       # optional ((position, "label"), ...)


def _acc(axis, color, pos):
    return Panel(f"Linear acceleration {axis.upper()} (m/s²)", "m/s²", pos,
                 [Line(f"la{axis}", "raw", color, **FAINT), Line(f"fla{axis}", "filtered", color)],
                 range_key="acc")


_bits = sorted(STILL_FAIL_BITS)
PANELS = [
    _acc("x", X_C, (0, 0)),
    _acc("y", Y_C, (0, 1)),
    _acc("z", Z_C, (1, 0)),
    Panel("Gyroscope (deg/s)   faint = raw", "deg/s", (1, 1),
          [Line("gx", "_", X_C, **FAINT), Line("gy", "_", Y_C, **FAINT), Line("gz", "_", Z_C, **FAINT),
           Line("fgx", "x", X_C), Line("fgy", "y", Y_C), Line("fgz", "z", Z_C)], range_key="gyro"),
    Panel("Velocity (m/s)", "m/s", (2, 0),
          [Line("vx", "vx", X_C), Line("vy", "vy", Y_C), Line("vz", "vz", Z_C),
           Line("speed", "|v|", "k", lw=1.0, ls="--")], range_key="vel"),
    Panel("Attitude vs gyro-only shadow (deg)", "deg", (2, 1),
          [Line("roll", "roll", X_C), Line("pitch", "pitch", Y_C),
           Line("rollG", "roll gyro-only", X_C, ls="--", alpha=0.8, lw=1.0),
           Line("pitchG", "pitch gyro-only", Y_C, ls="--", alpha=0.8, lw=1.0)], range_key="att"),
    Panel("State: trust, still, moving", "", (3, 0),
          [Line("trust", "trust", "#7b3fa0"), Line("still", "still", "#444444", lw=1.6, step=True),
           Line("moving", "moving", "#e08a00", lw=1.2, step=True)], ylim=(-0.05, 1.15)),
    Panel("Why not still?   bar = that check is failing", "", (3, 1),
          [Line(f"fb{b}", "_", "#c0392b", lw=1.6, step=True, scale=0.55, offset=j) for j, b in enumerate(_bits)],
          ylim=(-0.2, len(_bits)),
          yticks=tuple((j + 0.27, STILL_FAIL_BITS[b]) for j, b in enumerate(_bits))),
]


# =============================================================================
# 4. DATA SOURCES  (all run in a background thread and push parsed rows into a queue)
# =============================================================================
class Source(threading.Thread):
    def __init__(self, q, parser, cfg):
        super().__init__(daemon=True)
        self.q, self.parser, self.cfg = q, parser, cfg
        self.stop_evt = threading.Event()
        self.state = "starting"
        self.rejected, self.last_reject = 0, ""

    def stop(self):
        self.stop_evt.set()

    def handle_line(self, line):
        row = self.parser.feed(line, time.monotonic())
        if row is not None:
            self.q.put(row)
        elif self.parser.last_reason:
            self.rejected += 1
            self.last_reject = f"{self.parser.last_reason}:  {line.strip()[:50]}"


def find_port():
    from serial.tools import list_ports
    hints = ("usbmodem", "usbserial", "ttyacm", "ttyusb", "slab", "ch340", "cp210", "com")
    for p in list_ports.comports():
        text = (p.device + " " + (p.description or "")).lower()
        if any(h in text for h in hints):
            return p.device
    return None


class SerialSource(Source):
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
                        chunk = ser.read(max(1, ser.in_waiting))     # grab everything that is waiting
                        if not chunk:
                            continue
                        buf += chunk
                        *lines, buf = buf.split(b"\n")
                        for ln in lines:
                            self.handle_line(ln.decode("ascii", "ignore"))
                        if len(buf) > 65536:                         # garbage guard
                            buf = b""
            except (serial.SerialException, OSError) as e:
                self.state = f"serial error: {str(e)[:40]}"
                time.sleep(1.0)


class DemoSource(Source):
    """Fake robot: still -> crawling -> still -> tilt event, repeating every 12 s."""

    def run(self):
        self.state = "demo data"
        rng = np.random.default_rng(7)
        rate, dt = 100.0, 0.01
        t0, n = time.monotonic(), 0
        lp, glp, v, p = np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)
        prev = np.array([0.0, 5.0, -172.0])
        still_cnt = 0
        while not self.stop_evt.is_set():
            target = int((time.monotonic() - t0) * rate)
            while n < target:
                t, n = n * dt, n + 1
                ph = t % 12.0
                moving = 2.0 <= ph < 6.0
                a_true = (np.array([0.4 * math.sin(2.1 * t), 2.2 * math.sin(2 * math.pi * 1.4 * t), 0.0])
                          if moving else np.zeros(3))
                a_raw = a_true + rng.normal(0, 0.9 if moving else 0.06, 3)
                if moving and rng.random() < 0.02:
                    a_raw = a_raw + rng.normal(0, 3.5, 3)
                lp += 0.12 * (a_raw - lp)
                tilt = -30.0 * math.sin(math.pi * (ph - 9.0)) if 9.0 <= ph < 10.0 else 0.0
                att = np.array([0.3 * math.sin(t), 5.0 + tilt, ((-172 + 25 * math.sin(0.3 * t) + 180) % 360) - 180])
                d_att = att - prev
                d_att[2] = (d_att[2] + 180) % 360 - 180
                prev = att
                g_raw = d_att * rate + rng.normal(0, 3.0 if moving else 0.4, 3)
                glp += 0.2 * (g_raw - glp)
                still_cnt = still_cnt + 1 if (not moving and tilt == 0.0) else 0
                still = still_cnt > 30
                v += lp * dt
                v = v * 0.0 if still else v * 0.996
                p += v * dt
                vals = {"t": t,
                        "dt": 0.006, "ax": lp[0], "ay": lp[1], "az": 9.8 + lp[2],
                        "lax": a_raw[0], "lay": a_raw[1], "laz": a_raw[2],
                        "flax": lp[0], "flay": lp[1], "flaz": lp[2],
                        "gx": g_raw[0], "gy": g_raw[1], "gz": g_raw[2],
                        "fgx": glp[0], "fgy": glp[1], "fgz": glp[2],
                        "roll": att[0], "pitch": att[1], "yaw": att[2],
                        "rollG": att[0] + 0.2 * math.sin(0.4 * t), "pitchG": att[1] + 0.5 * math.sin(0.2 * t),
                        "vx": v[0], "vy": v[1], "vz": v[2], "px": p[0], "py": p[1], "pz": p[2],
                        "temp_c": 36.3 + 0.01 * t, "trust": 1.0 if still else (0.2 if moving else 0.5),
                        "stdA": 0.0057 * (30 if moving else 1), "still": float(still), "moving": float(moving),
                        "still_failure_mask": 0.0 if still else float((4 if moving else 0) | (1 if tilt else 0))}
                row = np.full(N_CH, np.nan)
                for k, val in vals.items():
                    row[CH[k]] = val
                self.q.put(finish_row(row))
            time.sleep(0.005)


class ReplaySource(Source):
    def __init__(self, q, parser, cfg, path, speed=1.0):
        super().__init__(q, parser, cfg)
        self.path, self.speed = path, speed

    def run(self):
        self.state = f"replay {Path(self.path).name}"
        wall0, start = time.monotonic(), None
        for ln in Path(self.path).read_text().splitlines():
            if self.stop_evt.is_set():
                return
            row = self.parser.feed(ln, time.monotonic())
            if row is None:
                continue
            start = row[0] if start is None else start
            delay = (row[0] - start) / self.speed - (time.monotonic() - wall0)
            if delay > 0 and self.stop_evt.wait(delay):
                return
            self.q.put(row)
        self.state = "replay finished"


class Logger:
    def __init__(self, folder, prefix=""):
        self.prefix = f"{prefix}," if prefix else ""
        Path(folder).mkdir(parents=True, exist_ok=True)
        self.path = Path(folder) / f"imu_{datetime.now():%Y-%m-%d_%H-%M-%S}.csv"
        self.f = open(self.path, "w")
        self.f.write("#" + ",".join(DEFAULT_FIELDS) + "\n")
        self._t = time.monotonic()
        self._names = DEFAULT_FIELDS[1:]
        self._idx = [CH[n] for n in self._names]

    def write_row(self, row):
        vals = ",".join(f"{x:.4f}" for x in row[self._idx])
        self.f.write(f"{self.prefix}{row[0] * 1000:.1f},{vals}\n")
        if time.monotonic() - self._t > 1.0:
            self.f.flush()
            self._t = time.monotonic()

    def close(self):
        self.f.close()


# =============================================================================
# 5. DRAWING HELPERS
# =============================================================================
class Blitter:
    """Redraw only the animated artists on top of a cached background (much faster
    than redrawing the whole figure). With enabled=False it just redraws everything."""

    def __init__(self, canvas, enabled):
        self.canvas, self.enabled, self.bg, self.artists = canvas, enabled, None, []
        canvas.mpl_connect("draw_event", self._on_draw)

    def add(self, *arts):
        for a in arts:
            if self.enabled:
                a.set_animated(True)
            self.artists.append(a)

    def _draw_artists(self):
        fig = self.canvas.figure
        for a in self.artists:
            if not a.get_visible():
                continue
            proj = getattr(a, "do_3d_projection", None)       # 3D polygons need this before drawing
            if proj is not None:
                try:
                    proj()
                except TypeError:                              # old matplotlib wants a renderer
                    proj(self.canvas.get_renderer())
            fig.draw_artist(a)

    def _on_draw(self, _event):
        if self.enabled:
            self.bg = self.canvas.copy_from_bbox(self.canvas.figure.bbox)
            self._draw_artists()

    def update(self):
        if not self.enabled or self.bg is None:
            self.canvas.draw_idle()
            return
        self.canvas.restore_region(self.bg)
        self._draw_artists()
        self.canvas.blit(self.canvas.figure.bbox)


# Top-view robot icon in the heading dial, body frame: x = right, y = front (dial radius = 1)
ICON_BODY = np.array([(-0.20, -0.34), (0.20, -0.34), (0.20, 0.34), (-0.20, 0.34)])
ICON_NOSE = np.array([(-0.14, 0.34), (0.14, 0.34), (0.0, 0.56)])
ICON_RIGHT = np.array([(0.20, -0.34), (0.20, 0.34)])
ICON_NEEDLE = np.array([(0.0, 0.56), (0.0, 0.86)])
ICON_HEAD = np.array([(-0.07, 0.84), (0.07, 0.84), (0.0, 1.0)])


def rot2(pts, psi):
    """rotate body-frame points by psi (radians, counter-clockwise) into dial coordinates (X right, Y up)"""
    c, s = math.cos(psi), math.sin(psi)
    return np.column_stack([pts[:, 0] * c - pts[:, 1] * s, pts[:, 0] * s + pts[:, 1] * c])


def rot_matrix(roll, pitch, yaw, signs=(1, 1, 1)):
    r, p, y = np.radians([roll * signs[0], pitch * signs[1], yaw * signs[2]])
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def box_faces(R, size):
    sx, sy, sz = np.array(size) / 2
    c = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)]) @ R.T
    faces = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [c[list(f)] for f in faces]          # order: x-, x+, y-, y+(front), z-, z+(top)


def nice_ceil(x):
    if not np.isfinite(x) or x <= 0:
        return None
    e = 10 ** math.floor(math.log10(x))
    for m in (1, 1.2, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10):
        if m * e >= x - 1e-12:
            return round(m * e, 6)


# =============================================================================
# 6. THE APP
# =============================================================================
class App:
    def __init__(self, cfg, source, q, logger=None, mode="LIVE"):
        self.cfg, self.source, self.q, self.logger, self.mode = cfg, source, q, logger, mode
        cap = int(max(cfg.buffer_s, max(cfg.window_steps), cfg.trail_s) * cfg.max_rate_hz)
        self.ring = Ring(cap)
        self.peak = np.zeros(N_CH)                  # biggest |value| ever seen, for range suggestions
        self.win_i = cfg.window_index
        self.preset = cfg.range_preset
        self.paused = False
        self.zero, self.zero_t = np.zeros(2), -np.inf
        self.rx_total, self.rx_rate, self._rx_mark = 0, 0.0, (time.monotonic(), 0)
        self.fps, self._frames, self._fps_t = 0.0, 0, time.monotonic()
        self.last_rx = None
        self._zeroed = False
        self.last_row = None
        self._trail = None
        self.need_full = False
        self.last_slow = 0.0
        self._build_figure()

    # ------------------------------------------------------------------ figure
    def _build_figure(self):
        cfg = self.cfg
        plt.rcParams.update({"font.size": 8, "axes.titlesize": 9.5, "axes.titleweight": "bold"})
        self.fig = fig = plt.figure(figsize=cfg.figsize)
        try:
            fig.canvas.manager.set_window_title("Crawling Bot V2 - IMU visualizer")
        except Exception:
            pass
        gl = fig.add_gridspec(4, 2, left=0.045, right=0.625, top=0.955, bottom=0.08, hspace=0.38,
                              wspace=0.18, height_ratios=[1, 1, 1, 0.9])
        gr = fig.add_gridspec(4, 1, left=0.675, right=0.975, top=0.955, bottom=0.08,
                              height_ratios=[1.15, 1.0, 0.85, 1.0], hspace=0.2)
        self.blit = Blitter(fig.canvas, cfg.use_blit)
        last_row = max(p.pos[0] + 0 for p in PANELS)

        self.pw = []                                           # (panel, ax, [(Line, artist)], clip_text)
        for p in PANELS:
            r, c = p.pos
            ax = fig.add_subplot(gl[r, c:c + p.colspan])
            ax.set_title(p.title, loc="left")
            ax.set_ylabel(p.ylabel)
            ax.grid(alpha=0.3)
            ax.axhline(0, color="k", lw=0.6, alpha=0.35)
            if r < last_row:
                ax.tick_params(labelbottom=False)
            else:
                ax.set_xlabel("seconds ago   (0 = now)")
            if p.yticks:
                ax.set_yticks([t[0] for t in p.yticks])
                ax.set_yticklabels([t[1] for t in p.yticks], fontsize=7)
            arts = []
            for ln in p.lines:
                (a,) = ax.plot([], [], color=ln.color, alpha=ln.alpha, lw=ln.lw, ls=ln.ls, label=ln.label,
                               drawstyle="steps-post" if ln.step else "default")
                arts.append((ln, a))
                self.blit.add(a)
            if any(not ln.label.startswith("_") for ln in p.lines):
                ax.legend(loc="upper left", ncol=4, fontsize=7, framealpha=0.75, borderaxespad=0.2,
                          handlelength=1.4, columnspacing=1.0)
            clip = ax.text(0.995, 0.03, "", transform=ax.transAxes, ha="right", va="bottom",
                           color="red", fontsize=8, fontweight="bold")
            self.blit.add(clip)
            self.pw.append((p, ax, arts, clip))

        # ---- 3D orientation ----
        ax3 = fig.add_subplot(gr[0], projection="3d")
        ax3.set_title("Robot 3D orientation   (teal face = front = +Y)", fontsize=9.5, fontweight="bold")
        ax3.set_xlim(-1, 1); ax3.set_ylim(-1, 1); ax3.set_zlim(-1, 1)
        ax3.set_box_aspect((1, 1, 1))
        ax3.set_xlabel("X"); ax3.set_ylabel("Y"); ax3.set_zlabel("Z")
        ax3.tick_params(labelsize=6)
        ax3.view_init(elev=24, azim=-58)
        base, top, front = "#e8873a", "#f4b66f", "#2a9d8f"
        cols = [to_rgba(c, 0.88) for c in (base, base, base, front, base, top)]
        self.poly = Poly3DCollection(box_faces(np.eye(3), cfg.box_size), facecolors=cols,
                                     edgecolors="k", linewidths=0.8)
        ax3.add_collection3d(self.poly)
        self.axes3d_lines = [ax3.plot([0, 0], [0, 0], [0, 0], color=c, lw=3)[0] for c in (X_C, Y_C, Z_C)]
        self.t3d = ax3.text2D(0.02, 0.95, "", transform=ax3.transAxes, fontsize=10, fontweight="bold")
        self.blit.add(self.poly, *self.axes3d_lines, self.t3d)

        # ---- heading dial ----
        gh = gr[1].subgridspec(1, 2, width_ratios=[1.0, 1.1], wspace=0.02)
        axd = fig.add_subplot(gh[0])
        axd.set_aspect("equal"); axd.set_xlim(-1.6, 1.6); axd.set_ylim(-1.6, 1.6); axd.axis("off")
        axd.add_patch(Circle((0, 0), 1.0, fill=False, ec="0.35", lw=1.3))
        for deg in range(0, 360, 10):
            t = math.radians(deg)
            r0 = 0.86 if deg % 90 == 0 else (0.92 if deg % 30 == 0 else 0.96)
            axd.plot([r0 * math.sin(t), math.sin(t)], [r0 * math.cos(t), math.cos(t)], color="0.35",
                     lw=1.5 if deg % 90 == 0 else 0.8)
            if deg % 90 == 0:                                   # cardinal points, outside the ring
                lab = {0: "+Y", 90: "+X", 180: "−Y", 270: "−X"}[deg]
                axd.text(1.27 * math.sin(t), 1.27 * math.cos(t), f"{lab}\n{deg}°", fontsize=6.8, ha="center",
                         va="center", color="0.25", fontweight="bold", linespacing=0.95)
            elif deg % 30 == 0:
                axd.text(1.16 * math.sin(t), 1.16 * math.cos(t), f"{deg}", fontsize=6.3, ha="center",
                         va="center", color="0.4")
        (self.d_arc,) = axd.plot([], [], color="#7b3fa0", lw=5, alpha=0.35, solid_capstyle="butt")
        self.d_body = axd.add_patch(Polygon(ICON_BODY, closed=True, fc="#e8873a", ec="k", lw=1.0, zorder=3))
        self.d_nose = axd.add_patch(Polygon(ICON_NOSE, closed=True, fc=Y_C, ec="k", lw=1.0, zorder=4))
        (self.d_right,) = axd.plot([], [], color=X_C, lw=4.5, solid_capstyle="butt", zorder=5)
        (self.d_needle,) = axd.plot([], [], color="#2a9d8f", lw=2.5, zorder=3)
        self.d_head = axd.add_patch(Polygon(ICON_HEAD, closed=True, fc="#2a9d8f", ec="#2a9d8f", zorder=3))
        axh = fig.add_subplot(gh[1])
        axh.axis("off")
        axh.text(0.0, 0.98, "HEADING   (0° = +Y, clockwise)", transform=axh.transAxes, va="top",
                 fontsize=7.5, color="0.4")
        self.t_head_big = axh.text(0.0, 0.64, "", transform=axh.transAxes, va="center", fontsize=28,
                                   fontweight="bold")
        self.t_head_sub = axh.text(0.0, 0.40, "", transform=axh.transAxes, va="center", fontsize=8.5,
                                   family="monospace")
        self.t_head_turn = axh.text(0.0, 0.26, "", transform=axh.transAxes, va="center", fontsize=9.5,
                                    fontweight="bold")
        axh.text(0.0, 0.10, "■ green = front (+Y)", transform=axh.transAxes, va="center", fontsize=7.5, color=Y_C)
        axh.text(0.0, 0.00, "■ red = right side (+X)", transform=axh.transAxes, va="center", fontsize=7.5, color=X_C)
        self.blit.add(self.d_arc, self.d_body, self.d_nose, self.d_right, self.d_needle, self.d_head,
                      self.t_head_big, self.t_head_sub, self.t_head_turn)

        # ---- status text ----
        axs = fig.add_subplot(gr[2])
        axs.axis("off")
        self.t_badge = axs.text(0.0, 1.0, "", transform=axs.transAxes, va="top", fontsize=10,
                                fontweight="bold", family="monospace")
        self.t_body = axs.text(0.0, 0.86, "", transform=axs.transAxes, va="top", fontsize=8,
                               family="monospace", linespacing=1.25)
        self.blit.add(self.t_badge, self.t_body)

        # ---- top-down ----
        axt = self.ax_td = fig.add_subplot(gr[3])
        axt.set_title("Top-down displacement (dead reckoning, since zero)", loc="left")
        axt.set_xlabel("ΔX (m)"); axt.set_ylabel("ΔY (m)")
        axt.grid(alpha=0.3)
        axt.set_aspect("equal", adjustable="box")
        axt.set_xlim(-0.5, 0.5); axt.set_ylim(-0.5, 0.5)
        axt.plot([0], [0], "o", color="#2ca02c", ms=7, label="zero point")
        (self.td_trail,) = axt.plot([], [], color="#8c5bbf", lw=2, label="path")
        (self.td_now,) = axt.plot([], [], "x", color="#d62728", ms=9, mew=2, label="now")
        (self.td_head,) = axt.plot([], [], color="#2a9d8f", lw=3, solid_capstyle="round", label="heading")
        axt.legend(loc="upper left", fontsize=7, framealpha=0.75)
        self.blit.add(self.td_trail, self.td_now, self.td_head)

        fig.text(0.045, 0.012, "space = pause     [ ] = time window     m = Y-range preset     "
                 "z = zero position     |     toolbar: zoom / pan / save", fontsize=8, color="0.4")

        fig.canvas.mpl_connect("key_press_event", self._on_key)
        fig.canvas.mpl_connect("close_event", lambda _e: self.shutdown())
        self._apply_ranges()
        self._apply_window()

    # ---------------------------------------------------------- user controls
    def _apply_ranges(self):
        preset = self.cfg.range_presets[self.preset]
        for p, ax, _a, _c in self.pw:
            if p.ylim is not None:
                ax.set_ylim(*p.ylim)
            elif preset is None:
                ax.set_autoscaley_on(True)
            else:
                lim = preset[p.range_key]
                ax.set_ylim(-lim, lim)
        self.need_full = True

    def _apply_window(self):
        w = self.cfg.window_steps[self.win_i]
        for _p, ax, _a, _c in self.pw:
            ax.set_xlim(-w, 0)
        self.need_full = True

    def _on_key(self, ev):
        if ev.key == " ":
            self.paused = not self.paused
        elif ev.key in ("[", "]"):
            d = -1 if ev.key == "[" else 1
            self.win_i = min(max(self.win_i + d, 0), len(self.cfg.window_steps) - 1)
            self._apply_window()
        elif ev.key == "m":
            names = list(self.cfg.range_presets)
            self.preset = names[(names.index(self.preset) + 1) % len(names)]
            self._apply_ranges()
        elif ev.key == "z" and self.last_row is not None:
            self.zero = np.array([self.last_row[CH["px"]], self.last_row[CH["py"]]])
            self.zero_t = self.last_row[CH["t"]]

    # ------------------------------------------------------------ main loop
    def _drain(self):
        rows = []
        try:
            while True:
                rows.append(self.q.get_nowait())
        except queue.Empty:
            pass
        if not rows:
            return
        for row in rows:
            last = self.ring.last_time()
            if last is not None and row[0] < last - 1.0:         # ESP32 rebooted: start fresh
                self.ring.clear()
                self.zero, self.zero_t, self._zeroed = np.zeros(2), -np.inf, False
            self.ring.push(row)
            if self.cfg.auto_zero_on_connect and not self._zeroed and np.isfinite(row[[CH["px"], CH["py"]]]).all():
                self.zero, self.zero_t, self._zeroed = row[[CH["px"], CH["py"]]].copy(), row[0], True
            if self.logger:
                self.logger.write_row(row)
        arr = np.vstack(rows)
        self.peak = np.fmax(self.peak, np.fmax.reduce(np.abs(arr), axis=0))
        self.rx_total += len(rows)
        self.last_rx = time.monotonic()

    def tick(self):
        now = time.monotonic()
        self._drain()
        if not self.paused:
            self._refresh_plots()
        if now - self.last_slow > 0.5:
            self._slow(now)
            self.last_slow = now
        self._update_status(now)
        self._frames += 1
        if self.need_full:
            self.need_full = False
            self.fig.canvas.draw_idle()
        else:
            self.blit.update()

    def _tick_safe(self):
        try:
            self.tick()
        except Exception:
            traceback.print_exc()

    def _slow(self, now):
        t0, n0 = self._rx_mark
        if now - t0 > 0:
            self.rx_rate = (self.rx_total - n0) / (now - t0)
        self._rx_mark = (now, self.rx_total)
        self.fps = self._frames / max(now - self._fps_t, 1e-6)
        self._frames, self._fps_t = 0, now
        if self.cfg.range_presets[self.preset] is None:           # auto mode: re-fit Y now and then
            for p, ax, _a, _c in self.pw:
                if p.ylim is None:
                    ax.relim()
                    ax.autoscale_view(scalex=False)
            self.need_full = True
        if self._fit_topdown():
            self.need_full = True

    def _fit_topdown(self):
        if self._trail is None:
            return False
        tx, ty = self._trail
        x0, x1, y0, y1 = tx.min(), tx.max(), ty.min(), ty.max()
        half = max(self.cfg.topdown_min_half_span_m, 0.6 * max(x1 - x0, y1 - y0))
        ax = self.ax_td
        (lx0, lx1), (ly0, ly1) = ax.get_xlim(), ax.get_ylim()
        span, m = lx1 - lx0, 0.05 * (lx1 - lx0)
        if x0 > lx0 + m and x1 < lx1 - m and y0 > ly0 + m and y1 < ly1 - m and span < 5 * half:
            return False
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)
        return True

    # ---------------------------------------------------------- refresh plots
    def _refresh_plots(self):
        cfg = self.cfg
        t_last = self.ring.last_time()
        if t_last is None:
            return
        win = cfg.window_steps[self.win_i]
        w = self.ring.window(t_last - win)
        m = w.shape[1]
        if m < 2:
            return
        self.last_row = w[:, -1]
        x = w[CH["t"]] - t_last
        if m > cfg.max_points:                                   # min/max envelope decimation
            k = max(2, m // (cfg.max_points // 2))
            B = m // k
            sl = slice(m - B * k, m)
            xb = np.repeat(x[sl].reshape(B, k).mean(axis=1), 2)

            def dec(y):
                yy = y[sl].reshape(B, k)
                lo, hi = np.fmin.reduce(yy, axis=1), np.fmax.reduce(yy, axis=1)
                swap = np.argmin(yy, axis=1) > np.argmax(yy, axis=1)
                out = np.empty(2 * B)
                out[0::2], out[1::2] = np.where(swap, hi, lo), np.where(swap, lo, hi)
                return out
        else:
            xb, dec = x, (lambda y: y)

        fixed = cfg.range_presets[self.preset] is not None
        for p, ax, arts, clip in self.pw:
            lo_seen, hi_seen = np.inf, -np.inf
            for ln, a in arts:
                y = dec(w[CH[ln.field]])
                if ln.scale != 1.0 or ln.offset != 0.0:
                    y = y * ln.scale + ln.offset
                a.set_data(xb, y)
                if fixed and p.ylim is None:
                    lo_seen = min(lo_seen, np.fmin.reduce(y))
                    hi_seen = max(hi_seen, np.fmax.reduce(y))
            lo, hi = ax.get_ylim()
            if fixed and p.ylim is None and (hi_seen > hi or lo_seen < lo):
                clip.set_text(f"clipped! peak {max(abs(hi_seen), abs(lo_seen)):.3g}")
                clip.set_visible(True)
            else:
                clip.set_visible(False)

        # 3D box + heading
        r = self.last_row
        R = None
        if np.isfinite(r[[CH["roll"], CH["pitch"], CH["yaw"]]]).all():
            R = rot_matrix(r[CH["roll"]], r[CH["pitch"]], r[CH["yaw"]], cfg.euler_signs)
            self.poly.set_verts(box_faces(R, cfg.box_size))
            for i, l3 in enumerate(self.axes3d_lines):
                v = R[:, i] * 1.2
                l3.set_data_3d([0, v[0]], [0, v[1]], [0, v[2]])
            self.t3d.set_text(f"R {r[CH['roll']]:+.1f}°   P {r[CH['pitch']]:+.1f}°   Y {r[CH['yaw']]:+.1f}°")
            self._update_heading(r)

        # top-down trail (positions relative to the zero point)
        tw = self.ring.window(max(t_last - cfg.trail_s, self.zero_t), rows=[CH["px"], CH["py"]])
        if tw is not None and tw.shape[1] >= 1:
            k = max(1, math.ceil(tw.shape[1] / 1500))
            idx = np.unique(np.r_[np.arange(0, tw.shape[1], k), tw.shape[1] - 1])
            tx, ty = tw[0, idx] - self.zero[0], tw[1, idx] - self.zero[1]
            ok = np.isfinite(tx) & np.isfinite(ty)
            if ok.any():
                tx, ty = tx[ok], ty[ok]
                self._trail = (tx, ty)
                self.td_trail.set_data(tx, ty)
                self.td_now.set_data([tx[-1]], [ty[-1]])
                if R is not None:
                    f = R @ np.array([0.0, 1.0, 0.0])
                    n = math.hypot(f[0], f[1])
                    if n > 1e-6:
                        L = 0.14 * (self.ax_td.get_xlim()[1] - self.ax_td.get_xlim()[0])
                        self.td_head.set_data([tx[-1], tx[-1] + f[0] / n * L], [ty[-1], ty[-1] + f[1] / n * L])

    # ---------------------------------------------------------------- heading
    def _update_heading(self, r):
        sign = self.cfg.euler_signs[2]
        yaw = r[CH["yaw"]] * sign                         # counter-clockwise positive (right-hand rule, Z up)
        psi = math.radians(yaw)
        heading = (-yaw) % 360.0                          # compass style: clockwise from +Y
        self.d_body.set_xy(rot2(ICON_BODY, psi))
        self.d_nose.set_xy(rot2(ICON_NOSE, psi))
        self.d_head.set_xy(rot2(ICON_HEAD, psi))
        for line, pts in ((self.d_right, ICON_RIGHT), (self.d_needle, ICON_NEEDLE)):
            xy = rot2(pts, psi)
            line.set_data(xy[:, 0], xy[:, 1])
        ts = np.radians(np.linspace(0.0, heading, max(2, int(heading / 3) + 2)))
        self.d_arc.set_data(0.93 * np.sin(ts), 0.93 * np.cos(ts))          # swept angle since 0°
        self.t_head_big.set_text(f"{heading:5.1f}°")
        self.t_head_sub.set_text(f"yaw {r[CH['yaw']]:+7.1f}° (as sent)")
        rate = r[CH["fgz"]] * sign                        # + = turning left (counter-clockwise)
        if np.isfinite(rate) and rate > 2.0:
            self.t_head_turn.set_text(f"↺ turning LEFT  {rate:.1f} °/s")
            self.t_head_turn.set_color("#d98200")
        elif np.isfinite(rate) and rate < -2.0:
            self.t_head_turn.set_text(f"↻ turning RIGHT {-rate:.1f} °/s")
            self.t_head_turn.set_color("#d98200")
        else:
            self.t_head_turn.set_text("not turning")
            self.t_head_turn.set_color("0.45")

    # ----------------------------------------------------------------- status
    def _update_status(self, now):
        age = None if self.last_rx is None else now - self.last_rx
        if self.paused:
            badge, color = "❚❚ PAUSED", "#d98200"
        elif age is None or age > 1.0:
            badge, color = "● NO DATA", "#d62728"
        else:
            badge, color = f"● {self.mode}", "#2ca02c" if self.mode == "LIVE" else "#d98200"
        self.t_badge.set_text(f"{badge}   {self.source.state[:38]}")
        self.t_badge.set_color(color)
        r = self.last_row
        if r is None:
            msg = "waiting for data ..."
            if self.source.rejected:
                msg += (f"\n{self.source.rejected} lines received but REJECTED, last one:\n"
                        + textwrap.fill(self.source.last_reject, 56))
            self.t_body.set_text(msg)
            return
        win = self.cfg.window_steps[self.win_i]
        dx, dy = r[CH["px"]] - self.zero[0], r[CH["py"]] - self.zero[1]
        mask = int(r[CH["still_failure_mask"]]) if np.isfinite(r[CH["still_failure_mask"]]) else 0
        fails = ", ".join(STILL_FAIL_BITS.get(i, f"bit{i}") for i in range(FAIL_BITS) if (mask >> i) & 1) or "-"
        dt = r[CH["dt"]]
        loop = 1.0 / dt if np.isfinite(dt) and dt > 0 else float("nan")
        yn = lambda x: "YES" if x >= 0.5 else "no"
        self.t_body.set_text(
            f"rx {self.rx_rate:4.0f} Hz  draw {self.fps:3.0f} fps  loop {loop:4.0f} Hz  win {win:g} s  {self.preset}\n"
            f"Temp {r[CH['temp_c']]:.1f} °C   Trust {r[CH['trust']]:.2f}   stdA {r[CH['stdA']]:.5f} g\n"
            f"Moving: {yn(r[CH['moving']])}   Still: {yn(r[CH['still']])}   fail-mask {mask} -> {fails}\n"
            f"Roll {r[CH['roll']]:+6.1f}°  Pitch {r[CH['pitch']]:+6.1f}°  Yaw {r[CH['yaw']]:+7.1f}°  |a| {r[CH['a_norm']]:.2f}\n"
            f"Vel  x {r[CH['vx']]:+6.2f}  y {r[CH['vy']]:+6.2f}  z {r[CH['vz']]:+6.2f} m/s   |v| {r[CH['speed']]:.2f}\n"
            f"Pos  Δx {dx:+9.3f}  Δy {dy:+9.3f} m   dist {math.hypot(dx, dy):.3f} m\n"
            f"Gyro-only drift   roll {r[CH['drift_roll']]:+.1f}°   pitch {r[CH['drift_pitch']]:+.1f}°")

    # ------------------------------------------------------------- lifecycle
    def run(self):
        self.timer = self.fig.canvas.new_timer(interval=max(1, int(1000 / self.cfg.fps)))
        self.timer.add_callback(self._tick_safe)
        self.timer.start()
        plt.show()

    def shutdown(self):
        if getattr(self, "_closed", False):
            return
        self._closed = True
        self.source.stop()
        if self.logger:
            self.logger.close()
            print(f"\nlog saved: {self.logger.path}")
        self.print_summary()

    def print_summary(self):
        if self.rx_total == 0:
            return
        print("\nRange suggestions  (largest |value| seen x 1.2, rounded up to a 'nice' number)")
        cur = self.cfg.range_presets.get("overview") or {}
        for key in sorted({p.range_key for p in PANELS if p.range_key}):
            fields = {ln.field for p in PANELS if p.range_key == key for ln in p.lines}
            peak = max(self.peak[CH[f]] for f in fields)
            sug = nice_ceil(peak * 1.2)
            print(f"  {key:5s} peak {peak:9.3f}   suggested +-{sug}   (overview preset now +-{cur.get(key)})")


# =============================================================================
# 7. MAIN
# =============================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("port_pos", nargs="?", metavar="PORT", help="serial port (same as --port)")
    ap.add_argument("--port"); ap.add_argument("--baud", type=int)
    ap.add_argument("--prefix", help="line prefix of data lines, default IMU ('' = none)")
    ap.add_argument("--demo", action="store_true", help="fake data, no robot needed")
    ap.add_argument("--replay", metavar="CSV", help="play back a log written with --log")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")
    ap.add_argument("--window", type=float, help="initial time window in seconds")
    ap.add_argument("--fps", type=int); ap.add_argument("--preset", choices=["overview", "detail", "auto"])
    ap.add_argument("--log", action="store_true"); ap.add_argument("--no-blit", action="store_true")
    a = ap.parse_args()

    cfg = Config()
    if a.port_pos: cfg.port = a.port_pos
    if a.port: cfg.port = a.port
    if a.baud: cfg.baud = a.baud
    if a.prefix is not None: cfg.line_prefix = a.prefix
    if a.fps: cfg.fps = a.fps
    if a.preset: cfg.range_preset = a.preset
    if a.log: cfg.log = True
    if a.no_blit: cfg.use_blit = False
    if a.window:
        cfg.window_steps = tuple(sorted(set(cfg.window_steps) | {a.window}))
        cfg.window_index = cfg.window_steps.index(a.window)

    q, parser = queue.SimpleQueue(), LineParser(cfg.line_prefix)
    if a.demo:
        src, mode = DemoSource(q, parser, cfg), "DEMO"
    elif a.replay:
        src, mode = ReplaySource(q, parser, cfg, a.replay, a.speed), "REPLAY"
    else:
        src, mode = SerialSource(q, parser, cfg), "LIVE"
    logger = Logger(cfg.log_dir, cfg.line_prefix) if (cfg.log and mode != "REPLAY") else None
    app = App(cfg, src, q, logger, mode)
    src.start()
    app.run()


if __name__ == "__main__":
    sys.exit(main())