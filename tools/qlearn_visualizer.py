#!/usr/bin/env python3
"""
Crawling Bot V2 - Q-learning viewer  (matplotlib)
=================================================

    python qlearn_visualizer.py --demo                     # fake robot, learns in front of you
    python qlearn_visualizer.py --port auto                # find the ESP32 automatically
    python qlearn_visualizer.py /dev/cu.usbmodem1101
    python qlearn_visualizer.py --replay logs/qlearn.csv   # replay a log written with --log

What you see
------------
  Q table        the 25 poses (down x up leg angle) x 5 actions, colour = Q value
  best action    one cell per pose, showing what the greedy policy would do there
  reward         reward per episode + rolling mean (is it learning?)
  epsilon / TD   exploration rate and the size of the TD error (is it converging?)
  forward accel  live forward acceleration, the signal the reward is built from
  status         episode counter, current pose, last action, totals

Keys (click the window first)
-----------------------------
  space  pause / resume the display (data keeps being recorded)
  [  ]   shorter / longer episode history
  g      toggle the greedy-policy overlay on the Q table
  l      ask the robot to print its table again (sends "table")
  r      start a fresh learning run (sends "learn"; the firmware resets first)
  s      stop the robot (sends "stop")
  c      re-read the experiment configuration (sends "status")

Firmware protocol (one line per event, newline terminated)
----------------------------------------------------------
  Q,EP,<n>,<down>,<up>,<action>,<nextDown>,<nextUp>,<reward>,<td>,<q>,<epsilon>,<learn>
  Q,TABLE,<125 floats>                       row-major: state = down*5 + up, then 5 actions
  Q,STATUS,running=..,learn=..,episode=..,epsilon=..,total_reward=..,down=..,up=..
  Q,START|STOP|RESET|SAVED|LOADED|READY|ERR,...
  IMU,<t_ms>,...                             the normal telemetry, used for the accel panel

Unknown Q,<kind> lines are shown in the status panel instead of being dropped, so a
newer firmware never leaves the viewer silently blank.
"""
from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm

# =============================================================================
# 1. CONFIG
# =============================================================================
DEFAULT_STEP_DEG = 45
ANGLES = tuple(range(0, 181, DEFAULT_STEP_DEG))   # replaced by Q,CONFIG at runtime
ACTIONS = ("down+", "down-", "up+", "up-", "hold")
N_ACTIONS = 5

# Arrow drawn in the best-action grid. The leg names are kept in the label so
# there is no ambiguity about which way "up" is.
ACTION_GLYPH = {"down+": "D+", "down-": "D-", "up+": "U+", "up-": "U-", "hold": "·"}


@dataclass
class Config:
    port: str = "auto"
    baud: int = 115200
    log: bool = False
    log_dir: str = "logs"

    fps: int = 10                 # redraw rate; the table is small, no blitting needed
    history_steps: tuple = (100, 250, 500, 1000, 2500)
    history_index: int = 1        # start at 250 episodes
    accel_window_s: float = 30.0
    max_imu_points: int = 600

    # Must match TRAIN_ACCEL_DEADBAND_MS2 in include/config.h. The accel panel
    # draws the raw signal faint and the gated signal bold, so what you see is
    # exactly what the reward is built from.
    deadband_ms2: float = 0.15

    figsize: tuple = (16, 9)
    frames: int = 0               # >0: stop after N frames (used by --selftest)


# =============================================================================
# 2. PROTOCOL PARSING
# =============================================================================
@dataclass
class Episode:
    n: int
    down: int
    up: int
    action: str
    next_down: int
    next_up: int
    reward: float
    td: float
    q: float
    epsilon: float
    learn: int
    active: float = float("nan")   # fraction of the move above the deadband
    peak: float = float("nan")     # strongest gated push


class QParser:
    """Turns one serial line into an event tuple, or None."""

    def __init__(self):
        self.imu_cols: dict[str, int] | None = None
        self.last_reason: str | None = None

    def feed(self, line: str):
        self.last_reason = None
        line = line.strip()
        if not line:
            return None
        parts = line.split(",")
        tag = parts[0]
        if tag == "Q":
            return self._q(parts[1:])
        if tag == "IMU":
            return self._imu(parts[1:])
        return None

    # ------------------------------------------------------------------ Q,...
    def _q(self, parts):
        if not parts:
            return None
        kind = parts[0]

        if kind == "EP":
            if len(parts) < 12:
                self.last_reason = f"Q,EP needs 12 fields, got {len(parts)}"
                return None
            try:
                ep = Episode(
                    n=int(parts[1]), down=int(parts[2]), up=int(parts[3]),
                    action=parts[4], next_down=int(parts[5]), next_up=int(parts[6]),
                    reward=float(parts[7]), td=float(parts[8]), q=float(parts[9]),
                    epsilon=float(parts[10]), learn=int(parts[11]),
                )
                # Appended in the deadband revision; an older firmware stops at 12.
                if len(parts) >= 14:
                    ep.active = float(parts[12])
                    ep.peak = float(parts[13])
            except ValueError:
                self.last_reason = "Q,EP has a non-numeric field"
                return None
            return ("ep", ep)

        if kind == "TABLE":
            # Q,TABLE,<chunk>,<chunks>,<count>,<v0>,...
            # Older firmware sent Q,TABLE,<v0>,... with no chunk header; that
            # form is still accepted as a single chunk.
            try:
                if len(parts) >= 4 and parts[1].isdigit() and parts[2].isdigit():
                    chunk, chunks, count = int(parts[1]), int(parts[2]), int(parts[3])
                    values = parts[4:4 + count]
                else:
                    chunk, chunks, values = 1, 1, parts[1:]
                floats = np.array([float(v) for v in values])
            except ValueError:
                self.last_reason = "Q,TABLE has a non-numeric value"
                return None
            return ("table_chunk", chunk, chunks, floats)

        if kind == "CONFIG":
            config = {}
            for item in parts[1:]:
                if "=" in item:
                    key, _, value = item.partition("=")
                    config[key] = value
            return ("config", config)

        if kind == "STATUS":
            status = {}
            for item in parts[1:]:
                if "=" in item:
                    key, _, value = item.partition("=")
                    status[key] = value
            return ("status", status)

        return ("event", kind, ",".join(parts[1:]))

    # ---------------------------------------------------------------- IMU,...
    def _imu(self, parts):
        if not parts:
            return None
        if parts[0] == "t_ms":                       # header line
            self.imu_cols = {name: i for i, name in enumerate(parts)}
            return None
        if self.imu_cols is None:
            return None
        try:
            t_ms = float(parts[self.imu_cols["t_ms"]])
            yaw = float(parts[self.imu_cols["yaw"]])
            ax = float(parts[self.imu_cols["flax"]])
            ay = float(parts[self.imu_cols["flay"]])
        except (KeyError, IndexError, ValueError):
            return None
        # Forward = the robot's nose (body +Y). World frame is X north, Y west,
        # so the nose is (-sin yaw, cos yaw, 0) and the projection is a dot product.
        rad = math.radians(yaw)
        return ("imu", t_ms, -math.sin(rad) * ax + math.cos(rad) * ay)


# =============================================================================
# 3. DATA SOURCES
# =============================================================================
class Source(threading.Thread):
    accepts_commands = False

    def __init__(self, q, parser, cfg):
        super().__init__(daemon=True)
        self.q, self.parser, self.cfg = q, parser, cfg
        self.stop_evt = threading.Event()
        self.state = "starting"
        self.rejected, self.last_reject = 0, ""
        self.out_q = queue.SimpleQueue()
        self.logger = None

    def send(self, cmd):
        if self.accepts_commands:
            self.out_q.put((cmd + "\n").encode("ascii", "ignore"))

    def stop(self):
        self.stop_evt.set()

    def handle_line(self, line):
        if self.logger is not None:
            self.logger.write(line)
        event = self.parser.feed(line)
        if event is not None:
            self.q.put(event)
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
    accepts_commands = True

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
                        try:
                            while True:
                                ser.write(self.out_q.get_nowait())
                        except queue.Empty:
                            pass
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


class ReplaySource(Source):
    def __init__(self, q, parser, cfg, path, speed=1.0):
        super().__init__(q, parser, cfg)
        self.path, self.speed = path, speed

    def run(self):
        self.state = f"replay {Path(self.path).name}"
        for ln in Path(self.path).read_text().splitlines():
            if self.stop_evt.is_set():
                return
            self.handle_line(ln)
            if ln.startswith("Q,EP"):
                time.sleep(0.01 / max(self.speed, 1e-6))
        self.state = "replay finished"


class DemoSource(Source):
    """Fake robot that runs the same Q-learning rule as the firmware.

    The ground truth is a plausible gait: pushing the down leg back (down+)
    while the up leg is planted (low angle) moves the robot forward, and
    lifting the up leg afterwards keeps the cycle going. Everything else is
    noise or slightly negative, so a working learner has something to find.
    """

    def run(self):
        self.state = "demo data"
        rng = np.random.default_rng(3)
        alpha, gamma = 0.2, 0.9
        eps, eps_end, eps_decay = 1.0, 0.05, 0.98
        table = np.zeros((25, N_ACTIONS))
        down, up = 4, 4
        episode = 0
        total_reward = 0.0
        t0 = time.monotonic()
        self.handle_line("Q,READY,demo")
        self.handle_line("Q,START,learn=1,episode=0,epsilon=1.000")
        self.handle_line(
            "Q,CONFIG,step=45,notches=5,states=25,axis=Y,sign=+1,deadband=0.150,"
            "mode=2,pos=1.000,neg=-1.000,idle=-0.100,threshold=0.500,pause=400")
        # A short IMU header is enough: the parser maps columns by name, so the
        # demo does not have to fake all 50 firmware columns.
        self.handle_line("IMU,t_ms,yaw,flax,flay")

        while not self.stop_evt.is_set():
            state = down * 5 + up
            valid = valid_actions(down, up)
            if rng.random() < eps:
                action = int(rng.choice(valid))
            else:
                action = int(max(valid, key=lambda a: table[state, a]))

            nd, nu = down, up
            if ACTIONS[action] == "down+" and down < 4:
                nd += 1
            elif ACTIONS[action] == "down-" and down > 0:
                nd -= 1
            elif ACTIONS[action] == "up+" and up < 4:
                nu += 1
            elif ACTIONS[action] == "up-" and up > 0:
                nu -= 1
            next_state = nd * 5 + nu

            # Discrete reward, same rule as TRAIN_REWARD_MODE 2 in the firmware.
            raw = true_reward(down, up, ACTIONS[action]) + rng.normal(0, 0.15)
            if raw >= 0.5:
                reward = 1.0
            elif raw <= -0.5:
                reward = -0.5
            else:
                reward = 0.0
            td = reward + gamma * table[next_state][valid_actions(nd, nu)].max() - table[state, action]
            table[state, action] += alpha * td
            eps = max(eps_end, eps_end + (eps - eps_end) * eps_decay)

            episode += 1
            total_reward += reward
            self.handle_line(
                f"Q,EP,{episode},{ANGLES[down]},{ANGLES[up]},{ACTIONS[action]},"
                f"{ANGLES[nd]},{ANGLES[nu]},{reward:.4f},{td:.4f},"
                f"{table[state, action]:.4f},{eps:.3f},1,"
                f"{min(1.0, abs(reward) / 0.5):.4f},{reward:.4f}")
            self.handle_line(
                f"Q,STATUS,running=1,learn=1,episode={episode},epsilon={eps:.3f},"
                f"total_reward={total_reward:.3f},last_reward={reward:.3f},"
                f"down={ANGLES[nd]},up={ANGLES[nu]}")
            if episode % 10 == 0:
                flat = table.ravel()
                for i in range(0, len(flat), 100):
                    part = flat[i:i + 100]
                    self.handle_line(
                        f"Q,TABLE,{i // 100 + 1},{(len(flat) + 99) // 100},{len(part)},"
                        + ",".join(f"{v:.4f}" for v in part))
            # yaw = 0 -> body +X is north, so the nose (body +Y) is world +Y and
            # forward == flay. The value therefore goes in the flay column.
            t_ms = int((time.monotonic() - t0) * 1000.0)
            self.handle_line(f"IMU,{t_ms},0,0,{reward * 0.8 + rng.normal(0, 0.1):.4f}")
            down, up = nd, nu
            time.sleep(0.02)


def valid_actions(down: int, up: int, notches: int = 5) -> list[int]:
    """Actions that actually change the pose. `hold` never does."""
    out = []
    if down < notches - 1:
        out.append(0)
    if down > 0:
        out.append(1)
    if up < notches - 1:
        out.append(2)
    if up > 0:
        out.append(3)
    return out


def true_reward(down: int, up: int, action: str) -> float:
    """Ground truth used by the demo only.

    A plausible gait: the down leg sweeps back while the up leg is planted
    (up == 0), then the up leg lifts, the down leg swings forward while the up
    leg is planted high (up == 4), and the up leg comes back down. Each phase
    only pays in the pose where it makes sense, so the learner has to discover
    the whole 16-step cycle rather than one favourite action.
    """
    if action == "down+":
        return 1.0 if up == 0 else -0.3
    if action == "up+":
        return 0.2 if down == 4 else -0.2
    if action == "down-":
        return 0.2 if up == 4 else -0.2
    if action == "up-":
        return 0.2 if down == 0 else -0.2
    return -0.3


class Logger:
    def __init__(self, folder):
        Path(folder).mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.path = Path(folder) / f"qlearn_{stamp}.csv"
        self.fh = self.path.open("w")

    def write(self, line: str):
        self.fh.write(line.rstrip("\n") + "\n")

    def close(self):
        self.fh.close()


# =============================================================================
# 4. THE APP
# =============================================================================
class App:
    def __init__(self, cfg, source, q, logger=None, mode="LIVE"):
        self.cfg, self.source, self.q, self.logger, self.mode = cfg, source, q, logger, mode
        self.episodes: deque[Episode] = deque(maxlen=max(cfg.history_steps))
        self.step_deg = DEFAULT_STEP_DEG
        self.notches = 5
        self.table = np.zeros((self.notches * self.notches, N_ACTIONS))
        self._chunks: dict[int, np.ndarray] = {}
        self._chunk_total = 0
        self._have_table = False
        self.status: dict[str, str] = {}
        self.events: deque[str] = deque(maxlen=6)
        self.imu_t: deque[float] = deque(maxlen=cfg.max_imu_points)
        self.imu_a: deque[float] = deque(maxlen=cfg.max_imu_points)
        self.last_ep: Episode | None = None
        self._reward_flash_t = -1e9
        self._reward_flash_val = None
        self.paused = False
        self.hist_i = cfg.history_index
        self.show_greedy = True
        self.frames = 0
        self._build_figure()

    # ------------------------------------------------------------------ figure
    def _build_figure(self):
        cfg = self.cfg
        plt.rcParams.update({"font.size": 8, "axes.titlesize": 9.5, "axes.titleweight": "bold"})
        self.fig = fig = plt.figure(figsize=cfg.figsize)
        try:
            fig.canvas.manager.set_window_title("Crawling Bot V2 - Q-learning")
        except Exception:
            pass

        gl = fig.add_gridspec(3, 2, left=0.055, right=0.66, top=0.94, bottom=0.07,
                              hspace=0.42, wspace=0.22, height_ratios=[1.25, 1.0, 0.85])
        gr = fig.add_gridspec(2, 1, left=0.70, right=0.975, top=0.94, bottom=0.07,
                              hspace=0.45, height_ratios=[1.55, 0.75])

        # ---- Q table heatmap ----
        axq = fig.add_subplot(gl[0, :])
        axq.set_title("Q table   rows = pose (down / up leg angle), columns = action", loc="left")
        self.im = axq.imshow(np.zeros((self.notches * self.notches, N_ACTIONS)), aspect="auto",
                             cmap="RdBu_r", interpolation="nearest")
        axq.set_xticks(range(N_ACTIONS))
        axq.set_xticklabels(ACTIONS)
        axq.set_yticks([d * self.notches for d in range(self.notches)])
        axq.set_yticklabels([f"{ANGLES[d]}/0" for d in range(self.notches)], fontsize=6.5)
        axq.set_ylabel("down / up  (deg)")
        self.q_text = [[axq.text(a, s, "", ha="center", va="center", fontsize=5.6)
                        for a in range(N_ACTIONS)] for s in range(self.notches * self.notches)]
        self.ax_q = axq
        self.cbar = fig.colorbar(self.im, ax=axq, pad=0.01, fraction=0.025)
        self.cbar.set_label("Q", fontsize=7)
        self.cbar.ax.tick_params(labelsize=6)

        # ---- best action grid ----
        axg = fig.add_subplot(gl[1, 0])
        axg.set_title("Greedy policy   (cell = best action in that pose)", loc="left")
        axg.set_xlim(-0.5, 4.5)
        axg.set_ylim(-0.5, 4.5)
        axg.set_xticks(range(5))
        axg.set_xticklabels(ANGLES)
        axg.set_yticks(range(5))
        axg.set_yticklabels(ANGLES)
        axg.set_xlabel("up leg (deg)")
        axg.set_ylabel("down leg (deg)")
        axg.grid(alpha=0.3)
        self.g_text = [[axg.text(u, d, "", ha="center", va="center", fontsize=9,
                                 fontweight="bold") for u in range(5)] for d in range(5)]
        self.g_cell = [[axg.add_patch(plt.Rectangle((u - 0.5, d - 0.5), 1, 1, fill=False,
                                                    ec="#2a9d8f", lw=2.5, visible=False))
                        for u in range(5)] for d in range(5)]
        self.ax_g = axg

        # ---- reward per episode ----
        axr = fig.add_subplot(gl[1, 1])
        axr.set_title("Reward per episode", loc="left")
        axr.set_xlabel("episode")
        axr.set_ylabel("reward")
        axr.grid(alpha=0.3)
        axr.axhline(0, color="k", lw=0.6, alpha=0.35)
        (self.r_pts,) = axr.plot([], [], ".", color="#1f77b4", ms=3, alpha=0.45, label="episode")
        (self.r_avg,) = axr.plot([], [], "-", color="#d62728", lw=1.6, label="mean (20)")
        axr.legend(loc="upper left", fontsize=7, framealpha=0.75)
        self.ax_r = axr

        # ---- epsilon + TD error ----
        axe = fig.add_subplot(gl[2, 0])
        axe.set_title("Exploration and TD error", loc="left")
        axe.set_xlabel("episode")
        axe.set_ylabel("epsilon", color="#7b3fa0", labelpad=1)
        axe.tick_params(axis="y", colors="#7b3fa0", labelsize=7)
        axe.grid(alpha=0.3)
        axe.set_ylim(0, 1.05)
        (self.e_line,) = axe.plot([], [], "-", color="#7b3fa0", lw=1.6, label="epsilon")
        axe.legend(loc="upper right", fontsize=7, framealpha=0.75)
        self.ax_e = axe
        self.ax_td = axe.twinx()
        self.ax_td.set_ylabel("|TD error|", color="#e08a00", labelpad=1)
        self.ax_td.tick_params(axis="y", colors="#e08a00", labelsize=7)
        (self.td_line,) = self.ax_td.plot([], [], "-", color="#e08a00", lw=1.0, alpha=0.8,
                                          label="|TD|")
        self.ax_td.legend(loc="upper left", fontsize=7, framealpha=0.75)

        # ---- live forward acceleration ----
        axa = fig.add_subplot(gl[2, 1])
        axa.set_title("Forward acceleration   faint = raw, bold = after deadband", loc="left")
        axa.set_xlabel("seconds ago   (0 = now)")
        axa.set_ylabel("m/s²")
        axa.grid(alpha=0.3)
        axa.axhline(0, color="k", lw=0.6, alpha=0.35)
        (self.a_raw,) = axa.plot([], [], "-", color="#2ca02c", lw=0.8, alpha=0.30)
        (self.a_line,) = axa.plot([], [], "-", color="#2ca02c", lw=1.6)
        # The deadband itself, so "did it cross?" is a visual question.
        for sign in (1, -1):
            axa.axhline(sign * cfg.deadband_ms2, color="#c0392b", lw=0.9, ls="--", alpha=0.7)
        self.ax_a = axa

        # ---- status text ----
        axs = fig.add_subplot(gr[0])
        axs.axis("off")
        self.t_badge = axs.text(0.0, 1.0, "", transform=axs.transAxes, va="top", fontsize=11,
                                fontweight="bold", family="monospace")

        # The reward is the number that matters, so it gets its own big readout
        # and flashes when a new one arrives. The value stays on screen after
        # the flash so it can still be read.
        axs.text(0.0, 0.90, "REWARD", transform=axs.transAxes, va="top", fontsize=8,
                 color="0.4", fontweight="bold")
        self.t_reward = axs.text(0.0, 0.80, "-", transform=axs.transAxes, va="top",
                                 fontsize=34, fontweight="bold", family="monospace")
        self.t_reward_sub = axs.text(0.0, 0.50, "", transform=axs.transAxes, va="top",
                                     fontsize=8.5, family="monospace")

        self.t_body = axs.text(0.0, 0.42, "", transform=axs.transAxes, va="top", fontsize=8.5,
                               family="monospace", linespacing=1.5)

        # ---- pose bars ----
        axp = self.ax_p = fig.add_subplot(gr[1])
        axp.set_title("Current pose", loc="left")
        axp.set_xlim(0, 180)
        axp.set_ylim(-0.6, 1.6)
        axp.set_yticks([0, 1])
        axp.set_yticklabels(["down", "up"])
        axp.set_xlabel("degrees")
        axp.grid(alpha=0.3, axis="x")
        for deg in ANGLES:
            axp.axvline(deg, color="0.75", lw=0.8, zorder=0)
        self.p_down = axp.barh([0], [0], height=0.5, color="#2ca02c")
        self.p_up = axp.barh([1], [0], height=0.5, color="#1f77b4")
        self.t_pose = axp.text(0.99, 0.95, "", transform=axp.transAxes, ha="right", va="top",
                               fontsize=8, family="monospace")

        fig.canvas.mpl_connect("key_press_event", self._on_key)

    # -------------------------------------------------------------------- keys
    def _on_key(self, ev):
        if ev.key == " ":
            self.paused = not self.paused
        elif ev.key == "[":
            self.hist_i = max(0, self.hist_i - 1)
        elif ev.key == "]":
            self.hist_i = min(len(self.cfg.history_steps) - 1, self.hist_i + 1)
        elif ev.key == "g":
            self.show_greedy = not self.show_greedy
        elif ev.key == "l":
            self.source.send("table")
        elif ev.key == "s":
            self.source.send("stop")
        elif ev.key == "r":
            self.source.send("learn")      # fresh run: the firmware resets first
        elif ev.key == "c":
            self.source.send("status")     # re-read the experiment configuration

    # -------------------------------------------------------------- data drain
    def _drain(self):
        while True:
            try:
                event = self.q.get_nowait()
            except queue.Empty:
                return
            kind = event[0]
            if kind == "ep":
                ep: Episode = event[1]
                if self.last_ep is not None and ep.n < self.last_ep.n:
                    self.episodes.clear()          # ESP32 rebooted
                self.episodes.append(ep)
                self.last_ep = ep
            elif kind == "config":
                self._apply_config(event[1])
            elif kind == "table_chunk":
                self._accept_chunk(event[1], event[2], event[3])
            elif kind == "status":
                self.status.update(event[1])
            elif kind == "imu":
                self.imu_t.append(event[1])
                self.imu_a.append(event[2])
            elif kind == "event":
                self.events.append(f"{event[1]} {event[2]}".strip())

    # ------------------------------------------------------- discretization
    def _apply_config(self, config: dict):
        """Adopt the firmware's discretization and reward settings."""
        try:
            step = int(config.get("step", self.step_deg))
        except ValueError:
            return
        notches = int(config.get("notches", 180 // step + 1))
        self.status.update(config)

        if step == self.step_deg and notches == self.notches:
            return

        # A table already on screen was sized from real data, which is more
        # trustworthy than a config line. Only resize when nothing has arrived.
        if self._have_table:
            return

        self.step_deg = step
        self.notches = notches
        self.table = np.zeros((self.notches * self.notches, N_ACTIONS))
        self._chunks.clear()
        self._chunk_total = 0
        self._rebuild_table_axes()

    def _accept_chunk(self, chunk: int, chunks: int, values: np.ndarray):
        """Reassemble a chunked Q,TABLE dump.

        The geometry is derived from the number of values received, not from
        Q,CONFIG. That way a viewer attached to a robot whose step was changed
        before the viewer started still draws the right grid: 125 values means
        5 notches, 500 means 10, 1805 means 19.
        """
        if chunks != self._chunk_total:
            self._chunks.clear()
            self._chunk_total = chunks
        self._chunks[chunk] = values
        if len(self._chunks) < chunks:
            return

        flat = np.concatenate([self._chunks[i] for i in range(1, chunks + 1)])
        self._chunks.clear()

        # values = notches^2 * actions  ->  notches = sqrt(values / actions)
        notches = int(round(math.sqrt(flat.size / N_ACTIONS)))
        if notches < 2 or notches * notches * N_ACTIONS != flat.size:
            self.source.last_reject = (
                f"Q,TABLE has {flat.size} values, which is not notches^2*{N_ACTIONS}")
            return

        if notches != self.notches:
            self.notches = notches
            self.step_deg = 180 // (notches - 1) if notches > 1 else 180
            self._rebuild_table_axes()

        self.table = flat.reshape(notches * notches, N_ACTIONS)
        self._have_table = True

    def _rebuild_table_axes(self):
        """Redraw the Q heatmap and policy grid for the current discretization."""
        angles = list(range(0, 181, self.step_deg))
        n = self.notches

        axq = self.ax_q
        axq.clear()
        axq.set_title("Q table   rows = pose (down / up leg angle), columns = action", loc="left")
        self.im = axq.imshow(np.zeros((n * n, N_ACTIONS)), aspect="auto", cmap="RdBu_r",
                             interpolation="nearest")
        axq.set_xticks(range(N_ACTIONS))
        axq.set_xticklabels(ACTIONS)
        # With many notches a label per row is unreadable, so label only the
        # rows where the down angle changes and let the grid carry the rest.
        ticks = [d * n for d in range(n)]
        axq.set_yticks(ticks)
        axq.set_yticklabels([f"{angles[d]}/0" for d in range(n)], fontsize=6)
        axq.set_ylabel("down / up  (deg)")
        # Per-cell numbers only fit while the grid is small.
        show_numbers = n <= 6
        self.q_text = [[axq.text(a, s, "", ha="center", va="center", fontsize=5.6)
                        if show_numbers else None
                        for a in range(N_ACTIONS)] for s in range(n * n)]
        self.cbar.update_normal(self.im)

        axg = self.ax_g
        axg.clear()
        axg.set_title("Greedy policy   (cell = best action in that pose)", loc="left")
        axg.set_xlim(-0.5, n - 0.5)
        axg.set_ylim(-0.5, n - 0.5)
        axg.set_xticks(range(n))
        axg.set_xticklabels(angles, fontsize=6)
        axg.set_yticks(range(n))
        axg.set_yticklabels(angles, fontsize=6)
        axg.set_xlabel("up leg (deg)")
        axg.set_ylabel("down leg (deg)")
        axg.grid(alpha=0.3)
        glyph_size = 9 if n <= 6 else (6 if n <= 10 else 4)
        self.g_text = [[axg.text(u, d, "", ha="center", va="center", fontsize=glyph_size,
                                 fontweight="bold") for u in range(n)] for d in range(n)]
        self.g_cell = [[axg.add_patch(plt.Rectangle((u - 0.5, d - 0.5), 1, 1, fill=False,
                                                    ec="#2a9d8f", lw=2.5, visible=False))
                        for u in range(n)] for d in range(n)]

        # Pose bars: the notch gridlines follow the step.
        axp = self.ax_p
        for line in list(axp.lines):
            line.remove()
        for deg in angles:
            axp.axvline(deg, color="0.75", lw=0.8, zorder=0)

    # ------------------------------------------------------------------ redraw
    def _refresh(self):
        table = self.table
        finite = table[np.isfinite(table)]
        if finite.size and finite.max() > finite.min():
            vmax = float(np.abs(finite).max())
            self.im.set_data(table)
            self.im.set_norm(TwoSlopeNorm(vmin=-vmax, vcenter=0.0, vmax=vmax))
        else:
            self.im.set_data(table)
            self.im.set_norm(TwoSlopeNorm(vmin=-1.0, vcenter=0.0, vmax=1.0))
        self.cbar.update_normal(self.im)

        # Q values as text
        n_states = self.notches * self.notches
        if self.q_text and self.q_text[0][0] is not None:
            for s in range(n_states):
                for a in range(N_ACTIONS):
                    value = table[s, a]
                    self.q_text[s][a].set_text(f"{value:+.2f}" if abs(value) >= 0.005 else "0")

        # greedy policy grid
        best = np.full(n_states, -1)
        for s in range(n_states):
            valid = valid_actions(s // self.notches, s % self.notches, self.notches)
            if valid:
                best[s] = max(valid, key=lambda a: table[s, a])
        for d in range(self.notches):
            for u in range(self.notches):
                s = d * self.notches + u
                a = best[s]
                self.g_text[d][u].set_text(ACTION_GLYPH[ACTIONS[a]] if a >= 0 else "·")
                self.g_text[d][u].set_color("#1f3b73" if a >= 0 else "0.6")
                self.g_cell[d][u].set_visible(False)

        if self.last_ep is not None:
            d = min(self.notches - 1, max(0, self.last_ep.next_down // self.step_deg))
            u = min(self.notches - 1, max(0, self.last_ep.next_up // self.step_deg))
            self.g_cell[d][u].set_visible(True)

        # reward history
        eps = list(self.episodes)[-self.cfg.history_steps[self.hist_i]:]
        if eps:
            n = np.array([e.n for e in eps], dtype=float)
            r = np.array([e.reward for e in eps], dtype=float)
            self.r_pts.set_data(n, r)
            window = min(20, len(r))
            if window >= 2:
                kernel = np.ones(window) / window
                self.r_avg.set_data(n[window - 1:], np.convolve(r, kernel, mode="valid"))
            else:
                self.r_avg.set_data([], [])
            self.ax_r.relim()
            self.ax_r.autoscale_view(scalex=True, scaley=True)

            self.e_line.set_data(n, [e.epsilon for e in eps])
            self.td_line.set_data(n, [abs(e.td) for e in eps])
            self.ax_e.set_xlim(n[0], max(n[-1], n[0] + 1))
            self.ax_td.relim()
            self.ax_td.autoscale_view(scalex=False, scaley=True)

        # live acceleration
        if self.imu_t:
            t = np.array(self.imu_t) / 1000.0          # firmware t_ms -> seconds
            a = np.array(self.imu_a)
            x = t - t[-1]
            self.a_raw.set_data(x, a)
            # Same rule as the firmware: below the deadband counts as zero.
            gated = np.where(np.abs(a) >= self.cfg.deadband_ms2, a, 0.0)
            self.a_line.set_data(x, gated)
            self.ax_a.set_xlim(-self.cfg.accel_window_s, 0)
            lo, hi = float(np.min(a)), float(np.max(a))
            pad = max(0.2, 0.15 * (hi - lo))
            self.ax_a.set_ylim(lo - pad, hi + pad)

        self._refresh_status()

    def _refresh_reward(self):
        ep = self.last_ep
        if ep is None:
            self.t_reward.set_text("-")
            self.t_reward.set_color("0.5")
            self.t_reward_sub.set_text("")
            return

        # Flash for a moment when the value changes, then settle to a steady
        # colour that encodes the sign.
        if ep.n != self._reward_flash_val:
            self._reward_flash_val = ep.n
            self._reward_flash_t = time.monotonic()
        fresh = (time.monotonic() - self._reward_flash_t) < 1.2

        value = ep.reward
        self.t_reward.set_text(f"{value:+.2f}")
        if value > 0:
            colour = "#2a9d8f"
        elif value < 0:
            colour = "#c0392b"
        else:
            colour = "0.45"
        self.t_reward.set_color(colour)
        self.t_reward.set_alpha(1.0 if fresh else 0.75)

        peak = f"{ep.peak:+.2f}" if np.isfinite(ep.peak) else "-"
        active = f"{ep.active:.2f}" if np.isfinite(ep.active) else "-"
        self.t_reward_sub.set_text(f"peak {peak} m/s²   active {active}")

    def _refresh_status(self):
        self._refresh_reward()
        ep = self.last_ep
        running = self.status.get("running", "?")
        learn = self.status.get("learn", "?")
        badge = "PAUSED" if self.paused else ("LEARNING" if learn == "1" else "RUNNING")
        self.t_badge.set_text(f"{badge}   {self.mode}")
        self.t_badge.set_color("#7b3fa0" if self.paused else ("#2a9d8f" if learn == "1" else "#e08a00"))

        lines = [
            f"episode  {ep.n if ep else 0}",
            f"epsilon  {ep.epsilon:.3f}" if ep else "epsilon  -",
            f"eps decay {self.status.get('eps_decay', '-')}",
            f"action   {ep.action}" if ep else "action   -",
            f"|TD|     {abs(ep.td):.3f}" if ep else "|TD|     -",
            f"total    {self.status.get('total_reward', '-')}",
            f"source   {self.source.state}",
            "",
            "r learn  s stop  l table  c config",
        ]
        if self.source.rejected:
            lines.append(f"rejected    {self.source.rejected}  ({self.source.last_reject[:34]})")
        if self.events:
            lines.append("")
            lines.extend(f"  {e}" for e in list(self.events)[-4:])
        self.t_body.set_text("\n".join(lines))

        down = ep.next_down if ep else 0
        up = ep.next_up if ep else 0
        self.p_down[0].set_width(down)
        self.p_up[0].set_width(up)
        self.t_pose.set_text(f"down {down}°   up {up}°")

    # -------------------------------------------------------------------- loop
    def tick(self):
        self._drain()
        if not self.paused:
            self._refresh()
        self.fig.canvas.draw_idle()
        self.frames += 1

    def _tick_safe(self):
        try:
            self.tick()
        except Exception:
            traceback.print_exc()

    def run(self):
        timer = self.fig.canvas.new_timer(interval=int(1000 / max(self.cfg.fps, 1)))
        timer.add_callback(self._tick_safe)
        timer.start()
        try:
            plt.show()
        finally:
            timer.stop()
            self.source.stop()
            if self.logger:
                self.logger.close()


# =============================================================================
# 5. MAIN
# =============================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("port_pos", nargs="?", metavar="PORT", help="serial port (same as --port)")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int)
    ap.add_argument("--demo", action="store_true", help="fake robot, no hardware needed")
    ap.add_argument("--replay", metavar="CSV", help="replay a log written with --log")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")
    ap.add_argument("--log", action="store_true")
    ap.add_argument("--fps", type=int)
    ap.add_argument("--selftest", type=int, metavar="N", default=0,
                    help="run N frames headless and exit (CI smoke test)")
    a = ap.parse_args()

    cfg = Config()
    if a.port_pos:
        cfg.port = a.port_pos
    if a.port:
        cfg.port = a.port
    if a.baud:
        cfg.baud = a.baud
    if a.log:
        cfg.log = True
    if a.fps:
        cfg.fps = a.fps
    cfg.frames = a.selftest
    if cfg.frames:
        import matplotlib
        matplotlib.use("Agg")          # no window in CI

    q, parser = queue.SimpleQueue(), QParser()
    if a.demo:
        src, mode = DemoSource(q, parser, cfg), "DEMO"
    elif a.replay:
        src, mode = ReplaySource(q, parser, cfg, a.replay, a.speed), "REPLAY"
    else:
        src, mode = SerialSource(q, parser, cfg), "LIVE"

    logger = Logger(cfg.log_dir) if (cfg.log and mode != "REPLAY") else None
    src.logger = logger
    app = App(cfg, src, q, logger, mode)
    src.start()

    if cfg.frames:
        # Headless smoke test: drive the same tick path the GUI timer uses.
        for _ in range(cfg.frames):
            app._tick_safe()
            time.sleep(0.02)
        src.stop()
        if logger:
            logger.close()
        print(f"selftest ok: {app.frames} frames, {len(app.episodes)} episodes, "
              f"table max {app.table.max():+.3f}")
        return 0

    app.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
