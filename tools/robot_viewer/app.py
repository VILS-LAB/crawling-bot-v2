"""The window: charts on the left, instruments and the command console on the right."""

from __future__ import annotations

import math
import queue
import time
import traceback
from typing import Optional

import numpy as np

from .console import (
    BUTTON_COLORS,
    CAL_STAGES,
    ActionResult,
    CommandConsole,
    action_set,
    stage_index,
)
from .drawing import (
    ICON_BODY,
    ICON_HEAD,
    ICON_NEEDLE,
    ICON_NOSE,
    ICON_RIGHT,
    Blitter,
    box_faces,
    compass_heading,
    nice_ceil,
    rot2,
    rot_matrix,
    yaw_unwrap_deg,
)
from .health import FAIL, OK, UNKNOWN, WARN, HealthReport
from .panels import PANELS, STILL_CHECK_NAMES, X_C, Y_C, Z_C
from .telemetry import CH, LineParser, Ring

STATE_COLORS = {OK: "#2ca02c", WARN: "#d98200", FAIL: "#d62728", UNKNOWN: "0.45"}


def _fmt(value, spec: str, unit: str) -> str:
    """Format a possibly-NaN value; older firmware may not send every column."""
    return "n/a" if value is None or not np.isfinite(value) else f"{value:{spec}} {unit}"


class App:
    def __init__(self, cfg, source, out_queue, logger=None, mode: str = "LIVE") -> None:
        self.cfg = cfg
        self.source = source
        self.queue = out_queue
        self.logger = logger
        self.mode = mode

        capacity = int(
            max(cfg.buffer_s, max(cfg.window_steps), cfg.trail_s) * cfg.max_rate_hz
        )
        self.ring = Ring(capacity)
        self.health = HealthReport()
        self.console = CommandConsole(self._send_command)
        self.console._timeout = cfg.command_timeout_s

        self.window_index = cfg.window_index
        self.preset = cfg.preset
        self.paused = False
        self.last_row: Optional[np.ndarray] = None
        self.receiving_since: Optional[float] = None

        self.zero = np.zeros(2)
        self.zero_t = -math.inf
        self.zeroed = False
        self.trail = None

        # Instrument feedback
        self.last_act_reply = ""
        self.frames_drawn = 0
        self.draw_fps = 0.0
        self._fps_mark = (time.monotonic(), 0)
        self._rx_mark = (time.monotonic(), 0)
        self.rx_rate = 0.0
        self._last_slow = 0.0
        self._need_full = False
        self._pending = {"d": False, "u": False}
        self._last_tx = 0.0

        # Auto-stream bookkeeping. A TCP client starts with streamHz = 0 on the
        # robot ("every TCP client starts quiet"), and serial runs whatever the
        # last session set, so the viewer has to ask before it can plot anything.
        self._want_stream_at = time.monotonic() + cfg.stream_grace_s
        self._stream_asked_at = 0.0
        self._stream_asks = 0
        self._stream_settled = False
        self._adopted_hz = 0

        self._closed = False
        self.buttons = {}
        self._build()

    # ------------------------------------------------------------------ input
    def _send_command(self, command: str) -> bool:
        return self.source.send(command)

    # ------------------------------------------------------------------ build
    def _build(self) -> None:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Polygon
        from matplotlib.widgets import Button, Slider, TextBox
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        from matplotlib.colors import to_rgba

        cfg = self.cfg
        plt.rcParams.update(
            {"font.size": 8, "axes.titlesize": 9.5, "axes.titleweight": "bold"}
        )
        self.fig = fig = plt.figure(figsize=cfg.figsize)
        self.plt = plt
        try:
            fig.canvas.manager.set_window_title("Crawling Bot V2 - robot visualizer")
        except Exception:
            pass

        # Left: the six consolidated charts. Right: instruments + console.
        left = fig.add_gridspec(
            4, 2, left=0.045, right=0.605, top=0.945, bottom=0.062,
            hspace=0.42, wspace=0.15,
            height_ratios=[1.0, 1.0, 0.95, 0.75],
        )
        self.blit = Blitter(fig.canvas, cfg.use_blit)

        self.plots = []
        for panel in PANELS:
            row, col = panel.pos
            ax = fig.add_subplot(left[row, col : col + panel.colspan])
            self._style_panel(ax, panel, is_bottom=row == 4)
            ax_right = ax.twinx() if panel.right_label else None
            if ax_right is not None:
                ax_right.set_ylabel(panel.right_label, color="0.35")
                if panel.right_ylim:
                    ax_right.set_ylim(*panel.right_ylim)
                ax_right.tick_params(labelsize=7, colors="0.35")

            artists = []
            for line in panel.lines:
                target = ax_right if line.axis == "right" and ax_right is not None else ax
                (artist,) = target.plot(
                    [],
                    [],
                    color=line.color,
                    alpha=line.alpha,
                    lw=line.lw,
                    ls=line.ls,
                    label=line.label,
                    drawstyle="steps-post" if line.step else "default",
                    solid_capstyle="butt",
                )
                artists.append((line, artist))
                self.blit.add(artist)

            if any(not line.label.startswith("_") for line in panel.lines):
                ax.legend(
                    loc="upper left",
                    ncol=panel.legend_cols,
                    fontsize=6.6,
                    framealpha=0.72,
                    borderaxespad=0.2,
                    handlelength=1.3,
                    # Multi-column legends need real gaps between columns,
                    # otherwise "roll drift" reads as part of the next entry.
                    columnspacing=1.6 if panel.legend_cols > 1 else 0.9,
                    handletextpad=0.5,
                )
            clipped = ax.text(
                0.995, 0.03, "", transform=ax.transAxes, ha="right", va="bottom",
                color="#d62728", fontsize=7.5, fontweight="bold",
            )
            self.blit.add(clipped)
            self.plots.append((panel, ax, ax_right, artists, clipped))

        # The path is square, so it gets a fixed slot rather than a grid cell.
        self._build_topdown(fig.add_axes([0.045, 0.055, 0.175, 0.142]))

        # ---- right column -------------------------------------------------
        right = fig.add_gridspec(
            4, 1, left=0.635, right=0.985, top=0.945, bottom=0.305,
            height_ratios=[1.22, 1.02, 1.18, 0.86], hspace=0.52,
        )
        self._build_robot_3d(fig.add_subplot(right[0], projection="3d"), Poly3DCollection, to_rgba)
        self._build_dial(fig.add_subplot(right[1]), Circle, Polygon)
        self._build_health(fig.add_subplot(right[2]))
        self._build_summary(fig.add_subplot(right[3]))

        self._build_console(fig, Slider, Button, TextBox)

        fig.text(
            0.045, 0.012,
            "space = pause   [ ] = window   m = range preset   z = zero position   "
            "|  buttons below send real commands",
            fontsize=7.5, color="0.4",
        )

        fig.canvas.mpl_connect("key_press_event", self._on_key)
        fig.canvas.mpl_connect("close_event", lambda _e: self.shutdown())
        self._apply_ranges()
        self._apply_window()

    def _style_panel(self, ax, panel, is_bottom: bool) -> None:
        ax.set_title(panel.title, loc="left")
        ax.set_ylabel(panel.ylabel)
        ax.grid(alpha=0.28)
        ax.axhline(0, color="k", lw=0.55, alpha=0.3)
        ax.tick_params(labelsize=7)
        if is_bottom:
            ax.set_xlabel("seconds ago  (0 = now)", fontsize=7.5)
        else:
            ax.tick_params(labelbottom=False)
        if panel.yticks:
            ax.set_yticks([position for position, _label in panel.yticks])
            ax.set_yticklabels([label for _position, label in panel.yticks], fontsize=6.8)
        if panel.ylim:
            ax.set_ylim(*panel.ylim)

    # ------------------------------------------------------------ 3D + dial
    def _build_robot_3d(self, ax, Poly3DCollection, to_rgba) -> None:
        cfg = self.cfg
        self.ax3d = ax
        ax.set_title("Robot orientation   (N up, E right, teal = front)",
                     fontsize=9.2, fontweight="bold", pad=0)
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_zlim(-1, 1)
        ax.set_box_aspect((1, 1, 0.75))
        ax.dist = 9
        ax.set_xlabel("E", fontsize=7, labelpad=-8)
        ax.set_ylabel("N", fontsize=7, labelpad=-8)
        ax.set_zlabel("U", fontsize=7, labelpad=-8)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.set_ticks([-1.0, 0.0, 1.0])
        ax.tick_params(labelsize=5.5, pad=-2)
        ax.view_init(elev=22, azim=-58)

        base, top, front = "#e8873a", "#f4b66f", "#2a9d8f"
        colors = [to_rgba(c, 0.9) for c in (base, base, base, front, base, top)]
        self.poly = Poly3DCollection(
            box_faces(np.eye(3), cfg.box_size), facecolors=colors,
            edgecolors="k", linewidths=0.7,
        )
        ax.add_collection3d(self.poly)
        self.axis_lines = [
            ax.plot([0, 0], [0, 0], [0, 0], color=color, lw=2.5)[0]
            for color in (X_C, Y_C, Z_C)
        ]
        self.t_rpy = ax.text2D(
            0.0, 1.0, "", transform=ax.transAxes, fontsize=8.2,
            fontweight="bold", family="monospace", va="top",
        )
        self.blit.add(self.poly, *self.axis_lines, self.t_rpy)

    def _build_dial(self, ax, Circle, Polygon) -> None:
        ax.set_aspect("equal")
        ax.set_xlim(-1.66, 1.66)
        ax.set_ylim(-2.58, 1.46)
        ax.axis("off")
        ax.set_title("Heading   0° = north, clockwise", fontsize=9.2,
                     fontweight="bold", pad=2)
        ax.add_patch(Circle((0, 0), 1.0, fill=False, ec="0.35", lw=1.2))
        for degrees in range(0, 360, 10):
            theta = math.radians(degrees)
            inner = 0.86 if degrees % 90 == 0 else (0.92 if degrees % 30 == 0 else 0.96)
            ax.plot(
                [inner * math.sin(theta), math.sin(theta)],
                [inner * math.cos(theta), math.cos(theta)],
                color="0.35",
                lw=1.4 if degrees % 90 == 0 else 0.7,
            )
            if degrees % 90 == 0:
                cardinal = {0: "N", 90: "E", 180: "S", 270: "W"}[degrees]
                ax.text(
                    1.3 * math.sin(theta), 1.3 * math.cos(theta),
                    f"{cardinal}\n{degrees}°", fontsize=6.5, ha="center", va="center",
                    color="0.25", fontweight="bold", linespacing=0.95,
                )
            elif degrees % 30 == 0:
                ax.text(
                    1.15 * math.sin(theta), 1.15 * math.cos(theta), f"{degrees}",
                    fontsize=6, ha="center", va="center", color="0.45",
                )

        (self.dial_arc,) = ax.plot([], [], color="#7b3fa0", lw=4.5, alpha=0.3)
        self.dial_body = ax.add_patch(Polygon(ICON_BODY, closed=True, fc="#e8873a", ec="k", lw=1.0, zorder=3))
        self.dial_nose = ax.add_patch(Polygon(ICON_NOSE, closed=True, fc=Y_C, ec="k", lw=1.0, zorder=4))
        (self.dial_right,) = ax.plot([], [], color=X_C, lw=4.0, solid_capstyle="butt", zorder=5)
        (self.dial_needle,) = ax.plot([], [], color="#2a9d8f", lw=2.2, zorder=3)
        self.dial_head = ax.add_patch(Polygon(ICON_HEAD, closed=True, fc="#2a9d8f", ec="#2a9d8f", zorder=3))
        self.t_head_big = ax.text(0.0, -1.95, "", ha="center", va="center",
                                  fontsize=14, fontweight="bold")
        # One combined line: the raw yaw and the turn rate read together.
        self.t_turn = ax.text(0.0, -2.36, "", ha="center", va="center",
                              fontsize=7.4, fontweight="bold", family="monospace")
        self.blit.add(
            self.dial_arc, self.dial_body, self.dial_nose, self.dial_right,
            self.dial_needle, self.dial_head, self.t_head_big, self.t_turn,
        )

    def _build_topdown(self, ax) -> None:
        """Dead-reckoned path, drawn on a north-up map."""
        ax.set_title("Top-down path (dead reckoning, relative to the zero point)",
                     loc="left")
        ax.set_xlabel("Δ east (m)", fontsize=7)
        ax.set_ylabel("Δ north (m)", fontsize=7)
        ax.grid(alpha=0.28)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(-0.5, 0.5)
        ax.tick_params(labelsize=7)
        ax.plot([0], [0], "o", color="#2ca02c", ms=6, label="zero")
        (self.td_trail,) = ax.plot([], [], color="#8c5bbf", lw=1.8, label="path")
        (self.td_now,) = ax.plot([], [], "x", color="#d62728", ms=8, mew=2, label="now")
        (self.td_head,) = ax.plot([], [], color="#2a9d8f", lw=2.5,
                                  solid_capstyle="round", label="heading")
        # The panel keeps an equal aspect, so it is much narrower than its grid
        # slot. Put the legend in the horizontal slack beside it: inside the
        # axes it would cover the newest part of the trail, and above the axes
        # it would collide with the title.
        ax.legend(loc="center left", bbox_to_anchor=(1.03, 0.5), ncol=1,
                  fontsize=6.5, frameon=False, handlelength=1.1,
                  handletextpad=0.5, labelspacing=0.6, borderaxespad=0.0)
        self.ax_td = ax
        self.blit.add(self.td_trail, self.td_now, self.td_head)

    def _build_health(self, ax) -> None:
        ax.axis("off")
        ax.set_title("Health checks", loc="left", fontsize=9.5)
        self.t_verdict = ax.text(
            0.0, 0.99, "", transform=ax.transAxes, va="top",
            fontsize=10.5, fontweight="bold", family="monospace",
        )
        # The headline is one line of axis height ~0.11 for this (short) axes, so
        # the rows start below it and are packed into what is left. Spacing is
        # derived from the row count, so adding a check cannot push a row under
        # the headline.
        self.health_rows = []
        keys = list(self.health.checks)
        # Centered text needs half a line of margin at both ends or the first
        # and last rows get clipped by the axes edge.
        first_y, last_y = 0.805, 0.055
        step = (first_y - last_y) / max(len(keys) - 1, 1)
        for index, key in enumerate(keys):
            y = first_y - index * step
            icon = ax.text(0.0, y, "", transform=ax.transAxes, va="center",
                           fontsize=10, fontweight="bold")
            label = ax.text(0.045, y, "", transform=ax.transAxes, va="center",
                            fontsize=7.6, fontweight="bold")
            detail = ax.text(0.30, y, "", transform=ax.transAxes, va="center",
                             fontsize=7.2, family="monospace", color="0.35")
            self.health_rows.append((key, icon, label, detail))
        self.blit.add(self.t_verdict, *[a for _k, i, l, d in self.health_rows for a in (i, l, d)])

    def _build_summary(self, ax) -> None:
        ax.axis("off")
        ax.set_title("Live state", loc="left", fontsize=9.5)
        self.t_summary = ax.text(
            0.0, 0.96, "", transform=ax.transAxes, va="top",
            fontsize=7.6, family="monospace", linespacing=1.45,
        )
        self.t_stream = self.fig.text(
            0.635, 0.296, "", fontsize=7.2, family="monospace", color="0.4",
            va="center",
        )
        self.blit.add(self.t_summary, self.t_stream)

    # ---------------------------------------------------------------- console
    def _build_console(self, fig, Slider, Button, TextBox) -> None:
        cfg = self.cfg
        fig.text(0.635, 0.272, "COMMAND CONSOLE", fontsize=8.8, fontweight="bold", color="0.25")

        # legs
        self.sl_down = Slider(
            fig.add_axes([0.695, 0.238, 0.235, 0.021]), "down leg", 0, 180,
            valinit=90, valstep=1, valfmt="%d°", color=Y_C,
        )
        self.sl_up = Slider(
            fig.add_axes([0.695, 0.209, 0.235, 0.021]), "up leg", 0, 180,
            valinit=90, valstep=1, valfmt="%d°", color=Z_C,
        )
        self.sl_down.on_changed(lambda _v: self._mark("d"))
        self.sl_up.on_changed(lambda _v: self._mark("u"))

        self.legs_text = fig.text(0.637, 0.243, "LEGS", fontsize=7.6, color="0.4", va="center")

        # SETID entry
        self.tb_setid = TextBox(
            fig.add_axes([0.800, 0.176, 0.048, 0.023]), "SETID", initial="1",
            textalignment="center",
        )
        self.tb_setid.label.set_fontsize(7.6)

        # Action buttons: 5 per row. ACT reads the sliders at click time.
        actions = action_set(lambda: (self.sl_down.val, self.sl_up.val))
        self._actions = {action.key: action for action in actions}
        specs = [
            ("ping", "PING"), ("id", "ID"), ("get", "GET"), ("setid", "SETID"),
            ("stream_on", "STREAM 50"), ("stream_off", "STREAM 0"),
            ("act", "ACT"), ("stop", "STOP"), ("park", "PARK"), ("cal", "CAL all"),
        ]
        left, top, width, height, gap = 0.637, 0.150, 0.0655, 0.029, 0.0045
        self._button_specs = {}
        for index, (key, label) in enumerate(specs):
            row, col = divmod(index, 5)
            x = left + col * (width + gap)
            y = top - row * (height + 0.007)
            button = Button(fig.add_axes([x, y, width, height]), label,
                            color=BUTTON_COLORS["idle"], hovercolor="0.82")
            button.label.set_fontsize(6.9)
            button.on_clicked(lambda _event, k=key: self._on_action(k))
            self.buttons[key] = button
            self._button_specs[key] = label

        self.t_reply = fig.text(
            0.637, 0.092, "", fontsize=7.3, family="monospace", color="0.28",
            va="top", wrap=True,
        )
        self.t_history = fig.text(
            0.637, 0.058, "", fontsize=6.6, family="monospace", color="0.45",
            va="top", linespacing=1.3,
        )
        self.blit.add(self.t_reply, self.t_history, self.legs_text)

    # ----------------------------------------------------------------- events
    def _on_action(self, key: str) -> None:
        action = self._actions[key]
        if key == "setid":
            text = self.tb_setid.text.strip()
            if not text.isdigit() or not 1 <= int(text) <= 8:
                self._flash(key, "err")
                self._set_reply("SETID needs a number 1-8")
                return
            command = action.build(text)
        else:
            command = action.build()
        if command is None:
            self._set_reply("that command is not available yet")
            return
        sent = self.console.run(action, "" if key != "setid" else self.tb_setid.text)
        if sent is None:
            self._flash(key, "err")
            self._set_reply("another command is still waiting for a reply")
            return
        self._flash(key, "busy")
        self._set_reply(f"> {command}")
        if key == "cal":
            self.health.set("calibration", WARN, "calibration in progress",
                            "follow the OLED prompts")
        self._need_full = True

    def _flash(self, key: str, state: str) -> None:
        button = self.buttons.get(key)
        if button is None:
            return
        try:
            button.color = BUTTON_COLORS[state]
            button.ax.set_facecolor(BUTTON_COLORS[state])
        except Exception:
            pass
        self._need_full = True

    def _set_reply(self, text: str) -> None:
        self.t_reply.set_text(text)
        self._need_full = True

    def _on_key(self, event) -> None:
        if event.key == " ":
            self.paused = not self.paused
        elif event.key in ("[", "]"):
            delta = -1 if event.key == "[" else 1
            self.window_index = min(max(self.window_index + delta, 0), len(self.cfg.window_steps) - 1)
            self._apply_window()
        elif event.key == "m":
            self.preset = self.cfg.presets.next_name(self.preset)
            self._apply_ranges()
        elif event.key == "z" and self.last_row is not None:
            self.zero = np.array([self.last_row[CH["px"]], self.last_row[CH["py"]]])
            self.zero_t = float(self.last_row[CH["t"]])
            self.zeroed = True

    # ------------------------------------------------------------- leg sliders
    def _mark(self, which: str) -> None:
        self._pending[which] = True

    def _set_sliders(self, down: int, up: int) -> None:
        for slider, value in ((self.sl_down, down), (self.sl_up, up)):
            slider.eventson = False
            slider.set_val(value)
            slider.eventson = True

    def _flush_sliders(self, now: float) -> None:
        if not any(self._pending.values()) or now - self._last_tx < 1.0 / self.cfg.send_hz:
            return
        down, up = int(self.sl_down.val), int(self.sl_up.val)
        self._pending = {"d": False, "u": False}
        self._last_tx = now
        self.console.run(self._actions["act"])

    # ------------------------------------------------------------------ data
    def _drain(self) -> None:
        rows = []
        while True:
            try:
                item = self.queue.get_nowait()
            except queue.Empty:
                break
            if isinstance(item, str):
                self._handle_text(item)
            else:
                rows.append(item)
        if not rows:
            return

        for row in rows:
            last = self.ring.last_time()
            if last is not None and row[CH["t"]] < last - 1.0:
                # The robot rebooted: its clock restarted.
                self.ring.clear()
                self.zero, self.zero_t, self.zeroed = np.zeros(2), -math.inf, False
            self.ring.push(row)
            if (
                self.cfg.auto_zero_on_connect
                and not self.zeroed
                and np.isfinite(row[[CH["px"], CH["py"]]]).all()
            ):
                self.zero = row[[CH["px"], CH["py"]]].copy()
                self.zero_t, self.zeroed = float(row[CH["t"]]), True
            self.health.feed_row(row)
            if self.logger is not None:
                self.logger.write_row(row)

        self.health.mark_data_seen(len(rows))
        self.health.set("probe", OK, "receiving telemetry")
        self.receiving_since = self.receiving_since or time.monotonic()

    def _handle_text(self, line: str) -> None:
        self.health.feed_line(line)

        result = self.console.feed_cal_result(line)
        if result is None:
            result = self.console.feed_line(line)
        if result is not None:
            self._on_result(result)
            return

        stage = stage_index(line)
        if stage is not None:
            _index, name, hint = CAL_STAGES[stage]
            self.health.set("calibration", WARN, f"CAL stage {stage + 1}/3: {name}", hint)

    def _on_result(self, result: ActionResult) -> None:
        verb = result.command.split()[0].upper() if result.command.split() else ""
        self.health.feed_reply(result.command, result.reply)
        key = {
            "PING": "ping", "ID": "id", "GET": "get", "SETID": "setid",
            "ACT": "act", "STOP": "stop", "PARK": "park", "CAL": "cal",
            "STREAM": "stream_on" if "50" in result.command else "stream_off",
        }.get(verb)
        if key:
            self._flash(key, "ok" if result.ok else "err")

        self._set_reply(f"> {result.command}\n{result.reply}")
        recent = self.console.history[-5:]
        self.t_history.set_text(
            "\n".join(
                f"{'✓' if item.ok else '✗'} {item.command[:22]:22} {item.reply[:34]}"
                for item in recent
            )
        )

        if verb == "ACT" and result.ok:
            parts = dict(
                token.split("=", 1) for token in result.reply.replace("OK ACT", "").split() if "=" in token
            )
            self.last_act_reply = result.reply
            if "d" in parts and "u" in parts:
                self._set_sliders(int(parts["d"]), int(parts["u"]))
        self._need_full = True

    # -------------------------------------------------------------- refresh
    def _apply_ranges(self) -> None:
        preset = self.cfg.presets.get(self.preset)
        for panel, ax, ax_right, _artists, _clip in self.plots:
            if panel.ylim is not None:
                ax.set_ylim(*panel.ylim)
            elif preset is None:
                ax.set_autoscaley_on(True)
            elif panel.range_key in preset:
                limit = preset[panel.range_key]
                ax.set_ylim(-limit, limit)
        self._need_full = True

    def _apply_window(self) -> None:
        width = self.cfg.window_steps[self.window_index]
        for _panel, ax, _right, _artists, _clip in self.plots:
            ax.set_xlim(-width, 0)
        self._need_full = True

    def _refresh(self) -> None:
        last_time = self.ring.last_time()
        if last_time is None:
            return
        window = self.cfg.window_steps[self.window_index]
        start = last_time - window
        view = self.ring.window(start)
        if view is None or view.shape[1] < 2:
            return
        self.last_row = view[:, -1].copy()

        # One decimation pass for every channel any panel draws.
        used = sorted({CH[line.field] for panel in PANELS for line in panel.lines})
        t_out, values = self.ring.decimate(self.cfg.max_points, used)
        if values is None:
            return
        index = {channel_index: position for position, channel_index in enumerate(used)}
        x = t_out

        preset = self.cfg.presets.get(self.preset)
        for panel, ax, ax_right, artists, clipped in self.plots:
            low_seen, high_seen = math.inf, -math.inf
            for line, artist in artists:
                series = values[index[CH[line.field]]]
                if line.scale != 1.0 or line.offset != 0.0:
                    series = series * line.scale + line.offset
                artist.set_data(x, series)
                if preset is not None and panel.ylim is None:
                    finite = series[np.isfinite(series)]
                    if finite.size:
                        low_seen = min(low_seen, float(finite.min()))
                        high_seen = max(high_seen, float(finite.max()))
            if preset is not None and panel.ylim is None:
                low, high = ax.get_ylim()
                if high_seen > high or low_seen < low:
                    clipped.set_text(f"peak {max(abs(high_seen), abs(low_seen)):.3g} → press m")
                    clipped.set_visible(True)
                else:
                    clipped.set_visible(False)
            else:
                clipped.set_visible(False)

        self._unwrap_yaw()
        row = self.last_row
        self._refresh_scene(row)
        self._refresh_trail(row, last_time)

    def _unwrap_yaw(self) -> None:
        """Keep the newest buffer of yaw continuous across the ±180 wrap."""
        if self.ring.n < 2:
            return
        index = self.ring._ordered_index()
        yaw = self.ring.buf[CH["yaw"], index]
        if not np.isfinite(yaw).all():
            return
        # Only the newly pushed samples can extend a wrap, so fix the tail.
        tail = min(len(index), 400)
        fixed = yaw_unwrap_deg(yaw[-tail:])
        self.ring.buf[CH["yaw"], index[-tail:]] = fixed

    def _refresh_scene(self, row: np.ndarray) -> None:
        cfg = self.cfg
        if not np.isfinite(row[[CH["roll"], CH["pitch"], CH["yaw"]]]).all():
            return
        rotation = rot_matrix(
            row[CH["roll"]], row[CH["pitch"]], row[CH["yaw"]], cfg.euler_signs
        )
        self.poly.set_verts(box_faces(rotation, cfg.box_size))
        for axis_index, line in enumerate(self.axis_lines):
            direction = rotation[:, axis_index] * 1.15
            line.set_data_3d([0, direction[0]], [0, direction[1]], [0, direction[2]])
        self.t_rpy.set_text(
            f"R {row[CH['roll']]:+6.1f}°  P {row[CH['pitch']]:+6.1f}°  Y {row[CH['yaw']]:+6.1f}°"
        )

        yaw = row[CH["yaw"]] * cfg.euler_signs[2]
        heading = compass_heading(yaw, cfg.heading_offset_deg)
        psi = -math.radians(heading)
        self.dial_body.set_xy(rot2(ICON_BODY, psi))
        self.dial_nose.set_xy(rot2(ICON_NOSE, psi))
        self.dial_head.set_xy(rot2(ICON_HEAD, psi))
        for line, shape in ((self.dial_right, ICON_RIGHT), (self.dial_needle, ICON_NEEDLE)):
            points = rot2(shape, psi)
            line.set_data(points[:, 0], points[:, 1])
        arc = np.radians(np.linspace(0.0, heading, max(2, int(heading / 3) + 2)))
        self.dial_arc.set_data(0.93 * np.sin(arc), 0.93 * np.cos(arc))
        self.t_head_big.set_text(f"{heading:5.1f}°")

        rate = row[CH["fgz"]] * cfg.euler_signs[2]
        if np.isfinite(rate) and rate > 2.0:
            turning, color = f"↺ LEFT {rate:.0f}°/s", "#d98200"
        elif np.isfinite(rate) and rate < -2.0:
            turning, color = f"↻ RIGHT {-rate:.0f}°/s", "#d98200"
        else:
            turning, color = "not turning", "0.45"
        self.t_turn.set_text(f"yaw {row[CH['yaw']]:+6.1f}°   {turning}")
        self.t_turn.set_color(color)

    def _refresh_trail(self, row: np.ndarray, last_time: float) -> None:
        trail = self.ring.window(max(last_time - self.cfg.trail_s, self.zero_t),
                                 rows=[CH["px"], CH["py"]])
        if trail is None or trail.shape[1] < 1:
            return
        step = max(1, math.ceil(trail.shape[1] / 1500))
        index = np.unique(np.r_[np.arange(0, trail.shape[1], step), trail.shape[1] - 1])
        north = trail[0, index] - self.zero[0]
        west = trail[1, index] - self.zero[1]
        east, up_north = -west, north
        ok = np.isfinite(east) & np.isfinite(up_north)
        if not ok.any():
            return
        east, up_north = east[ok], up_north[ok]
        self.trail = (east, up_north)
        self.td_trail.set_data(east, up_north)
        self.td_now.set_data([east[-1]], [up_north[-1]])
        if self.last_row is not None and np.isfinite(
            self.last_row[[CH["roll"], CH["pitch"], CH["yaw"]]]
        ).all():
            rotation = rot_matrix(
                self.last_row[CH["roll"]], self.last_row[CH["pitch"]],
                self.last_row[CH["yaw"]], self.cfg.euler_signs,
            )
            nose = rotation @ np.array([0.0, 1.0, 0.0])
            length = math.hypot(nose[0], nose[1])
            if length > 1e-6:
                span = self.ax_td.get_xlim()[1] - self.ax_td.get_xlim()[0]
                self.td_head.set_data(
                    [east[-1], east[-1] + nose[0] / length * 0.12 * span],
                    [up_north[-1], up_north[-1] + nose[1] / length * 0.12 * span],
                )
        self._fit_topdown()

    def _fit_topdown(self) -> None:
        if self.trail is None:
            return
        east, north = self.trail
        x0, x1 = float(east.min()), float(east.max())
        y0, y1 = float(north.min()), float(north.max())
        half = max(self.cfg.topdown_min_half_span_m, 0.6 * max(x1 - x0, y1 - y0))
        (lx0, lx1), (ly0, ly1) = self.ax_td.get_xlim(), self.ax_td.get_ylim()
        margin = 0.05 * (lx1 - lx0)
        span = lx1 - lx0
        if (
            x0 > lx0 + margin and x1 < lx1 - margin
            and y0 > ly0 + margin and y1 < ly1 - margin
            and span < 5 * half
        ):
            return
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        self.ax_td.set_xlim(cx - half, cx + half)
        self.ax_td.set_ylim(cy - half, cy + half)
        self._need_full = True

    # ------------------------------------------------------------- status text
    def _update_status(self, now: float) -> None:
        icon, summary = self.health.verdict
        state = FAIL if icon == "✗" else (WARN if icon == "!" else OK)
        self.t_verdict.set_text(f"{icon}  {summary}")
        self.t_verdict.set_color(STATE_COLORS[state])

        for key, icon_artist, label_artist, detail_artist in self.health_rows:
            check = self.health[key]
            icon_artist.set_text(check.icon)
            icon_artist.set_color(STATE_COLORS[check.state])
            label_artist.set_text(check.label)
            label_artist.set_color("0.15" if check.state != UNKNOWN else "0.55")
            detail_artist.set_text(check.detail[:46])

        row = self.last_row
        if row is None:
            self.t_summary.set_text("waiting for telemetry ...")
        else:
            dn = row[CH["px"]] - self.zero[0]
            dw = row[CH["py"]] - self.zero[1]
            east, north = -dw, dn
            mask = int(row[CH["still_failure_mask"]]) if np.isfinite(row[CH["still_failure_mask"]]) else 0
            failing = ", ".join(
                STILL_CHECK_NAMES.get(bit, f"bit{bit}")
                for bit in STILL_CHECK_NAMES
                if (mask >> bit) & 1
            ) or "-"
            dt = row[CH["dt"]]
            loop_hz = 1.0 / dt if np.isfinite(dt) and dt > 0 else float("nan")
            bias = row[CH["bias_norm"]]
            acc_rms = row[CH["accel_rms_g"]]
            mag_rms = row[CH["mag_rms_ut"]]
            self.t_summary.set_text(
                f"imu {loop_hz:4.0f} Hz   rx {self.rx_rate:4.0f} Hz   draw {self.draw_fps:3.0f} fps\n"
                f"temp {row[CH['temp_c']]:5.1f} °C   trust {row[CH['trust']]:.2f}   "
                f"|a| {row[CH['a_norm']]:.2f} g   |v| {row[CH['speed']]:.2f} m/s\n"
                f"moving {'YES' if row[CH['moving']] >= 0.5 else 'no '}   "
                f"still {'YES' if row[CH['still']] >= 0.5 else 'no '}   "
                f"fail: {failing}\n"
                f"pos  ΔE {east:+8.3f}  ΔN {north:+8.3f} m   "
                f"dist {math.hypot(east, north):.3f} m\n"
                f"yaw {row[CH['yaw']]:+7.1f}°   "
                f"bias {_fmt(bias, '.3f', 'deg/s')}   "
                f"acc_rms {_fmt(acc_rms, '.4f', 'g')}   mag_rms {_fmt(mag_rms, '.2f', 'µT')}"
            )
        age = None if self.source.last_rx_at is None else now - self.source.last_rx_at
        status = "PAUSED" if self.paused else (
            "NO DATA" if age is None or age > 1.5 else self.mode
        )
        self.t_stream.set_text(
            f"{status}   {self.source.state[:52]}   window {self.cfg.window_steps[self.window_index]}s   "
            f"{self.preset}"
        )
        self.t_stream.set_color("#d62728" if status == "NO DATA" else "0.4")

    # ------------------------------------------------------------------- loop
    def tick(self) -> None:
        now = time.monotonic()
        self._drain()
        self._ensure_stream(now)
        self._flush_sliders(now)

        timeout = self.console.poll_timeout()
        if timeout is not None:
            self._on_result(timeout)

        if not self.paused:
            self._refresh()

        if now - self._last_slow > 0.5:
            self._slow(now)
            self._last_slow = now

        self._update_status(now)
        self.frames_drawn += 1
        if self._need_full:
            self._need_full = False
            self.fig.canvas.draw_idle()
        else:
            self.blit.update()

    def _ensure_stream(self, now: float) -> None:
        """Ask the robot to stream, because it will not do so on its own.

        The firmware starts a TCP client with ``streamHz = 0`` and its serial
        port at 10 Hz, so a viewer that only listens sees either nothing or a
        trickle. Three cases, in order:

        1. the robot announced a rate (``PROTOCOL,READY,...,stream_hz=N``) that
           is non-zero, or frames are arriving — it is already streaming, which
           means our own ``STREAM`` (or the user's) is in force: do nothing;
        2. it announced 0 — start at ``auto_stream_hz``;
        3. it said nothing (an older firmware) — start after a short grace
           period, once, and retry a few times if no frame ever arrives.

        The request is only sent while no command is in flight, so it cannot
        steal the reply that belongs to a button the user just pressed.
        """
        cfg = self.cfg
        if not cfg.auto_stream or not getattr(self.source, "accepts_commands", False):
            return
        if self._stream_settled or self._stream_asks >= 3:
            return
        if self.console._pending is not None:
            return
        if now - self._stream_asked_at < 1.5:
            return

        announced = self.health.announced_stream_hz
        receiving = self.source.rx_rows > 0

        if receiving or self._adopted_hz > 0:
            # Already flowing: adopt it rather than override what the user chose.
            # `announced` is what the robot says it is doing *now*, so trust it
            # for the rate read-out and leave the stream alone.
            if announced and announced > 0:
                self._adopted_hz = int(announced)
            self._stream_settled = True
            return
        if announced is not None and announced == 0 and now < self._want_stream_at:
            return
        if announced is None and now < self._want_stream_at:
            return

        self._stream_asked_at = now
        self._stream_asks += 1
        self.source.send(f"STREAM {cfg.auto_stream_hz}")
        self.health.set("data", WARN, f"asked the robot to stream at {cfg.auto_stream_hz} Hz",
                        "TCP clients start quiet; serial may be left at 0 Hz")

    def _slow(self, now: float) -> None:
        mark_t, mark_n = self._rx_mark
        total = self.source.rx_rows
        if now > mark_t:
            self.rx_rate = (total - mark_n) / (now - mark_t)
            # Let the health panel judge whether the link is throttling us; this
            # is the question "why are the samples slow?" asked as a check.
            self.health.set_achieved_rate(self.rx_rate)
        self._rx_mark = (now, total)
        fps_t, fps_n = self._fps_mark
        if now > fps_t:
            self.draw_fps = (self.frames_drawn - fps_n) / (now - fps_t)
        self._fps_mark = (now, self.frames_drawn)

        if self.cfg.presets.get(self.preset) is None:
            for panel, ax, _right, _artists, _clip in self.plots:
                if panel.ylim is None:
                    ax.relim()
                    ax.autoscale_view(scalex=False)
            self._need_full = True

    def _tick_safe(self) -> None:
        try:
            self.tick()
        except Exception:
            traceback.print_exc()

    def run(self) -> None:
        timer = self.fig.canvas.new_timer(interval=max(1, int(1000 / self.cfg.fps)))
        timer.add_callback(self._tick_safe)
        timer.start()
        self._timer = timer
        self.plt.show()

    def shutdown(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._timer.stop()
        except Exception:
            pass
        self.source.stop()
        if self.logger is not None:
            self.logger.close()
            print(f"log saved: {self.logger.path}")
        self.print_summary()

    def print_summary(self) -> None:
        if self.source.rx_rows == 0:
            return
        print("\nPanel range suggestions (largest |value| x 1.2):")
        overview = self.cfg.presets.get("overview") or {}
        last = self.ring.window(-math.inf)
        if last is None:
            return
        for panel in PANELS:
            if panel.range_key is None or panel.ylim is not None:
                continue
            peak = 0.0
            for line in panel.lines:
                if line.axis != "left":
                    continue
                series = last[CH[line.field]]
                finite = np.abs(series[np.isfinite(series)])
                if finite.size:
                    peak = max(peak, float(finite.max()))
            suggestion = nice_ceil(peak * 1.2)
            if suggestion is not None:
                print(
                    f"  {panel.range_key:5} peak {peak:9.3f}   suggested ±{suggestion:<7} "
                    f"(overview now ±{overview.get(panel.range_key)})"
                )
