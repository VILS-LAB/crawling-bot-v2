"""The command console: one button per protocol command, with results.

Why a widget-based console instead of raw typing: every command in the protocol
has a strict reply, so the viewer can tell you whether the robot accepted it.
Each action therefore reports back in three ways — the reply text, the button
colour, and the health checklist.

Commands that block (``CAL``) run in a background worker so the UI keeps
drawing; the button shows its progress and the health row tracks the stage.

Nothing here touches matplotlib; ``app.py`` passes in a small ``Panel`` factory,
which keeps this testable headlessly.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

# Button states -> face colours used by the app.
BUTTON_COLORS = {
    "idle": "0.90",
    "busy": "#ffe08a",
    "ok": "#b8e6b8",
    "err": "#f4a6a0",
}


@dataclass
class ActionResult:
    command: str
    reply: str
    ok: bool
    at: float


class Action:
    """One console button.

    ``build`` returns the command text to send, or None to do nothing.
    ``blocking`` marks commands that take a long time (CAL), which are run on a
    worker so the event loop stays responsive.
    """

    def __init__(
        self,
        key: str,
        label: str,
        build: Callable[[], Optional[str]],
        needs_reply: bool = True,
        blocking: bool = False,
        tooltip: str = "",
    ) -> None:
        self.key = key
        self.label = label
        self.build = build
        self.needs_reply = needs_reply
        self.blocking = blocking
        self.tooltip = tooltip


class CommandConsole:
    """Owns the pending-command bookkeeping and the result history."""

    def __init__(self, send: Callable[[str], bool], max_history: int = 60) -> None:
        self._send = send
        self.history: list[ActionResult] = []
        self.max_history = max_history

        self._lock = threading.Lock()
        self._pending: Optional[str] = None       # command awaiting a reply
        self._deadline = 0.0
        self._timeout = 2.5

        # A blocking command (CAL) reports a result when its worker finishes.
        self._result_queue: queue.SimpleQueue = queue.SimpleQueue()
        self._busy_until = 0.0
        self.busy_key: Optional[str] = None

        self.last_command = ""
        self.last_reply = ""

    # -- sending ------------------------------------------------------------

    def run(self, action: Action, argument: str = "") -> Optional[str]:
        """Send the action's command. Returns the text sent, or None."""
        command = action.build(argument) if argument else action.build()
        if command is None:
            return None
        with self._lock:
            if self._pending is not None:
                # Only one command in flight: a second would confuse replies.
                return None
            self._pending = command if action.needs_reply else None
            self._deadline = time.monotonic() + self._timeout
        self.busy_key = action.key if action.blocking else None
        self.last_command = command
        self._send(command)
        if not action.needs_reply:
            self._record(command, "OK (sent)", True)
        return command

    # -- receiving ----------------------------------------------------------

    def feed_line(self, line: str) -> Optional[ActionResult]:
        """Accept a reply line. Returns the result if it completed a command."""
        line = line.strip()
        if not (line.startswith("OK") or line.startswith("ERR") or line.startswith("CALRESULT")):
            return None
        with self._lock:
            command, self._pending = self._pending, None
        if command is None:
            return None
        return self._record(command, line, not line.startswith(("ERR", "CALRESULT,ok=0")))

    def feed_cal_result(self, line: str) -> Optional[ActionResult]:
        if not line.startswith("CALRESULT"):
            return None
        with self._lock:
            command, self._pending = self._pending, None
        return self._record(command or "CAL", line, ",ok=1" in line)

    def poll_timeout(self) -> Optional[ActionResult]:
        """If the robot never answered, record a timeout."""
        with self._lock:
            if self._pending is None or time.monotonic() < self._deadline:
                return None
            command = self._pending
            self._pending = None
        result = ActionResult(command, "TIMEOUT (no reply)", False, time.monotonic())
        self._remember(result)
        return result

    @property
    def pending(self) -> Optional[str]:
        with self._lock:
            return self._pending

    def _record(self, command: str, reply: str, ok: bool) -> ActionResult:
        result = ActionResult(command, reply, ok, time.monotonic())
        self.last_reply = reply
        if self.busy_key and command.upper().startswith("CAL"):
            self.busy_key = None
        self._remember(result)
        return result

    def _remember(self, result: ActionResult) -> None:
        self.history.append(result)
        if len(self.history) > self.max_history:
            self.history = self.history[-self.max_history :]

    def last_for(self, key: str) -> Optional[ActionResult]:
        for result in reversed(self.history):
            if result.command.upper().startswith(key.upper()):
                return result
        return None

    def mark_busy(self, key: str, seconds: float) -> None:
        self.busy_key = key
        self._busy_until = time.monotonic() + seconds


def action_set(legs) -> list[Action]:
    """The console's buttons, in the order they appear.

    ``legs`` must be a *callable* (or a live, mutable pair) that returns the
    current ``(down, up)`` slider angles. Reading it inside the button's builder
    is what makes ``ACT`` send the angles shown on the sliders: capturing a
    snapshot here instead froze ``ACT 90 90`` at the values present at build
    time, so moving a slider changed nothing.
    """
    read_legs = legs if callable(legs) else (lambda: legs)
    return [
        Action("ping", "PING", lambda: "PING", tooltip="liveness + uptime"),
        Action("id", "ID", lambda: "ID", tooltip="robot number, SSID, IP"),
        Action("get", "GET", lambda: "GET", tooltip="attitude + servo snapshot"),
        Action(
            "setid",
            "SETID",
            lambda arg="": f"SETID {int(arg)}" if str(arg).strip().isdigit() else None,
            tooltip="store robot number 1-8 (reboot to rename the AP)",
        ),
        Action(
            "stream_on",
            "STREAM 50",
            lambda: "STREAM 50",
            tooltip="telemetry at 50 Hz",
        ),
        Action(
            "stream_off",
            "STREAM 0",
            lambda: "STREAM 0",
            tooltip="stop telemetry",
        ),
        Action(
            "act",
            "ACT",
            lambda arg="": (
                lambda down, up: f"ACT {int(round(down))} {int(round(up))}"
            )(*read_legs()),
            tooltip="move both legs to the slider angles",
        ),
        Action("stop", "STOP", lambda: "STOP", tooltip="freeze both legs"),
        Action("park", "PARK", lambda: "PARK", tooltip="move to the park pose (180/180)"),
        Action(
            "cal",
            "CAL all",
            lambda: "CAL",
            blocking=True,
            tooltip="guided calibration: keep still, 6 positions, rotate 20 s",
        ),
    ]


CAL_STAGES = [
    (0, "gyro", "keep the robot still"),
    (1, "accel", "six positions, follow the OLED"),
    (2, "mag", "rotate slowly for 20 s"),
]


def stage_index(line: str) -> Optional[int]:
    """Which calibration stage a ``CAL,`` line refers to."""
    for index, name, _hint in CAL_STAGES:
        if line.startswith(f"CAL,{name}"):
            return index
    if line.startswith("CAL,START"):
        return 0
    return None
