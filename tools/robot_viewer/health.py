"""Turn diagnostic lines and ``GET`` snapshots into a pass/fail checklist.

The firmware prints its own findings; the viewer's job is to collect them and
judge what it can see itself. Two sources feed this:

* **Log lines** — ``PROTOCOL,`` ``CAL,`` ``HEALTH,`` ``ERROR,`` ``WARN,`` and the
  ``AP SSID:``/``IP Address:`` boot banners. They only arrive once (at boot) or
  during a calibration, so they are remembered.
* **Telemetry** — cheap continuous checks the firmware cannot report: is data
  arriving, is the sampling rate right, is the estimator seeded.

Each check has a state of *unknown*, *ok*, *warn* or *fail*, and a short detail
so a red row says what to do rather than just that something is wrong.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

OK, WARN, FAIL, UNKNOWN = "ok", "warn", "fail", "unknown"

ICONS = {OK: "✓", WARN: "!", FAIL: "✗", UNKNOWN: "·"}

# ServoHealthCode, from lib/ServoControl/ServoHealth.h
SERVO_CODES = {
    0: (OK, "OK"),
    1: (FAIL, "attach failed - pin not PWM-capable"),
    2: (WARN, "not attached"),
    3: (FAIL, "pulse read-back mismatch"),
    4: (FAIL, "no motion seen by the IMU"),
    5: (UNKNOWN, "disabled (ENABLE_SERVOS=0)"),
}


@dataclass
class Check:
    key: str
    label: str
    state: str = UNKNOWN
    detail: str = ""
    hint: str = ""       # what to do when it is not OK

    def set(self, state: str, detail: str = "", hint: str = "") -> None:
        self.state = state
        self.detail = detail
        if hint:
            self.hint = hint

    @property
    def icon(self) -> str:
        return ICONS.get(self.state, "·")


@dataclass
class HealthReport:
    """The checklist. Ordered: what breaks the robot first, then cosmetics."""

    checks: dict[str, Check] = field(default_factory=dict)
    boot_lines: int = 0
    last_error: str = ""
    announced_stream_hz: Optional[float] = None  # PROTOCOL,READY,stream_hz=N
    requested_hz: float = 0.0     # from the STREAM reply; 0 = not asked this session
    achieved_hz: float = 0.0      # rows per second actually arriving
    throttled: bool = False       # achieved rate is materially below the request

    def __post_init__(self) -> None:
        for key, label in (
            ("probe", "Robot answering"),
            ("identity", "Identity / network"),
            ("data", "Telemetry stream"),
            ("rate", "IMU sample rate"),
            ("estimator", "Attitude estimator"),
            ("calibration", "Calibration"),
            ("servos", "Servo self-test"),
            ("sensor", "Sensor sanity"),
            ("protocol", "Protocol handshake"),
        ):
            self.checks[key] = Check(key, label)

    # -- convenience --------------------------------------------------------

    def __getitem__(self, key: str) -> Check:
        return self.checks[key]

    def set(self, key: str, state: str, detail: str = "", hint: str = "") -> None:
        self.checks[key].set(state, detail, hint)

    @property
    def failures(self) -> list[Check]:
        return [c for c in self.checks.values() if c.state == FAIL]

    @property
    def warnings(self) -> list[Check]:
        return [c for c in self.checks.values() if c.state == WARN]

    @property
    def verdict(self) -> tuple[str, str]:
        """(icon, summary) for the headline."""
        fails, warns = len(self.failures), len(self.warnings)
        if fails:
            first = self.failures[0]
            return "✗", f"{fails} problem{'s' if fails > 1 else ''}: {first.label}"
        if warns:
            return "!", f"{warns} warning{'s' if warns > 1 else ''}"
        known = [c for c in self.checks.values() if c.state == OK]
        if not known:
            return "·", "waiting for data"
        return "✓", f"all {len(known)} checks passed"

    # -- log lines ----------------------------------------------------------

    def feed_line(self, line: str) -> None:
        """Update from one non-telemetry line."""
        self.boot_lines += 1
        line = line.strip()
        if not line:
            return

        if line.startswith("ERROR,") or line.startswith("ERROR"):
            self.last_error = line
            if "IMU_INIT_FAILED" in line:
                self.set("data", FAIL, "IMU did not initialise",
                         "check IMU wiring and 3.3 V")
            elif "IMU_TASK_FAILED" in line:
                self.set("data", FAIL, "IMU task did not start", "reflash / report")
            elif "HEALTH_CHECK_FAILED" in line:
                self.set("servos", WARN, "boot self-test reported a fault")
            elif "I2C" in line or "SSD1306" in line:
                self.set("identity", WARN, "display/I2C issue (non-fatal)")
            else:
                self.set("probe", WARN, "robot reported an error", line)
            return

        if line.startswith("PROTOCOL,READY"):
            self.set("protocol", OK, "serial + TCP listener up")
            return

        if line.startswith("PROTOCOL,CLIENT"):
            if "dropped" in line:
                self.set("data", WARN, "a client was dropped", line)
            return

        if line.startswith("AP SSID:"):
            self.set("identity", OK, line.strip())
            return
        if line.startswith("IP Address:"):
            check = self["identity"]
            detail = f"{check.detail} · {line.split(':', 1)[1].strip()}".strip(" ·")
            self.set("identity", OK if check.state != OK else OK, detail)
            return

        if line.startswith("CAL,SELF_CHECK,ok"):
            self.set("calibration", OK, "stored calibration verified")
            return
        if line.startswith("CAL,LOADED"):
            self.set("calibration", WARN, "loaded, not yet verified")
            return
        if "SELF_CHECK" in line and "ok" not in line:
            self.set("calibration", FAIL, "self-check failed",
                     "run CAL (see calibrate.py or the console)")
            return
        if line.startswith("CAL,NONE"):
            self.set("calibration", FAIL, "no calibration stored",
                     "run CAL; the legs stay parked until then")
            return
        if line.startswith("CAL,FAIL"):
            self.set("calibration", FAIL, line[4:][:48], "rerun CAL")
            return
        if line.startswith("CAL,DONE"):
            self.set("calibration", OK, "guided calibration completed")
            return
        if line.startswith("ROBOT,UNCALIBRATED"):
            self.set("calibration", FAIL, "robot booted uncalibrated",
                     "run CAL")
            return

        if line.startswith("HEALTH,SERVO,START"):
            return
        if line.startswith("HEALTH,SERVO,SKIPPED"):
            self.set("servos", UNKNOWN, "self-test skipped (uncalibrated)")
            return
        if line.startswith("HEALTH,SERVO,"):
            parts = dict(
                item.split("=", 1) for item in line.split(",")[2:] if "=" in item
            )
            name = line.split(",")[2]
            code = int(parts.get("code", "5"))
            state, meaning = SERVO_CODES.get(code, (FAIL, f"code {code}"))
            current = self["servos"]
            detail = f"{name}: {meaning}"
            if current.detail and current.state not in (FAIL,):
                detail = f"{current.detail} · {detail}"
            hint = "see docs/SERVO_HEALTH.md" if state == FAIL else ""
            # A single bad servo decides the row; never downgrade a failure.
            if current.state == FAIL and state != FAIL:
                return
            self.set("servos", state, detail, hint)
            return
        if line.startswith("HEALTH,SUMMARY"):
            parts = dict(
                item.split("=", 1) for item in line.split(",")[1:] if "=" in item
            )
            ok = parts.get("ok", "0") == "1"
            if not ok:
                self.set("servos", FAIL, "self-test failed",
                         "see docs/SERVO_HEALTH.md")
            return

        if line.startswith("PROTOCOL,READY"):
            # "PROTOCOL,READY,serial=115200,tcp_port=4242,stream_hz=10" is what
            # the robot streams at *right now*; 0 means it waits to be asked.
            for part in line.split(","):
                if part.startswith("stream_hz="):
                    try:
                        self.announced_stream_hz = float(part.split("=", 1)[1])
                    except ValueError:
                        pass
            return

        if line.startswith("OK stream="):
            # "OK stream=<hz>": what the robot was asked to emit. Comparing it
            # with the achieved rate tells you whether the link is the limit.
            try:
                self.requested_hz = float(line.split("=", 1)[1].split(",")[0])
            except (IndexError, ValueError):
                pass
            return

        if line.startswith("WARN,ROBOT_NUM_INVALID"):
            self.set("identity", WARN, "EEPROM invalid - defaulted to robot 1",
                     "send SETID <1..8>, then reboot")
            return

    # -- telemetry ----------------------------------------------------------

    def set_achieved_rate(self, hz: float) -> None:
        """Record the measured arrival rate and judge whether it is throttled.

        The robot can only emit what the transport drains: over USB-serial or
        WiFi the achieved rate is often far below ``STREAM <hz>``, and that looks
        like "slow samples" with no explanation. Say so explicitly, and name the
        likely cause, instead of leaving it to guesswork.

        This measurement is taken every half second but the "we are receiving
        data" reset runs every frame, so the verdict has to be re-applied after
        that reset (see :meth:`feed_row`) or it would only survive one frame.
        """
        if hz <= 0:
            return
        self.achieved_hz = hz
        if self.requested_hz <= 0:
            return
        self.throttled = hz < self.requested_hz * 0.7
        if self.throttled:
            self.set(
                "data", WARN,
                f"{hz:.0f} Hz of {self.requested_hz:.0f} requested",
                "the link, not the robot, is the limit: lower STREAM, "
                "or use TCP for more headroom",
            )

    def mark_data_seen(self, samples: int) -> None:
        """Telemetry arrived: pass, unless the measured rate says otherwise.

        ``app._drain`` calls this on every frame, while the rate is measured
        twice a second, so the throttle verdict is re-applied here. Without that
        the generic "receiving data" pass would overwrite the warning on the very
        next frame and the user would never see it.
        """
        if self.throttled:
            self.set(
                "data", WARN,
                f"{self.achieved_hz:.0f} Hz of {self.requested_hz:.0f} requested",
                "the link, not the robot, is the limit: lower STREAM, "
                "or use TCP for more headroom",
            )
        else:
            # Report the *rate*, not this frame's batch: a per-frame count reads
            # "6 samples" while the stream is a healthy 55 Hz, which looks broken.
            rate = f"{self.achieved_hz:.0f} Hz" if self.achieved_hz > 0 else "receiving"
            # Data flowing also clears "asked the robot to stream ...": the ask
            # worked, and leaving it would look like a fault.
            self.set("data", OK, rate)

    def feed_row(self, row) -> None:
        """Cheap continuous checks from one parsed sample."""
        import math

        from .telemetry import CH

        def value(name):
            v = row[CH[name]]
            return None if not isinstance(v, float) or not math.isfinite(v) else v

        rate = value("rate_hz")
        if rate is not None:
            if 150.0 <= rate <= 260.0:
                self.set("rate", OK, f"{rate:.0f} Hz")
            else:
                self.set("rate", WARN, f"{rate:.0f} Hz (expected ~200)",
                         "the IMU task may be starved")

        seeded = value("est_init")
        if seeded is not None:
            self.set("estimator", OK if seeded >= 0.5 else WARN,
                     "seeded" if seeded >= 0.5 else "waiting for a gravity vector")

        trust = value("trust")
        accel_norm = value("acc_norm_g")
        if accel_norm is not None and trust is not None:
            if trust > 0.5 and not (0.85 <= accel_norm <= 1.15):
                self.set("sensor", WARN, f"|a| {accel_norm:.2f} g while trusting accel",
                         "sensor may be loose or mis-calibrated")
            elif 0.85 <= accel_norm <= 1.15:
                self.set("sensor", OK, f"|a| {accel_norm:.2f} g")
            else:
                self.set("sensor", WARN, f"|a| {accel_norm:.2f} g")

    # -- command results ----------------------------------------------------

    def feed_reply(self, command: str, reply: str) -> None:
        """Judge one command/reply pair."""
        verb = command.split()[0].upper() if command.split() else command.upper()
        if reply.startswith("ERR"):
            code = reply[4:].split(",")[0]
            if verb == "PING":
                self.set("probe", FAIL, f"ERR {code}")
            elif verb == "SETID":
                self.set("identity", WARN, f"SETID rejected: {code}")
            elif verb in ("ACT", "STOP", "PARK"):
                self.set("servos", WARN, f"{verb} rejected: {code}",
                         "servos may be disabled in this build")
            elif verb == "CAL":
                self.set("calibration", FAIL, f"CAL rejected: {code}")
            else:
                self.set("protocol", WARN, f"{verb}: {code}")
            return

        if verb == "PING":
            self.set("probe", OK, "answered PING")
        elif verb == "ID":
            self.set("identity", OK, reply[3:].strip() if reply.startswith("OK ") else reply)
        elif verb == "GET":
            self.set("probe", OK, "GET returned a snapshot")
        elif verb in ("ACT", "STOP", "PARK"):
            current = self["servos"]
            if current.state in (FAIL, UNKNOWN):
                self.set("servos", OK, f"{verb} accepted")
        elif verb == "CAL":
            self.set("calibration", WARN, "calibration in progress")
