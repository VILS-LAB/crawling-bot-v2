"""High-level robot client: one method per protocol command, typed results.

Design notes:

* Commands are serialized. Two threads sharing one :class:`Robot` would
  interleave request/response frames, so a caller that needs concurrency should
  open one connection per thread (the firmware serves
  ``PROTOCOL_MAX_CLIENTS`` connections).
* Every command has a deadline. A reply that never arrives raises
  :class:`~crawlingbot.protocol.RobotTimeout` instead of hanging the caller.
* Unsolicited events (``IMU,``, ``CAL,``) are routed to the appropriate handler
  rather than being mistaken for a command reply.
"""

from __future__ import annotations

import time
from typing import Callable, Dict, Iterator, Optional

from .protocol import (
    CAL_PREFIX,
    CAL_RESULT_PREFIX,
    TELEMETRY_COLUMNS,
    ImuFrame,
    RobotError,
    RobotTimeout,
    Snapshot,
    is_event,
    is_reply,
    parse_header,
    parse_reply,
    split_kv,
)
from .transport import DEFAULT_TCP_HOST, SerialTransport, TcpTransport, Transport

DEFAULT_TIMEOUT = 2.0
SWEEP_TIMEOUT = 900.0  # a full guided calibration needs a human


class Robot:
    """A connection to one robot.

    Construct with :meth:`tcp` or :meth:`serial`, or pass any
    :class:`~crawlingbot.transport.Transport`.
    """

    def __init__(
        self,
        transport: Transport,
        timeout: float = DEFAULT_TIMEOUT,
        log: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._transport = transport
        self._timeout = timeout
        self._log = log
        self._columns: Optional[tuple[str, ...]] = None  # from the header line

    # -- construction -------------------------------------------------------

    @classmethod
    def tcp(
        cls,
        host: str = DEFAULT_TCP_HOST,
        port: int = 4242,
        timeout: float = DEFAULT_TIMEOUT,
        log: Optional[Callable[[str], None]] = None,
    ) -> "Robot":
        return cls(TcpTransport(host, port), timeout=timeout, log=log)

    @classmethod
    def serial(
        cls,
        port: str,
        baud: int = 115200,
        timeout: float = DEFAULT_TIMEOUT,
        log: Optional[Callable[[str], None]] = None,
        quiet: bool = True,
    ) -> "Robot":
        """Open a serial connection.

        `quiet` stops the default telemetry stream so command replies are not
        buried in ``IMU,`` frames. Pass ``quiet=False`` to keep it.
        """
        transport = SerialTransport(port, baud)
        robot = cls(transport, timeout=timeout, log=log)
        if quiet:
            robot.stream(0, timeout=1.0)
        return robot

    # -- lifecycle ---------------------------------------------------------

    def close(self) -> None:
        self._transport.close()

    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    @property
    def description(self) -> str:
        return self._transport.description

    # -- core request/response ---------------------------------------------

    def _note(self, line: str) -> None:
        if self._log is not None:
            self._log(line)

    def command(self, text: str, timeout: Optional[float] = None) -> Dict[str, str]:
        """Send one command and return its parsed ``OK`` reply.

        Raises :class:`RobotError` for ``ERR`` replies and :class:`RobotTimeout`
        if nothing arrives before the deadline.
        """
        if "\n" in text:
            raise ValueError("a command must not contain a newline")

        self._transport.write(text.encode("ascii") + b"\n")
        deadline = time.monotonic() + (self._timeout if timeout is None else timeout)

        while True:
            line = self._transport.read_line(deadline)
            if line is None:
                raise RobotTimeout(f"no reply to {text!r} within the deadline")
            line = line.strip()
            if not line:
                continue

            # Telemetry and calibration progress are events, not replies.
            if is_event(line):
                self._note(line)
                continue
            if not is_reply(line):
                # Boot banners and other informational lines.
                self._note(line)
                continue

            self._note(f"<- {line}")
            return parse_reply(line)

    # -- identity ----------------------------------------------------------

    def ping(self) -> int:
        """Return the robot's uptime in milliseconds."""
        reply = self.command("PING")
        pong = reply.get("_positional", "")
        # "OK PONG 123456" -> _positional == "PONG 123456"
        for token in pong.split():
            if token.isdigit():
                return int(token)
        return 0

    @property
    def robot_id(self) -> int:
        return int(self.command("ID").get("id", 0))

    def id_info(self) -> Dict[str, str]:
        return self.command("ID")

    def set_id(self, number: int) -> int:
        """Store a new robot number (1-8). Takes effect on the next boot for
        the WiFi access point name."""
        if not 1 <= number <= 8:
            raise ValueError("robot number must be between 1 and 8")
        return int(self.command(f"SETID {number}").get("id", number))

    # -- state -------------------------------------------------------------

    def snapshot(self) -> Snapshot:
        return Snapshot.parse(self._raw_command("GET"))

    def _raw_command(self, text: str, timeout: Optional[float] = None) -> str:
        """Like :meth:`command` but returns the raw reply line."""
        self._transport.write(text.encode("ascii") + b"\n")
        deadline = time.monotonic() + (self._timeout if timeout is None else timeout)
        while True:
            line = self._transport.read_line(deadline)
            if line is None:
                raise RobotTimeout(f"no reply to {text!r} within the deadline")
            line = line.strip()
            if not line:
                continue
            if is_event(line) or not is_reply(line):
                self._note(line)
                continue
            self._note(f"<- {line}")
            if line.startswith("ERR "):
                parse_reply(line)  # raises RobotError with code/detail
            return line

    # -- streaming ---------------------------------------------------------

    def stream(self, hz: int, timeout: Optional[float] = None) -> int:
        """Set the telemetry rate for this connection (0 turns it off)."""
        if hz < 0:
            raise ValueError("rate must be >= 0")
        reply = self.command(f"STREAM {int(hz)}", timeout=timeout)
        return int(reply.get("stream", 0))

    def frames(
        self,
        hz: int = 50,
        count: Optional[int] = None,
        duration: Optional[float] = None,
        timeout: Optional[float] = None,
    ) -> Iterator[ImuFrame]:
        """Yield telemetry frames until `count` frames or `duration` seconds.

        Enables streaming for this connection, and turns it off again when the
        iterator is exhausted or closed::

            for frame in bot.frames(hz=50, count=200):
                print(frame.roll, frame.pitch)
        """
        self.stream(hz, timeout=timeout)
        deadline = None if duration is None else time.monotonic() + duration
        produced = 0
        try:
            while count is None or produced < count:
                if deadline is not None and time.monotonic() >= deadline:
                    return
                line = self._transport.read_line(time.monotonic() + 1.0)
                if line is None:
                    continue
                line = line.strip()
                if not line:
                    continue

                header = parse_header(line)
                if header is not None:
                    self._columns = header
                    continue
                if not line.startswith("IMU,"):
                    if is_reply(line):
                        self._note(f"<- {line}")
                    else:
                        self._note(line)
                    continue

                yield ImuFrame.parse(line, self._columns or TELEMETRY_COLUMNS)
                produced += 1
        finally:
            try:
                self.stream(0, timeout=min(1.0, self._timeout))
            except (RobotError, RobotTimeout, OSError):
                pass

    # -- actuation ---------------------------------------------------------

    def act(self, down: int, up: int) -> tuple[int, int]:
        """Command both legs. Returns the clamped targets the robot accepted."""
        reply = self.command(f"ACT {int(down)} {int(up)}")
        return int(reply["d"]), int(reply["u"])

    def stop(self) -> None:
        self.command("STOP")

    def park(self) -> None:
        self.command("PARK")

    # -- calibration -------------------------------------------------------

    def calibrate(
        self,
        target: str = "all",
        on_progress: Optional[Callable[[str], None]] = None,
        timeout: Optional[float] = None,
    ) -> bool:
        """Run a guided calibration and return whether it succeeded.

        Blocks for minutes: the robot guides a human through keeping still,
        six accelerometer positions and a 20 s magnetometer rotation.

        :param target: ``all``, ``gyro``, ``accel`` or ``mag``.
        :param on_progress: called with each ``CAL,...`` stage line.
        """
        if target not in ("all", "gyro", "accel", "mag"):
            raise ValueError("target must be all, gyro, accel or mag")

        command = "CAL" if target == "all" else f"CAL {target}"
        self.command(command, timeout=self._timeout)

        deadline = time.monotonic() + (SWEEP_TIMEOUT if timeout is None else timeout)
        while True:
            line = self._transport.read_line(deadline)
            if line is None:
                raise RobotTimeout("calibration produced no result")
            line = line.strip()
            if not line:
                continue

            if line.startswith(CAL_RESULT_PREFIX):
                reply = split_kv(line[len(CAL_RESULT_PREFIX) :])
                return reply.get("ok", "0") == "1"

            if line.startswith(CAL_PREFIX):
                if on_progress is not None:
                    on_progress(line)
                self._note(line)
                continue

            if line.startswith("IMU,"):
                continue
            self._note(f"<- {line}")
