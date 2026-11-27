"""Where data comes from: serial, TCP, demo and replay.

Every source runs in a background thread and pushes two kinds of message into
one queue:

* a parsed telemetry row (``np.ndarray``), and
* a raw non-telemetry line (``str``) — boot banners, ``HEALTH,``/``CAL,``/
  ``PROTOCOL,`` diagnostics — which the health panel interprets.

Commands travel the other way through ``send()``. Only serial and TCP can send;
demo and replay report that they cannot instead of silently dropping commands.

The robot's wire protocol may interleave a command reply (``OK``/``ERR``) with
telemetry, so replies are forwarded as strings too and the console picks them up
by prefix.
"""

from __future__ import annotations

import math
import queue
import socket
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

from .telemetry import N_CH, CH, LineParser, finish_row

DEFAULT_TCP_PORT = 4242
DEFAULT_HOST = "192.168.4.1"
RX_STALL_S = 1.0


class Source(threading.Thread):
    """Base class: owns the thread, the command queue and the rx counters."""

    accepts_commands = False

    def __init__(self, out: queue.SimpleQueue, parser: LineParser, cfg) -> None:
        super().__init__(daemon=True, name=type(self).__name__)
        self.out = out
        self.parser = parser
        self.cfg = cfg

        self.stop_event = threading.Event()

        self.state = "starting"
        self.rejected = 0
        self.last_reject = ""

        self.out_q: queue.SimpleQueue = queue.SimpleQueue()
        self.last_tx = ""
        self.tx_count = 0

        self.rx_rows = 0
        self.rx_lines = 0
        self.last_rx_at: Optional[float] = None
        self.connected = False

    # -- lifecycle ----------------------------------------------------------

    def stop(self) -> None:
        self.stop_event.set()

    # -- rx -----------------------------------------------------------------

    def handle_line(self, line: str) -> None:
        self.rx_lines += 1
        text = line.strip()

        # Only data lines are telemetry. Everything else (``OK``/``ERR`` replies,
        # ``HEALTH,``/``CAL,`` diagnostics, the ``#`` header) must reach the app
        # verbatim: the console matches replies by prefix, and the health panel
        # parses the rest. Feeding them to the parser first made every reply look
        # like a malformed telemetry row, so it was dropped and every command
        # reported "no reply" while the robot had in fact obeyed it.
        if self.parser.is_data_line(line):
            row = self.parser.feed(line, time.monotonic())
            if row is not None:
                self.rx_rows += 1
                self.last_rx_at = time.monotonic()
                self.out.put(row)
                return
            if self.parser.last_reason:
                self.rejected += 1
                self.last_reject = f"{self.parser.last_reason}: {text[:60]}"
                return

        if text:
            self.out.put(text)

    # -- tx -----------------------------------------------------------------

    def send(self, command: str) -> bool:
        """Queue one protocol command. Returns False when this source cannot send."""
        self.last_tx = command
        if not self.accepts_commands:
            return False
        self.tx_count += 1
        self.out_q.put((command + self.cfg.tx_eol).encode("ascii", "ignore"))
        return True

    def drain_commands(self, write) -> None:
        """Write every queued command using ``write(bytes)``."""
        while True:
            try:
                payload = self.out_q.get_nowait()
            except queue.Empty:
                return
            try:
                write(payload)
            except OSError:
                return


def list_serial_ports() -> list[str]:
    """Every serial port the OS reports, for ``--list-ports``."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [port.device for port in list_ports.comports()]


def find_serial_port() -> Optional[str]:
    """Best-effort match for an ESP32-class USB device.

    Checks concrete USB naming patterns instead of a loose substring search, so
    virtual ports such as ``/dev/cu.Bluetooth-Incoming-Port`` ("incoming"
    contains "com") are not picked. Windows ``COM5`` is matched on the device
    name only, and a native-USB port is preferred over a UART bridge.
    """
    try:
        from serial.tools import list_ports
    except ImportError:
        return None

    native, bridge = [], []
    for port in list_ports.comports():
        device = (port.device or "").lower()
        text = device + " " + (port.description or "").lower()
        if device.startswith("/dev/cu.usbmodem") or device.startswith("/dev/ttyacm"):
            native.append(port.device)          # ESP32-S3 native USB (CDC)
        elif any(hint in text for hint in ("usbserial", "slab", "ch340", "cp210", "ftdi")):
            bridge.append(port.device)          # UART bridge
        elif device.startswith("com") and device[3:].isdigit():
            bridge.append(port.device)          # Windows COM port

    for candidate in (native, bridge):
        if candidate:
            return sorted(candidate)[0]
    return None


class SerialSource(Source):
    """USB serial at ``cfg.baud`` (USB-CDC ignores the rate, a UART does not)."""

    accepts_commands = True

    def run(self) -> None:
        try:
            import serial
        except ImportError:
            self.state = "pyserial missing: pip install pyserial"
            return

        while not self.stop_event.is_set():
            port = self.cfg.port if self.cfg.port != "auto" else find_serial_port()
            if not port:
                self.state = "no serial port found (retrying)"
                self.stop_event.wait(1.0)
                continue
            try:
                with serial.Serial(port, self.cfg.baud, timeout=0.05,
                                   rtscts=False, dsrdtr=False) as port_handle:
                    # One telemetry line is ~440 B, which does not fit the
                    # default 256 B receive buffer, so a burst is always split.
                    # Ask the driver for more and keep the read loop tight.
                    try:
                        port_handle.set_buffer_size(rx_size=8192, tx_size=8192)
                    except (AttributeError, OSError):
                        pass
                    self.state = f"serial {port} @ {self.cfg.baud}"
                    self.connected = True
                    buffer = b""
                    while not self.stop_event.is_set():
                        self.drain_commands(port_handle.write)
                        chunk = port_handle.read(max(1, port_handle.in_waiting))
                        if not chunk:
                            continue
                        buffer += chunk
                        *lines, buffer = buffer.split(b"\n")
                        for raw in lines:
                            self.handle_line(raw.decode("ascii", "ignore"))
                        if len(buffer) > 65536:  # a partial line that never ends
                            buffer = b""
            except (OSError, Exception) as exc:  # serial.SerialException is an OSError
                if self.stop_event.is_set():
                    return
                self.connected = False
                self.state = f"serial error: {str(exc)[:44]}"
                self.stop_event.wait(1.0)


class TcpSource(Source):
    """TCP to ``cfg.host:cfg.port`` — the protocol's network transport."""

    accepts_commands = True

    def run(self) -> None:
        while not self.stop_event.is_set():
            host = self.cfg.host
            port = self.cfg.tcp_port
            try:
                with socket.create_connection((host, port), timeout=3.0) as sock:
                    sock.settimeout(0.05)
                    self.state = f"tcp {host}:{port}"
                    self.connected = True
                    buffer = b""
                    while not self.stop_event.is_set():
                        self.drain_commands(sock.sendall)
                        try:
                            chunk = sock.recv(8192)
                        except socket.timeout:
                            continue
                        if not chunk:
                            raise ConnectionError("robot closed the connection")
                        buffer += chunk
                        *lines, buffer = buffer.split(b"\n")
                        for raw in lines:
                            self.handle_line(raw.decode("ascii", "ignore"))
                        if len(buffer) > 65536:
                            buffer = b""
            except OSError as exc:
                self.connected = False
                if self.stop_event.is_set():
                    return
                self.state = f"tcp: {str(exc)[:44]} (retrying)"
                self.stop_event.wait(1.5)


class DemoSource(Source):
    """Synthetic robot: still -> crawling -> still -> tilt, repeating every 12 s.

    Also emits one plausible boot banner set so the health panel has something
    to show without hardware.
    """

    def __init__(self, out, parser, cfg) -> None:
        super().__init__(out, parser, cfg)
        self._banners_sent = False

    def _banners(self) -> None:
        for line in (
            # 10 Hz like a real serial port, not 0 like a fresh TCP client:
            # the demo has no command channel, so it must look already-streaming
            # or the viewer would try to ask it for a stream it cannot send.
            "PROTOCOL,READY,serial=115200,tcp_port=4242,stream_hz=10",
            "CAL,LOADED,robot=1",
            "CAL,SELF_CHECK,ok",
            "HEALTH,SERVO,down,pin=7,code=0,status=OK",
            "HEALTH,SERVO,up,pin=6,code=0,status=OK",
            "HEALTH,SUMMARY,ok=1,down=0,up=0",
        ):
            self.out.put(line)
        self._banners_sent = True

    def run(self) -> None:
        self.state = "demo data"
        self.connected = True
        self._banners()

        rng = np.random.default_rng(7)
        rate = 100.0
        dt = 1.0 / rate
        start = time.monotonic()
        produced = 0
        low_pass = np.zeros(3)
        velocity = np.zeros(3)
        position = np.zeros(3)
        previous_attitude = np.array([0.0, 5.0, -172.0])
        still_count = 0

        while not self.stop_event.is_set():
            target = int((time.monotonic() - start) * rate)
            while produced < target:
                t = produced * dt
                produced += 1
                phase = t % 12.0
                moving = 2.0 <= phase < 6.0

                if moving:
                    accel_true = np.array([
                        0.4 * np.sin(2.1 * t),
                        2.2 * np.sin(2 * np.pi * 1.4 * t),
                        0.0,
                    ])
                else:
                    accel_true = np.zeros(3)
                accel_raw = accel_true + rng.normal(0, 0.9 if moving else 0.06, 3)
                if moving and rng.random() < 0.02:
                    accel_raw = accel_raw + rng.normal(0, 3.5, 3)
                low_pass += 0.12 * (accel_raw - low_pass)

                tilt = -30.0 * np.sin(np.pi * (phase - 9.0)) if 9.0 <= phase < 10.0 else 0.0
                attitude = np.array([
                    0.3 * np.sin(t),
                    5.0 + tilt,
                    ((-172 + 25 * math.sin(0.3 * t) + 180) % 360) - 180,
                ])
                delta = attitude - previous_attitude
                delta[2] = (delta[2] + 180) % 360 - 180
                previous_attitude = attitude
                gyro_raw = delta * rate + rng.normal(0, 3.0 if moving else 0.4, 3)

                still_count = still_count + 1 if (not moving and tilt == 0.0) else 0
                still = still_count > 30
                velocity += low_pass * dt
                velocity = np.zeros(3) if still else velocity * 0.996
                position += velocity * dt

                row = np.full(N_CH, np.nan)
                values = {
                    "t": t,
                    "dt": 0.005,
                    "ax": low_pass[0], "ay": low_pass[1], "az": 9.8 + low_pass[2],
                    "lax": accel_raw[0], "lay": accel_raw[1], "laz": accel_raw[2],
                    "flax": low_pass[0], "flay": low_pass[1], "flaz": low_pass[2],
                    "gx": gyro_raw[0], "gy": gyro_raw[1], "gz": gyro_raw[2],
                    "fgx": gyro_raw[0] * 0.8, "fgy": gyro_raw[1] * 0.8, "fgz": gyro_raw[2] * 0.8,
                    "roll": attitude[0], "pitch": attitude[1], "yaw": attitude[2],
                    "rollG": attitude[0] + 0.2 * math.sin(0.4 * t),
                    "pitchG": attitude[1] + 0.5 * math.sin(0.2 * t),
                    "vx": velocity[0], "vy": velocity[1], "vz": velocity[2],
                    "px": position[0], "py": position[1], "pz": position[2],
                    "temp_c": 36.3 + 0.01 * t,
                    "trust": 1.0 if still else (0.2 if moving else 0.5),
                    "stdA": 0.0057 * (30 if moving else 1),
                    "still": float(still),
                    "moving": float(moving),
                    "still_failure_mask": 0.0 if still else float((4 if moving else 0) | (1 if tilt else 0)),
                    "ebx": 0.03 * math.sin(0.2 * t),
                    "eby": -0.02 + 0.02 * math.sin(0.13 * t),
                    "ebz": 0.05 * math.sin(0.07 * t),
                    "rate_hz": 199.0,
                    "acc_norm_g": 1.0,
                    "mag_norm": 48.0,
                    "accel_rms_g": 0.004,
                    "mag_rms_ut": 0.8,
                    "mag_fit": 2,
                    "est_init": 1,
                }
                for key, value in values.items():
                    row[CH[key]] = value
                self.rx_rows += 1
                self.last_rx_at = time.monotonic()
                self.out.put(finish_row(row))

            time.sleep(0.004)


class ReplaySource(Source):
    """Replays a CSV written by this viewer (or by the firmware, with a header)."""

    def __init__(self, out, parser, cfg, path: str, speed: float = 1.0) -> None:
        super().__init__(out, parser, cfg)
        self.path = Path(path)
        self.speed = max(0.05, speed)

    def run(self) -> None:
        if not self.path.exists():
            self.state = f"replay: no such file {self.path}"
            return
        self.state = f"replay {self.path.name}"
        self.connected = True
        wall_start = time.monotonic()
        first_t: Optional[float] = None

        for line in self.path.read_text(errors="replace").splitlines():
            if self.stop_event.is_set():
                return
            row = self.parser.feed(line, time.monotonic())
            if row is None:
                if self.parser.last_reason is None and line.strip():
                    self.out.put(line.strip())
                continue

            if first_t is None:
                first_t = float(row[CH["t"]])
            target = (float(row[CH["t"]]) - first_t) / self.speed
            delay = target - (time.monotonic() - wall_start)
            if delay > 0 and self.stop_event.wait(delay):
                return
            self.rx_rows += 1
            self.last_rx_at = time.monotonic()
            self.out.put(row)

        self.state = "replay finished"
        self.connected = False
