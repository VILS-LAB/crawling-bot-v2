"""Byte transports: the only part that knows whether the robot is on a socket
or a serial port. Everything above this layer sees a line-oriented stream.
"""

from __future__ import annotations

import socket
import time
from abc import ABC, abstractmethod
from typing import Optional

DEFAULT_TCP_PORT = 4242
DEFAULT_BAUD = 115200
DEFAULT_TCP_HOST = "192.168.4.1"


class TransportError(OSError):
    """Raised when the transport cannot be opened or has been lost."""


class Transport(ABC):
    """A bidirectional, line-oriented connection to the robot."""

    @abstractmethod
    def write(self, data: bytes) -> None:
        """Send raw bytes. Appends no newline; the caller owns framing."""

    @abstractmethod
    def read_line(self, deadline: float) -> Optional[str]:
        """Return one line without its terminator, or None after `deadline`.

        `deadline` is a ``time.monotonic()`` value, not a duration.
        """

    @abstractmethod
    def close(self) -> None:
        """Release the connection. Safe to call more than once."""

    def __enter__(self) -> "Transport":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    @property
    def description(self) -> str:
        return type(self).__name__


class TcpTransport(Transport):
    """TCP connection to the robot's access point (default 192.168.4.1:4242)."""

    def __init__(
        self,
        host: str = DEFAULT_TCP_HOST,
        port: int = DEFAULT_TCP_PORT,
        timeout: float = 3.0,
    ) -> None:
        self.host = host
        self.port = port
        self._timeout = timeout
        self._buffer = b""
        try:
            self._socket = socket.create_connection((host, port), timeout=timeout)
        except OSError as exc:
            raise TransportError(
                f"cannot connect to {host}:{port} ({exc}). "
                "Is the robot powered on and are you joined to its WiFi?"
            ) from exc
        # Reads must poll so the caller's deadline is honoured; writes should
        # not stall the loop either.
        self._socket.settimeout(0)
        try:
            self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass

    def write(self, data: bytes) -> None:
        view = memoryview(data)
        deadline = time.monotonic() + self._timeout
        try:
            while view:
                try:
                    sent = self._socket.send(view)
                except (BlockingIOError, InterruptedError):
                    sent = 0
                if sent:
                    view = view[sent:]
                    continue
                if time.monotonic() >= deadline:
                    raise TransportError("timed out writing to the robot")
                time.sleep(0.002)
        except OSError as exc:
            raise TransportError(f"connection lost while writing: {exc}") from exc

    def read_line(self, deadline: float) -> Optional[str]:
        while True:
            newline = self._buffer.find(b"\n")
            if newline >= 0:
                raw, self._buffer = self._buffer[:newline], self._buffer[newline + 1 :]
                return raw.decode("ascii", "replace").rstrip("\r")

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None

            try:
                chunk = self._socket.recv(4096)
            except (BlockingIOError, InterruptedError):
                time.sleep(0.002)
                continue
            except OSError as exc:
                raise TransportError(f"connection lost while reading: {exc}") from exc

            if not chunk:
                raise TransportError("robot closed the connection")
            self._buffer += chunk

    def close(self) -> None:
        try:
            self._socket.close()
        except OSError:
            pass

    @property
    def description(self) -> str:
        return f"tcp://{self.host}:{self.port}"


class SerialTransport(Transport):
    """USB serial connection (115200 baud by default).

    The robot's telemetry stream is on by default over serial, so a caller that
    only wants command replies should send ``STREAM 0`` first; the client does
    this automatically in :meth:`Robot.connect`.
    """

    def __init__(
        self,
        port: str,
        baud: int = DEFAULT_BAUD,
        timeout: float = 3.0,
    ) -> None:
        try:
            import serial  # imported here so TCP users do not need pyserial
        except ImportError as exc:  # pragma: no cover - depends on environment
            raise TransportError(
                "serial support needs pyserial: pip install pyserial"
            ) from exc

        self.port = port
        self.baud = baud
        self._timeout = timeout
        self._buffer = b""
        try:
            self._serial = serial.Serial(port, baud, timeout=0.02)
        except (serial.SerialException, OSError) as exc:
            raise TransportError(f"cannot open serial port {port}: {exc}") from exc

    def write(self, data: bytes) -> None:
        try:
            self._serial.write(data)
            self._serial.flush()
        except OSError as exc:
            raise TransportError(f"serial write failed: {exc}") from exc

    def read_line(self, deadline: float) -> Optional[str]:
        while True:
            newline = self._buffer.find(b"\n")
            if newline >= 0:
                raw, self._buffer = self._buffer[:newline], self._buffer[newline + 1 :]
                return raw.decode("ascii", "replace").rstrip("\r")

            if time.monotonic() >= deadline:
                return None

            try:
                chunk = self._serial.read(4096)
            except OSError as exc:
                raise TransportError(f"serial read failed: {exc}") from exc
            if chunk:
                self._buffer += chunk

    def close(self) -> None:
        try:
            self._serial.close()
        except OSError:
            pass

    # Drop whatever the robot already queued (boot messages, telemetry) so the
    # first reply the caller sees belongs to the first command it sent.
    def flush_input(self) -> None:
        try:
            self._serial.reset_input_buffer()
        except OSError:
            pass
        self._buffer = b""

    @property
    def description(self) -> str:
        return f"serial://{self.port}@{self.baud}"


def list_serial_ports() -> list[str]:
    """Return the serial ports that look like a robot, best effort.

    Uses pyserial's enumeration when available and falls back to an empty list,
    so this is safe on machines without hardware attached.
    """
    try:
        from serial.tools import list_ports
    except ImportError:  # pragma: no cover - depends on environment
        return []
    return [port.device for port in list_ports.comports()]
