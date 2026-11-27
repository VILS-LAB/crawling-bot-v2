"""Reference Python client for the Crawling Bot V2 command protocol.

    >>> from crawlingbot import Robot
    >>> with Robot.tcp("192.168.4.1") as bot:
    ...     print(bot.ping())
    ...     print(bot.snapshot().roll)
    ...     bot.act(120, 60)

The protocol itself is documented in ``docs/PROTOCOL.md``. This package only
depends on the standard library; ``pyserial`` is imported lazily and is needed
solely for serial connections.
"""

from .client import Robot
from .protocol import ImuFrame, RobotError, RobotTimeout, Snapshot
from .transport import SerialTransport, TcpTransport, list_serial_ports

__version__ = "1.0.0"

__all__ = [
    "Robot",
    "ImuFrame",
    "Snapshot",
    "RobotError",
    "RobotTimeout",
    "SerialTransport",
    "TcpTransport",
    "list_serial_ports",
]
