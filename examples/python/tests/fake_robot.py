"""A fake robot that speaks the firmware's protocol, for offline tests.

It answers with the same literal strings lib/CommandServer emits, and can
interleave telemetry and calibration progress so the client's event/reply
separation is exercised.
"""

from __future__ import annotations

import socket
import threading
import time
from typing import List, Optional

SNAP = (
    "OK SNAP,id=1,t=51402,roll=1.24,pitch=-0.31,yaw=97.05,"
    "ax=0.104,ay=0.021,az=9.807,gx=0.12,gy=-0.07,gz=0.03,"
    "mx=21.44,my=-8.02,mz=41.19,temp_c=31.25,"
    "still=1,trust=0.998,rate_hz=199.4,cal=1,"
    "d=180,dt=180,u=180,ut=180,act_moving=0"
)

HEADER = (
    "IMU,t_ms,dt,ax,ay,az,lax,lay,laz,flax,flay,flaz,gx,gy,gz,gbx,gby,gbz,"
    "fgx,fgy,fgz,magx,magy,magz,roll,pitch,yaw,vx,vy,vz,px,py,pz,temp_c,"
    "moving,still,still_failure_mask,stdA,trust,rollG,pitchG,"
    "ebx,eby,ebz,acc_norm_g,mag_norm,rate_hz,accel_rms_g,mag_rms_ut,mag_fit,"
    "est_init"
)


def make_frame(t_ms: int = 1000) -> str:
    """Build one telemetry line from the canonical column list.

    Generating it from names (instead of hard-coded indices) keeps the fixture
    correct if the column order ever changes.
    """
    from crawlingbot.protocol import TELEMETRY_COLUMNS

    values = {
        "t_ms": t_ms,
        "dt": 0.005,
        "ax": 0.1, "ay": 0.0, "az": 9.8,
        "lax": 0.01, "lay": 0.02, "laz": 0.03,
        "flax": 0.011, "flay": 0.021, "flaz": 0.031,
        "gx": 0.5, "gy": -0.2, "gz": 0.1,
        "magx": 21.4, "magy": -8.0, "magz": 41.2,
        "roll": 1.2, "pitch": -0.3, "yaw": 97.0,
        "vx": 0.1, "vy": 0.0, "vz": 0.0,
        "px": 1.0, "py": 2.0, "pz": 0.0,
        "temp_c": 31.2,
        "moving": 0, "still": 1, "still_failure_mask": 0,
        "stdA": 0.001, "trust": 0.998, "rollG": 1.1, "pitchG": -0.4,
        "rate_hz": 199.4, "mag_norm": 48.9, "acc_norm_g": 9.8,
        "accel_rms_g": 0.002, "mag_rms_ut": 0.9,
        "mag_fit": 2, "est_init": 1,
    }
    parts = []
    for name in TELEMETRY_COLUMNS:
        parts.append(f"{values.get(name, 0):g}")
    return "IMU," + ",".join(parts)


class FakeRobot:
    """Serves the protocol on 127.0.0.1:<port> in a background thread."""

    def __init__(self, port: int = 0, cal_ok: bool = True) -> None:
        self._cal_ok = cal_ok
        self.received: List[str] = []
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("127.0.0.1", port))
        self.port = self._server.getsockname()[1]
        self._server.listen(4)
        self._conn: Optional[socket.socket] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    # -- helpers -----------------------------------------------------------

    def _send(self, text: str) -> None:
        if self._conn is not None:
            try:
                self._conn.sendall(text.encode("ascii") + b"\n")
            except OSError:
                pass

    def _handle(self, command: str) -> None:
        self.received.append(command)
        verb = command.split()[0].upper() if command.split() else ""
        rest = command[len(verb) :].strip()

        if verb == "PING":
            self._send("OK PONG 12345")
        elif verb == "ID":
            self._send("OK id=1 ssid=ESP32-AP-1 ip=192.168.4.1")
        elif verb == "SETID":
            try:
                number = int(rest)
            except ValueError:
                self._send("ERR SETID_range,1_to_8")
            else:
                if 1 <= number <= 8:
                    self._send(f"OK id={number}")
                else:
                    self._send("ERR SETID_range,1_to_8")
        elif verb == "GET":
            self._send(SNAP)
        elif verb == "STREAM":
            try:
                hz = int(rest)
            except ValueError:
                self._send("ERR STREAM_range,0_to_50")
                return
            hz = max(0, min(50, hz))
            self._send(f"OK stream={hz}")
            if hz:
                self._send(HEADER)
                for index in range(20):
                    self._send(make_frame(1000 + index * 20))
        elif verb == "ACT":
            parts = rest.split()
            try:
                down = max(0, min(180, int(parts[0])))
                up = max(0, min(180, int(parts[1])))
            except (IndexError, ValueError):
                self._send("ERR ACT_usage,ACT_<down>_<up>")
                return
            self._send(f"OK ACT d={down} u={up}")
        elif verb == "STOP":
            self._send("OK STOP")
        elif verb == "PARK":
            self._send("OK PARK")
        elif verb == "CAL":
            target = rest or "all"
            if target not in ("all", "gyro", "accel", "mag"):
                self._send("ERR CAL_usage,CAL_[all|gyro|accel|mag]")
                return
            self._send(f"OK CAL sensors={target}")
            self._send("CAL,START,robot=1")
            self._send("CAL,gyro,starting")
            self._send("IMU,1,0.005,0.1,0.0,9.8")
            ok = 1 if self._cal_ok else 0
            self._send(f"CALRESULT,ok={ok},sensors={target}")
        else:
            self._send("ERR unknown_command,use_PING_ID_SETID_GET_STREAM_ACT_STOP_PARK_CAL")

    def _serve(self) -> None:
        try:
            conn, _ = self._server.accept()
        except OSError:
            return
        self._conn = conn
        buffer = b""
        while not self._stop.is_set():
            try:
                chunk = conn.recv(4096)
            except OSError:
                break
            if not chunk:
                break
            buffer += chunk
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if line:
                    self._handle(line)
        try:
            conn.close()
        except OSError:
            pass

    def close(self) -> None:
        self._stop.set()
        for sock in (self._conn, self._server):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass

    def __enter__(self) -> "FakeRobot":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()
