"""Tests for the reference Python client.

They run against a fake robot that speaks the firmware's exact protocol strings,
so they need no hardware and are fast enough for CI:

    cd examples/python && python -m unittest discover -s tests -v
"""

from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from crawlingbot import Robot, RobotError, RobotTimeout  # noqa: E402
from crawlingbot.protocol import (  # noqa: E402
    TELEMETRY_COLUMNS,
    ImuFrame,
    Snapshot,
    is_event,
    is_reply,
    parse_header,
    parse_reply,
    split_kv,
)
from fake_robot import HEADER, SNAP, FakeRobot, make_frame  # noqa: E402


class ProtocolParsingTests(unittest.TestCase):
    def test_split_kv_keeps_positional_tokens(self):
        self.assertEqual(split_kv("PONG 12345")["_positional"], "PONG 12345")
        values = split_kv("id=2,ssid=ESP32-AP-2")
        self.assertEqual(values["id"], "2")
        self.assertEqual(values["ssid"], "ESP32-AP-2")

    def test_reply_classification(self):
        self.assertTrue(is_reply("OK PING"))
        self.assertTrue(is_reply("ERR ACT_usage,ACT_<down>_<up>"))
        self.assertTrue(is_event("IMU,1,2,3"))
        self.assertTrue(is_event("CAL,gyro,starting"))
        self.assertTrue(is_event("CALRESULT,ok=1"))
        self.assertFalse(is_reply("IMU,1,2,3"))

    def test_parse_reply_raises_on_err(self):
        with self.assertRaises(RobotError) as caught:
            parse_reply("ERR SETID_range,1_to_8")
        self.assertEqual(caught.exception.code, "SETID_range")
        self.assertEqual(caught.exception.detail, "1_to_8")

    def test_snapshot_parses_every_field(self):
        snap = Snapshot.parse(SNAP)
        self.assertEqual(snap.robot_id, 1)
        self.assertAlmostEqual(snap.yaw, 97.05, places=2)
        self.assertAlmostEqual(snap.accel[2], 9.807, places=3)
        self.assertTrue(snap.calibrated)
        self.assertTrue(snap.still)
        self.assertEqual(snap.down_target, 180)
        self.assertFalse(snap.servo_moving)

    def test_header_round_trip(self):
        columns = parse_header(HEADER)
        self.assertIsNotNone(columns)
        self.assertEqual(columns[0], "t_ms")
        # The parser and the fixture must agree with the firmware's own column
        # list; a mismatch silently shifts every telemetry value.
        self.assertEqual(len(columns), len(TELEMETRY_COLUMNS))
        self.assertEqual(columns, TELEMETRY_COLUMNS)

    def test_frame_maps_columns_by_name(self):
        frame = ImuFrame.parse(make_frame(1234), parse_header(HEADER))
        self.assertEqual(frame.timestamp_ms, 1234)
        self.assertAlmostEqual(frame.accel[2], 9.8)
        self.assertAlmostEqual(frame.yaw, 97.0)
        self.assertAlmostEqual(frame.position[1], 2.0)
        self.assertAlmostEqual(frame.temperature_c, 31.2, places=1)


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.server = FakeRobot()
        self.robot = Robot.tcp("127.0.0.1", self.server.port, timeout=2.0)

    def tearDown(self):
        self.robot.close()
        self.server.close()

    def test_ping_returns_uptime(self):
        self.assertEqual(self.robot.ping(), 12345)

    def test_id_and_setid(self):
        info = self.robot.id_info()
        self.assertEqual(info["id"], "1")
        self.assertEqual(info["ssid"], "ESP32-AP-1")
        self.assertEqual(self.robot.set_id(4), 4)

    def test_setid_rejects_out_of_range_without_sending(self):
        with self.assertRaises(ValueError):
            self.robot.set_id(9)
        self.assertNotIn("SETID 9", self.server.received)

    def test_setid_surfaces_robot_error(self):
        # The client validates locally, so drive the transport directly to
        # check that a robot-side ERR becomes a RobotError.
        with self.assertRaises(RobotError) as caught:
            self.robot.command("SETID 99")
        self.assertEqual(caught.exception.code, "SETID_range")

    def test_snapshot_is_typed(self):
        snap = self.robot.snapshot()
        self.assertIsInstance(snap, Snapshot)
        self.assertAlmostEqual(snap.pitch, -0.31, places=2)

    def test_act_reports_clamped_targets(self):
        self.assertEqual(self.robot.act(300, -20), (180, 0))

    def test_act_rejects_non_numeric(self):
        with self.assertRaises(RobotError) as caught:
            self.robot.command("ACT abc 10")
        self.assertEqual(caught.exception.code, "ACT_usage")

    def test_stop_and_park(self):
        self.robot.stop()
        self.robot.park()
        self.assertEqual(self.server.received[-2:], ["STOP", "PARK"])

    def test_unknown_command_raises(self):
        with self.assertRaises(RobotError) as caught:
            self.robot.command("NOPE")
        self.assertEqual(caught.exception.code, "unknown_command")

    def test_stream_rate_is_clamped_by_the_robot(self):
        self.assertEqual(self.robot.stream(200), 50)

    def test_frames_skip_events_and_yield_typed_records(self):
        frames = list(self.robot.frames(hz=50, count=5))
        self.assertEqual(len(frames), 5)
        self.assertIsInstance(frames[0], ImuFrame)
        self.assertAlmostEqual(frames[0].yaw, 97.0)
        # The iterator turns streaming back off when it finishes.
        self.assertEqual(self.server.received[-1], "STREAM 0")

    def test_calibration_reports_progress_and_result(self):
        stages = []
        self.assertTrue(self.robot.calibrate(on_progress=stages.append))
        self.assertTrue(any("CAL,gyro" in line for line in stages))

    def test_calibration_failure_is_reported(self):
        server = FakeRobot(cal_ok=False)
        robot = Robot.tcp("127.0.0.1", server.port, timeout=2.0)
        try:
            self.assertFalse(robot.calibrate("gyro"))
        finally:
            robot.close()
            server.close()

    def test_timeout_when_robot_does_not_answer(self):
        with self.assertRaises(RobotTimeout):
            self.robot.command("PING", timeout=0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
