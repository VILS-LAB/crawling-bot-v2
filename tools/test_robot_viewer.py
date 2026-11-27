"""Tests for the robot visualizer.

Runs headless (Agg), needs no robot, and covers the parts that are easy to break
silently: the column contract, the decimation that must never hide a spike, the
health verdicts, the console's reply bookkeeping, and that the whole window
renders with every panel populated.

    MPLBACKEND=Agg python tools/test_robot_viewer.py
"""

from __future__ import annotations

import os
import queue
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use("Agg")

import numpy as np

from robot_viewer.app import App
from robot_viewer.config import Config
from robot_viewer.console import CommandConsole, action_set, stage_index
from robot_viewer.drawing import box_faces, compass_heading, nice_ceil, rot_matrix
from robot_viewer.health import FAIL, OK, UNKNOWN, WARN, HealthReport
from robot_viewer.logging_sink import Logger
from robot_viewer.panels import PANELS, RangePresets
from robot_viewer.telemetry import (
    CH,
    DEFAULT_FIELDS,
    N_CH,
    LineParser,
    Ring,
    finish_row,
)

TEMPERATURE = tempfile_dir = None


def make_row(t: float = 0.0, **values) -> np.ndarray:
    row = np.full(N_CH, np.nan)
    row[CH["t"]] = t
    for name, value in values.items():
        row[CH[name]] = value
    return finish_row(row)


def make_frame_line(**values) -> str:
    """A firmware-style CSV line built from the canonical column names."""
    defaults = {
        "t_ms": 1000,
        "dt": 5.0,
        "ax": 0.1, "ay": 0.0, "az": 9.8,
        "lax": 0.01, "lay": 0.02, "laz": 0.03,
        "flax": 0.011, "flay": 0.021, "flaz": 0.031,
        "gx": 0.5, "gy": -0.2, "gz": 0.1,
        "magx": 21.4, "magy": -8.0, "magz": 41.2,
        "roll": 1.2, "pitch": -0.3, "yaw": 97.0,
        "vx": 0.1, "vy": 0.0, "vz": 0.0,
        "px": 1.0, "py": 2.0, "pz": 0.0,
        "temp_c": 31.2, "moving": 0, "still": 1, "still_failure_mask": 0,
        "stdA": 0.001, "trust": 0.998, "rollG": 1.1, "pitchG": -0.4,
        "ebx": 0.01, "eby": 0.02, "ebz": 0.03,
        "acc_norm_g": 9.8, "mag_norm": 48.9, "rate_hz": 199.4,
        "accel_rms_g": 0.002, "mag_rms_ut": 0.9, "mag_fit": 2, "est_init": 1,
    }
    defaults.update(values)
    return "IMU," + ",".join(f"{defaults.get(name, 0):g}" for name in DEFAULT_FIELDS)


class ColumnContractTests(unittest.TestCase):
    def test_header_matches_the_parser_defaults(self):
        # The parser must accept the exact header the firmware prints.
        header = ",".join(DEFAULT_FIELDS)
        parser = LineParser("IMU")
        self.assertIsNone(parser.feed(header, 0.0))
        self.assertEqual(parser.columns, DEFAULT_FIELDS)

    def test_column_count_is_documented(self):
        # If this changes, docs/PROTOCOL.md and the Python client must change too.
        self.assertEqual(len(DEFAULT_FIELDS), 50)

    def test_parses_a_real_frame(self):
        parser = LineParser("IMU")
        row = parser.feed(make_frame_line(), time.monotonic())
        self.assertIsNotNone(row)
        self.assertAlmostEqual(row[CH["roll"]], 1.2)
        self.assertAlmostEqual(row[CH["yaw"]], 97.0)
        self.assertAlmostEqual(row[CH["a_norm"]], np.hypot(np.hypot(0.1, 0.0), 9.8))
        self.assertEqual(row[CH["still"]], 1.0)
        self.assertEqual(row[CH["still_on"]], 0.0)

    def test_extra_columns_are_ignored(self):
        parser = LineParser("IMU")
        row = parser.feed(make_frame_line() + ",99,99", time.monotonic())
        self.assertIsNotNone(row)
        self.assertAlmostEqual(row[CH["roll"]], 1.2)

    def test_short_line_is_rejected_with_a_reason(self):
        parser = LineParser("IMU")
        self.assertIsNone(parser.feed("IMU,1,2,3", 0.0))
        self.assertIn("expected at least", parser.last_reason)

    def test_derived_channels(self):
        row = make_row(
            vx=1.0, vy=0.0, vz=0.0, roll=10.0, pitch=-5.0, rollG=8.0, pitchG=-5.5,
            ebx=0.3, eby=0.4, ebz=0.0, still=1.0, still_failure_mask=0b101,
        )
        self.assertAlmostEqual(row[CH["speed"]], 1.0)
        self.assertAlmostEqual(row[CH["drift_roll"]], 2.0)
        self.assertAlmostEqual(row[CH["drift_pitch"]], 0.5)
        self.assertAlmostEqual(row[CH["bias_norm"]], 0.5)
        self.assertEqual(row[CH["fb0"]], 1.0)
        self.assertEqual(row[CH["fb1"]], 0.0)
        self.assertEqual(row[CH["fb2"]], 1.0)
        self.assertEqual(row[CH["fail_any"]], 1.0)


class RingTests(unittest.TestCase):
    def test_window_selects_by_time(self):
        ring = Ring(50)
        for i in range(20):
            ring.push(make_row(t=i * 0.1))
        window = ring.window(1.0)
        self.assertAlmostEqual(window[CH["t"]][0], 1.0)
        self.assertAlmostEqual(ring.last_time(), 1.9)

    def test_decimation_keeps_every_spike(self):
        ring = Ring(4000)
        for i in range(3000):
            spike = 50.0 if i == 1500 else 0.0
            ring.push(make_row(t=i * 0.004, flax=spike))
        _t, values = ring.decimate(200, [CH["flax"]])
        self.assertLess(values.shape[1], 3000)
        self.assertEqual(values[0].max(), 50.0, "a single-sample spike must survive")

    def test_decimate_one_row_per_channel(self):
        ring = Ring(500)
        for i in range(800):
            ring.push(make_row(t=i * 0.004, flax=float(i), flay=-float(i)))
        channels = [CH["flax"], CH["flay"]]
        _t, values = ring.decimate(100, channels)
        self.assertEqual(values.shape[0], len(channels))
        self.assertEqual(values[1].min(), -799.0)

    def test_empty_ring_is_safe(self):
        ring = Ring(10)
        self.assertIsNone(ring.window(0.0))
        self.assertEqual(ring.decimate(10, [CH["flax"]]), (None, None))


class PanelSpecTests(unittest.TestCase):
    def test_every_plotted_channel_exists(self):
        for panel in PANELS:
            for line in panel.lines:
                self.assertIn(line.field, CH, f"{panel.title}: unknown {line.field}")

    def test_presets_cover_every_range_key(self):
        needed = {p.range_key for p in PANELS if p.range_key and p.ylim is None}
        presets = RangePresets()
        for name in ("overview", "detail"):
            values = presets.get(name)
            self.assertTrue(needed <= set(values), f"{name} missing {needed - set(values)}")

    def test_preset_cycle_visits_every_name_once(self):
        presets = RangePresets()
        seen, name = [], "overview"
        for _ in range(len(presets.keys())):
            seen.append(name)
            name = presets.next_name(name)
        self.assertEqual(name, "overview", "the cycle must return to the start")
        self.assertEqual(len(set(seen)), len(seen))

    def test_panels_fit_the_grid(self):
        # The left grid is 4 rows x 2 columns: charts must not overlap.
        occupied = set()
        for panel in PANELS:
            row, col = panel.pos
            for offset in range(panel.colspan):
                self.assertNotIn((row, col + offset), occupied, "panels overlap")
                occupied.add((row, col + offset))


class HealthTests(unittest.TestCase):
    def test_good_boot_reports_all_ok(self):
        report = HealthReport()
        for line in (
            "PROTOCOL,READY,serial=115200,tcp_port=4242,stream_hz=10",
            "CAL,LOADED,robot=1",
            "CAL,SELF_CHECK,ok",
            "HEALTH,SERVO,down,pin=7,code=0,status=OK",
            "HEALTH,SERVO,up,pin=6,code=0,status=OK",
            "HEALTH,SUMMARY,ok=1,down=0,up=0",
        ):
            report.feed_line(line)
        report.feed_row(make_row(rate_hz=199.0, est_init=1.0, trust=1.0, acc_norm_g=1.0))
        icon, summary = report.verdict
        self.assertEqual(icon, "✓", summary)
        self.assertEqual(report.failures, [])

    def test_dead_servo_is_a_failure_with_a_hint(self):
        report = HealthReport()
        report.feed_line("HEALTH,SERVO,up,pin=6,code=4,status=NO_MOTION")
        check = report["servos"]
        self.assertEqual(check.state, FAIL)
        self.assertIn("no motion", check.detail.lower())
        self.assertIn("SERVO_HEALTH", check.hint)

    def test_one_failing_servo_wins_over_one_ok(self):
        report = HealthReport()
        report.feed_line("HEALTH,SERVO,down,pin=7,code=4,status=NO_MOTION")
        report.feed_line("HEALTH,SERVO,up,pin=6,code=0,status=OK")
        self.assertEqual(report["servos"].state, FAIL)

    def test_uncalibrated_robot_is_reported(self):
        report = HealthReport()
        report.feed_line("CAL,NONE,robot_runs_uncalibrated_until_CAL")
        report.feed_line("ROBOT,UNCALIBRATED,send_CAL_to_calibrate")
        self.assertEqual(report["calibration"].state, FAIL)
        self.assertIn("CAL", report["calibration"].hint)

    def test_imu_init_failure_maps_to_the_data_row(self):
        report = HealthReport()
        report.feed_line("ERROR,IMU_INIT_FAILED")
        self.assertEqual(report["data"].state, FAIL)

    def test_err_reply_downgrades_the_right_row(self):
        report = HealthReport()
        report.feed_reply("SETID 99", "ERR SETID_range,1_to_8")
        self.assertEqual(report["identity"].state, WARN)
        report.feed_reply("ACT 1 2", "OK ACT d=1 u=2")
        report.feed_reply("PING", "OK PONG 123")
        self.assertEqual(report["probe"].state, OK)

    def test_bad_sample_rate_warns(self):
        report = HealthReport()
        report.feed_row(make_row(rate_hz=40.0))
        self.assertEqual(report["rate"].state, WARN)

    def test_unknown_until_something_arrives(self):
        self.assertEqual(HealthReport().verdict[0], "·")


class ThroughputTests(unittest.TestCase):
    """The panel must explain a slow link instead of just showing a low number."""

    def test_requested_rate_is_read_from_the_stream_reply(self):
        report = HealthReport()
        report.feed_line("OK stream=50")
        self.assertEqual(report.requested_hz, 50.0)

    def test_a_throttled_link_is_reported(self):
        report = HealthReport()
        report.feed_line("OK stream=50")
        report.set("data", OK, "1 samples")
        report.set_achieved_rate(12.0)
        report.mark_data_seen(1)
        self.assertEqual(report["data"].state, WARN)
        self.assertIn("12 Hz of 50 requested", report["data"].detail)
        self.assertIn("TCP", report["data"].hint,
                      "the hint should point at the transport, not the firmware")

    def test_a_healthy_link_is_left_alone(self):
        report = HealthReport()
        report.feed_line("OK stream=50")
        report.set("data", OK, "1 samples")
        report.set_achieved_rate(49.0)
        self.assertEqual(report["data"].state, OK, "46/50 is fine, not throttled")

    def test_the_warning_survives_subsequent_frames(self):
        """Regression: the warning was cleared by the next frame's data reset."""
        report = HealthReport()
        report.feed_line("OK stream=50")
        report.set_achieved_rate(12.0)
        for _ in range(5):
            report.mark_data_seen(7)      # what app._drain calls per frame
        self.assertEqual(report["data"].state, WARN, "the throttle warning vanished")

    def test_the_warning_clears_when_the_link_recovers(self):
        report = HealthReport()
        report.feed_line("OK stream=50")
        report.set_achieved_rate(12.0)
        report.mark_data_seen(7)
        report.set_achieved_rate(48.0)
        report.mark_data_seen(7)
        self.assertEqual(report["data"].state, OK)

    def test_no_verdict_before_a_stream_request(self):
        # Without a STREAM reply there is nothing to compare against, so the
        # panel must not invent a warning.
        report = HealthReport()
        report.set("data", OK, "1 samples")
        report.set_achieved_rate(5.0)
        self.assertEqual(report["data"].state, OK)


class ConsoleTests(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.console = CommandConsole(self.sent.append)
        self.actions = {a.key: a for a in action_set((120, 60))}

    def test_every_protocol_command_has_a_button(self):
        expected = {"ping", "id", "get", "setid", "stream_on", "stream_off",
                    "act", "stop", "park", "cal"}
        self.assertEqual(set(self.actions), expected)

    def test_act_uses_the_slider_values(self):
        self.console.run(self.actions["act"])
        self.assertEqual(self.sent[-1], "ACT 120 60")

    def test_act_follows_the_sliders_after_they_move(self):
        """Regression: ACT sent the angles captured when the buttons were built.

        The window built its actions once with ``action_set((90, 90))``, so
        dragging a slider changed the display but every ACT still sent 90 90.
        """
        legs = [90, 90]
        actions = {a.key: a for a in action_set(lambda: (legs[0], legs[1]))}
        self.assertEqual(actions["act"].build(), "ACT 90 90")

        legs[0], legs[1] = 150, 30
        self.assertEqual(actions["act"].build(), "ACT 150 30",
                         "ACT must read the sliders at click time")

    def test_act_rounds_fractional_slider_values(self):
        # Sliders with any valstep can report 89.99999; the protocol wants ints.
        actions = {a.key: a for a in action_set(lambda: (89.9999, 0.0001))}
        self.assertEqual(actions["act"].build(), "ACT 90 0")

    def test_only_one_command_in_flight(self):
        self.assertIsNotNone(self.console.run(self.actions["ping"]))
        self.assertIsNone(self.console.run(self.actions["get"]),
                          "a second command must not be sent before the reply")

    def test_reply_clears_the_pending_state(self):
        self.console.run(self.actions["ping"])
        result = self.console.feed_line("OK PONG 5")
        self.assertTrue(result.ok)
        self.assertEqual(result.command, "PING")
        self.assertIsNotNone(self.console.run(self.actions["get"]))

    def test_err_reply_is_not_ok(self):
        self.console.run(self.actions["act"])
        self.assertFalse(self.console.feed_line("ERR ACT_usage,nope").ok)

    def test_timeout_is_reported_once(self):
        self.console.run(self.actions["ping"])
        self.console._deadline = time.monotonic() - 1
        self.assertFalse(self.console.poll_timeout().ok)
        self.assertIsNone(self.console.poll_timeout(), "timeout must fire once")

    def test_calibration_result_before_reply(self):
        self.console.run(self.actions["cal"])
        result = self.console.feed_cal_result("CALRESULT,ok=1,sensors=all")
        self.assertTrue(result.ok)

    def test_event_lines_do_not_complete_a_command(self):
        self.console.run(self.actions["get"])
        self.assertIsNone(self.console.feed_line("IMU,1,2,3"))
        self.assertIsNone(self.console.feed_line("CAL,gyro,starting"))

    def test_stage_index(self):
        self.assertEqual(stage_index("CAL,gyro,starting"), 0)
        self.assertEqual(stage_index("CAL,accelerometer,starting"), 1)
        self.assertEqual(stage_index("CAL,magnetometer,starting"), 2)
        self.assertEqual(stage_index("CAL,START"), 0, "START begins the first stage")


class SourceRoutingTests(unittest.TestCase):
    """Replies and diagnostics must survive the trip from wire to console.

    Regression: ``Source.handle_line`` fed every line to the telemetry parser, so
    a reply like ``OK PONG 12345`` was reported as a malformed data row, dropped,
    and the console showed "TIMEOUT (no reply)" for commands the robot had
    already obeyed.
    """

    def _source(self):
        from robot_viewer.sources import Source

        out = queue.SimpleQueue()

        class S(Source):
            accepts_commands = True

            def send(self, command):
                self.last_tx = command
                self.out_q.put((command + "\n").encode())

            def start(self):
                pass

        return S(out, LineParser("IMU"), Config()), out

    def _drain(self, out):
        items = []
        while True:
            try:
                items.append(out.get_nowait())
            except queue.Empty:
                return items

    def test_reply_reaches_the_console(self):
        source, out = self._source()
        console = CommandConsole(source.send)
        console.run(action_set([90, 90])[0])          # PING

        source.handle_line("OK PONG 12345")
        self.assertIn("OK PONG 12345", self._drain(out))
        # The app feeds queued text back into the console; do that here.
        self.assertIsNotNone(console.feed_line("OK PONG 12345"))
        self.assertEqual(console._pending, None, "the console is still waiting for a reply")

    def test_reply_and_telemetry_can_interleave(self):
        source, out = self._source()
        console = CommandConsole(source.send)
        console.run(action_set([90, 90])[4])          # STREAM 50

        frame = "IMU," + ",".join(["0"] * 50)
        for line in (frame, "OK stream=50", frame):
            source.handle_line(line)

        items = self._drain(out)
        texts = [i for i in items if isinstance(i, str)]
        rows = [i for i in items if not isinstance(i, str)]
        self.assertIn("OK stream=50", texts)
        self.assertEqual(len(rows), 2, "both telemetry rows must still plot")
        self.assertIsNotNone(console.feed_line("OK stream=50"))
        self.assertEqual(console._pending, None)

    def test_diagnostics_are_not_parsed_as_telemetry(self):
        source, out = self._source()
        for line in ("PROTOCOL,READY,serial=115200", "HEALTH,SUMMARY,ok=1",
                     "CAL,START", "#" + ",".join(DEFAULT_FIELDS)):
            source.handle_line(line)
        texts = [i for i in self._drain(out) if isinstance(i, str)]
        self.assertIn("PROTOCOL,READY,serial=115200", texts)
        self.assertIn("HEALTH,SUMMARY,ok=1", texts)
        self.assertIn("CAL,START", texts)
        self.assertEqual(source.rejected, 0, "no diagnostic may count as a rejected frame")

    def test_malformed_telemetry_is_still_reported(self):
        source, out = self._source()
        source.handle_line("IMU,1,2,not_a_number")
        self.assertEqual(source.rejected, 1)
        self.assertIn("IMU,1,2,not_a_number", source.last_reject)
        self.assertEqual([i for i in self._drain(out) if isinstance(i, str)], [])

    def test_console_times_out_when_the_robot_is_silent(self):
        source, _out = self._source()
        console = CommandConsole(source.send)
        console.run(action_set([90, 90])[0])
        console._deadline = time.monotonic() - 1.0
        self.assertIsNotNone(console.poll_timeout())
        self.assertEqual(console._pending, None, "a timeout must clear the pending command")

    def test_throughput_keeps_up_with_the_stream(self):
        """A 440-byte line at 50 Hz must not be dropped by the parser."""
        source, out = self._source()
        frame = "IMU," + ",".join(["0"] * 50)
        for _ in range(500):
            source.handle_line(frame)
        rows = [i for i in self._drain(out) if not isinstance(i, str)]
        self.assertEqual(len(rows), 500)
        self.assertEqual(source.rejected, 0)


class AutoStreamTests(unittest.TestCase):
    """The viewer must ask for a stream, because the robot will not offer one.

    Regression: a fresh TCP client is accepted with ``streamHz = 0`` and a serial
    port runs whatever the last session left, so opening the viewer without
    pressing "STREAM 50" showed an empty window while `nc` worked fine.
    """

    class Source:
        accepts_commands = True
        state = "test"
        last_rx_at = None

        def __init__(self):
            self.sent = []
            self.rx_rows = 0
            self.streaming = False

        def send(self, command):
            self.sent.append(command)
            if command.startswith("STREAM"):
                self.streaming = command.split()[1] != "0"
            return True

        def start(self):
            pass

        def stop(self):
            pass

    def _app(self, announcement=None, streaming=False):
        import matplotlib.pyplot as plt

        source = self.Source()
        source.streaming = streaming
        out = queue.SimpleQueue()
        app = App(Config(), source, out, None, "TEST")
        if announcement is not None:
            out.put(f"PROTOCOL,READY,serial=115200,tcp_port=4242,stream_hz={announcement}")
        self._parser = LineParser("IMU")
        self._out = out
        self.addCleanup(plt.close, app.fig)
        return app, source

    def _run(self, app, source, ticks=60, dt=0.1):
        now = time.monotonic()
        for _ in range(ticks):
            if source.streaming:
                self._out.put(self._parser.feed("IMU," + ",".join(["0"] * 50), now))
                source.rx_rows += 1
            app._ensure_stream(now)
            now += dt
        return now

    def test_asks_a_quiet_tcp_client_to_stream(self):
        app, source = self._app(announcement=0)
        self._run(app, source)
        self.assertEqual(source.sent, ["STREAM 50"])

    def test_leaves_an_existing_stream_alone(self):
        # A serial port already at 10 Hz must not be overridden behind the
        # user's back; 10 Hz of data plotted beats a surprise 50 Hz.
        app, source = self._app(announcement=10, streaming=True)
        self._run(app, source)
        self.assertEqual(source.sent, [], "it should adopt the running stream")
        self.assertTrue(app._stream_settled)

    def test_asks_when_the_firmware_does_not_announce(self):
        app, source = self._app(announcement=None)
        self._run(app, source, ticks=80)
        self.assertEqual(source.sent, ["STREAM 50"])

    def test_asks_only_once(self):
        app, source = self._app(announcement=0)
        self._run(app, source, ticks=120)
        self.assertEqual(source.sent, ["STREAM 50"], "no repeated nagging")

    def test_waits_for_the_grace_period(self):
        app, source = self._app(announcement=None)
        now = time.monotonic()
        app._ensure_stream(now)              # immediately after startup
        self.assertEqual(source.sent, [], "the robot gets a moment to announce itself")

    def test_does_not_steal_a_pending_reply(self):
        app, source = self._app(announcement=0)
        app.console._pending = "PING"        # a button was just pressed
        app._ensure_stream(time.monotonic() + 10)
        self.assertEqual(source.sent, [], "the STREAM ask must wait for the reply")

    def test_can_be_disabled(self):
        source = self.Source()
        cfg = Config()
        cfg.auto_stream = False
        app = App(cfg, source, queue.SimpleQueue(), None, "TEST")
        self.addCleanup(__import__("matplotlib.pyplot", fromlist=["x"]).close, app.fig)
        app._ensure_stream(time.monotonic() + 10)
        self.assertEqual(source.sent, [])


class DrawingTests(unittest.TestCase):
    def test_heading_uses_the_documented_offset(self):
        # yaw 0 -> the nose (body +Y) points west, which is 270 on the dial.
        self.assertAlmostEqual(compass_heading(0.0), 270.0)
        self.assertAlmostEqual(compass_heading(90.0), 180.0)
        self.assertAlmostEqual(compass_heading(-90.0), 0.0)

    def test_identity_rotation_keeps_axes(self):
        rotation = rot_matrix(0.0, 0.0, 0.0)
        # Body +Y (the nose) maps to display -X, i.e. west. That is dial 270 and
        # matches compass_heading(0), so the box and the dial always agree.
        nose = rotation @ np.array([0.0, 1.0, 0.0])
        self.assertAlmostEqual(nose[0], -1.0, places=6)
        self.assertAlmostEqual(compass_heading(0.0), 270.0)

    def test_box_has_six_distinct_faces(self):
        faces = box_faces(np.eye(3), (1, 1, 1))
        self.assertEqual(len(faces), 6)
        centres = [face.mean(axis=0) for face in faces]
        self.assertEqual(len({tuple(np.round(c, 6)) for c in centres}), 6)

    def test_nice_ceil(self):
        self.assertEqual(nice_ceil(1.01), 1.2)
        self.assertEqual(nice_ceil(0.0), None)


class LoggerTests(unittest.TestCase):
    def test_round_trip_through_the_parser(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as folder:
            logger = Logger(folder, "IMU")
            parser = LineParser("IMU")
            source_rows = []
            for i in range(5):
                row = parser.feed(make_frame_line(t_ms=1000 + 5 * i), time.monotonic())
                logger.write_row(row)
                source_rows.append(row)
            logger.close()

            replay = LineParser("IMU")
            replayed = [
                replay.feed(line, time.monotonic())
                for line in Path(logger.path).read_text().splitlines()
            ]
            replayed = [row for row in replayed if row is not None]
            self.assertEqual(len(replayed), len(source_rows))
            self.assertAlmostEqual(replayed[0][CH["roll"]], source_rows[0][CH["roll"]])
            self.assertAlmostEqual(replayed[-1][CH["t"]], source_rows[-1][CH["t"]], places=2)


class WindowRenderTests(unittest.TestCase):
    """The window must build and draw with plausible data in every panel."""

    @classmethod
    def setUpClass(cls):
        import matplotlib.pyplot as plt

        cls.plt = plt
        cfg = Config()
        cfg.use_blit = True
        cls.out = queue.SimpleQueue()

        class Source:
            # A stub: the test feeds the queue directly, so nothing runs.
            rx_rows = 0
            last_rx_at = None
            state = "test"
            accepts_commands = True

            def send(self, command):
                Source.last_command = command
                return True

            def stop(self):
                pass

        cls.source = Source()
        cls.app = App(cfg, cls.source, cls.out, None, "TEST")

        parser = LineParser("IMU")
        for i in range(400):
            phase = (i * 0.005) % 12.0
            cls.out.put(parser.feed(
                make_frame_line(
                    t_ms=int(i * 5),
                    roll=10.0 * np.sin(phase),
                    pitch=5.0 * np.cos(phase),
                    yaw=((phase * 30 + 180) % 360) - 180,
                    flax=2.0 * np.sin(3 * phase),
                    fgx=20.0 * np.sin(2 * phase),
                    vx=0.2 * np.sin(phase),
                    px=0.3 * np.sin(phase),
                    py=0.2 * np.cos(phase),
                    ebx=0.01 * np.sin(phase),
                    still=0.0 if 2 < phase < 6 else 1.0,
                    moving=1.0 if 2 < phase < 6 else 0.0,
                    still_failure_mask=0.0 if phase > 6 else 0b100,
                ),
                time.monotonic(),
            ))
        for line in (
            "PROTOCOL,READY,serial=115200,tcp_port=4242,stream_hz=10",
            "CAL,LOADED,robot=1",
            "CAL,SELF_CHECK,ok",
            "HEALTH,SERVO,down,pin=7,code=0,status=OK",
            "HEALTH,SERVO,up,pin=6,code=0,status=OK",
            "HEALTH,SUMMARY,ok=1,down=0,up=0",
        ):
            cls.out.put(line)
        for _ in range(40):
            cls.app.tick()

    def test_figure_has_every_axes(self):
        # 6 chart axes + top-down + 3D + dial + health + state = 11
        self.assertGreaterEqual(len(self.app.fig.axes), 11)

    def test_all_chart_lines_have_data(self):
        for panel, _ax, _right, artists, _clip in self.app.plots:
            for line, artist in artists:
                xdata = artist.get_xdata()
                self.assertGreater(len(xdata), 0, f"{panel.title}/{line.label} has no data")

    def test_health_panel_shows_a_verdict(self):
        self.assertTrue(self.app.t_verdict.get_text())
        self.assertEqual(self.app.health.failures, [])

    def test_health_headline_does_not_cover_a_row(self):
        # Regression: the verdict was drawn on top of the first three rows.
        self.app.fig.canvas.draw()
        renderer = self.app.fig.canvas.get_renderer()
        verdict = self.app.t_verdict.get_window_extent(renderer)
        for key, _icon, label, _detail in self.app.health_rows:
            self.assertFalse(
                verdict.overlaps(label.get_window_extent(renderer)),
                f"the health headline overlaps the '{key}' row",
            )

    def test_health_rows_stay_inside_their_axes(self):
        # Spacing is derived from the row count, so a new check must still fit.
        self.app.fig.canvas.draw()
        renderer = self.app.fig.canvas.get_renderer()
        axes = self.app.t_verdict.axes.get_window_extent(renderer)
        for key, _icon, label, _detail in self.app.health_rows:
            box = label.get_window_extent(renderer)
            self.assertGreaterEqual(box.y0, axes.y0, f"'{key}' row spills below the panel")
            self.assertLessEqual(box.y1, axes.y1, f"'{key}' row spills above the panel")

    def test_topdown_legend_clears_the_trail(self):
        # The legend used to sit on top of the newest part of the path, and
        # moving it above the axes collided with the title instead.
        self.app.fig.canvas.draw()
        renderer = self.app.fig.canvas.get_renderer()
        axis = self.app.ax_td
        legend = axis.get_legend().get_window_extent(renderer)
        self.assertFalse(legend.overlaps(axis.get_window_extent(renderer)),
                         "the legend covers the plotted trail")
        self.assertFalse(legend.overlaps(axis.title.get_window_extent(renderer)),
                         "the legend collides with the panel title")

    def test_console_has_a_button_per_action(self):
        self.assertEqual(len(self.app.buttons), 10)

    def test_window_draws_to_a_png(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / "view.png"
            self.app.fig.canvas.draw()
            self.app.fig.savefig(out, dpi=90)
            self.assertGreater(out.stat().st_size, 20_000)

    def test_button_click_sends_the_command(self):
        self.app._on_action("ping")
        self.assertEqual(self.source.last_command, "PING")

    def test_setid_validates_locally(self):
        self.app.tb_setid.set_val("99")
        self.app._on_action("setid")
        self.assertNotEqual(getattr(self.source, "last_command", ""), "SETID 99")

    def test_preset_and_window_keys(self):
        start = self.app.preset
        self.app._on_key(type("E", (), {"key": "m"})())
        self.assertNotEqual(self.app.preset, start)
        width = self.app.cfg.window_steps[self.app.window_index]
        self.app._on_key(type("E", (), {"key": "["})())
        self.assertLessEqual(self.app.cfg.window_steps[self.app.window_index], width)

    def test_zero_position_resets_the_trail(self):
        self.app._on_key(type("E", (), {"key": "z"})())
        self.assertTrue(self.app.zeroed)

    @classmethod
    def tearDownClass(cls):
        cls.app.shutdown()
        cls.plt.close("all")


if __name__ == "__main__":
    unittest.main(verbosity=2)
