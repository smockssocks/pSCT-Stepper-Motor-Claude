"""Tests for the two LabVIEW-facing interfaces and the CLI.

The socket bridge is the route LabVIEW is expected to use, so it gets the same
scrutiny as the driver: malformed input must produce a well-formed error
rather than a dropped connection, and `stop` must get through while a move is
still running.
"""

import json
import os
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from psct_motors import labview_api  # noqa: E402
from psct_motors.config import BrakeConfig, PlatformLimits, default_config  # noqa: E402
from psct_motors.platform import FocalPlanePlatform  # noqa: E402
from psct_motors.server import (  # noqa: E402
    CommandDispatcher, PlatformServer, send_command,
)


def bench_config():
    cfg = default_config()
    for a in cfg.actuators:
        a.counts_per_mm = 1000.0
        a.velocity_raw = 8000
        a.min_travel_mm = 0.0
        a.max_travel_mm = 50.0
        a.in_position_tol_mm = 0.01
        a.move_timeout_s = 20.0
        a.brake = BrakeConfig(mode="output", settle_s=0.0)
    cfg.limits = PlatformLimits(min_focus_mm=0.5, max_focus_mm=49.5,
                                max_tilt_deg=1.0, max_step_mm=20.0,
                                max_tilt_step_deg=1.0)
    cfg.validate()
    return cfg


class TestServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.platform = FocalPlanePlatform(cfg=bench_config(), simulate=True)
        cls.platform.connect()
        cls.dispatcher = CommandDispatcher(cls.platform)
        cls.server = PlatformServer(("127.0.0.1", 0), cls.dispatcher)
        cls.port = cls.server.server_address[1]
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.platform.disconnect()

    def call(self, command, args=None, timeout_s=30.0):
        return send_command(command, args, host="127.0.0.1", port=self.port,
                            timeout_s=timeout_s)

    # ---- basics ----------------------------------------------------------

    def test_ping(self):
        reply = self.call("ping")
        self.assertTrue(reply["ok"])
        self.assertEqual(reply["result"]["actuators"], ["A", "B", "C"])
        self.assertTrue(reply["result"]["simulated"])

    def test_status_shape(self):
        reply = self.call("status")
        self.assertTrue(reply["ok"])
        result = reply["result"]
        self.assertEqual(len(result["motors"]), 3)
        self.assertTrue(result["orientation_valid"])
        self.assertIn("focus_mm", result["orientation"])

    def test_orientation(self):
        reply = self.call("orientation")
        self.assertTrue(reply["ok"])
        self.assertIn("total_tilt_arcsec", reply["result"])

    def test_move_and_read_back(self):
        reply = self.call("move", {"focus_mm": 28.0, "tip_deg": 0.05})
        self.assertTrue(reply["ok"], reply.get("error"))
        got = self.call("orientation")["result"]
        self.assertAlmostEqual(got["focus_mm"], 28.0, places=2)
        self.assertAlmostEqual(got["tip_deg"], 0.05, places=4)

    def test_move_relative(self):
        before = self.call("orientation")["result"]["focus_mm"]
        reply = self.call("move_relative", {"d_focus_mm": -1.0})
        self.assertTrue(reply["ok"], reply.get("error"))
        after = self.call("orientation")["result"]["focus_mm"]
        self.assertAlmostEqual(after, before - 1.0, places=2)

    def test_move_polar(self):
        reply = self.call("move_polar", {"focus_mm": 25.0, "total_tilt_deg": 0.1,
                                         "azimuth_deg": 30.0})
        self.assertTrue(reply["ok"], reply.get("error"))
        got = self.call("orientation")["result"]
        self.assertAlmostEqual(got["total_tilt_deg"], 0.1, places=3)
        self.assertAlmostEqual(got["tilt_azimuth_deg"], 30.0, places=1)

    def test_preview_does_not_move(self):
        before = self.call("orientation")["result"]["focus_mm"]
        reply = self.call("preview", {"focus_mm": 40.0})
        self.assertTrue(reply["ok"])
        self.assertEqual(len(reply["result"]["actuator_targets_mm"]), 3)
        after = self.call("orientation")["result"]["focus_mm"]
        self.assertAlmostEqual(before, after, places=4)

    def test_preview_reports_limit_violation_without_failing(self):
        reply = self.call("preview", {"focus_mm": 999.0})
        self.assertTrue(reply["ok"])          # the query succeeded ...
        self.assertFalse(reply["result"]["within_limits"])   # ... the move would not
        self.assertIn("focus", reply["result"]["limit_message"])

    def test_brake_status_and_control(self):
        self.assertTrue(self.call("brake", {"action": "engage"})["ok"])
        states = self.call("brake", {"action": "status"})["result"]
        self.assertEqual(set(states.values()), {"engaged"})

    # ---- error handling --------------------------------------------------

    def test_unknown_command_lists_alternatives(self):
        reply = self.call("teleport")
        self.assertFalse(reply["ok"])
        self.assertIn("Unknown command", reply["error"])
        self.assertIn("move", reply["error"])

    def test_missing_command_field(self):
        reply = self._raw('{"args": {}}')
        self.assertFalse(reply["ok"])
        self.assertIn("no 'command'", reply["error"])

    def test_malformed_json_gets_an_answer_not_a_dropped_socket(self):
        reply = self._raw("{this is not json")
        self.assertFalse(reply["ok"])
        self.assertIn("Malformed", reply["error"])

    def test_bad_arguments_are_explained(self):
        reply = self.call("move", {"nonsense": 1})
        self.assertFalse(reply["ok"])
        self.assertIn("Bad arguments", reply["error"])

    def test_refused_move_is_a_clean_error(self):
        reply = self.call("move", {"focus_mm": 999.0})
        self.assertFalse(reply["ok"])
        self.assertIn("Move refused", reply["error"])

    def test_connection_survives_a_bad_request(self):
        """One bad line must not end the session."""
        with socket.create_connection(("127.0.0.1", self.port), timeout=10) as sock:
            f = sock.makefile("rwb")
            f.write(b"garbage\n")
            f.flush()
            first = json.loads(f.readline())
            self.assertFalse(first["ok"])
            f.write(b'{"command": "ping"}\n')
            f.flush()
            second = json.loads(f.readline())
            self.assertTrue(second["ok"])

    def _raw(self, text, timeout=10.0):
        with socket.create_connection(("127.0.0.1", self.port), timeout=timeout) as sock:
            sock.sendall((text + "\n").encode())
            buf = b""
            while not buf.endswith(b"\n"):
                chunk = sock.recv(65536)
                if not chunk:
                    break
                buf += chunk
        return json.loads(buf.decode())

    # ---- the property that matters --------------------------------------

    def test_stop_gets_through_while_a_move_is_running(self):
        """A stop from a second connection must not queue behind the move.

        The bench actuators are configured fast enough that a normal move
        finishes in milliseconds, which would make this test pass without
        testing anything. Slow them right down so there is a real move in
        flight for the stop to interrupt.
        """
        self.call("move", {"focus_mm": 5.0})          # go to one end first

        original = [a.velocity_raw for a in self.platform.cfg.actuators]
        for actuator in self.platform.cfg.actuators:
            actuator.velocity_raw = 20              # ~2 mm/s in the simulator
        try:
            results = {}

            def slow_move():
                results["move"] = self.call("move", {"focus_mm": 20.0, "wait": True},
                                            timeout_s=60.0)

            mover = threading.Thread(target=slow_move, daemon=True)
            mover.start()
            time.sleep(1.0)                          # let the move get going

            # It really is still running, so the stop below has something to do.
            in_flight = self.call("orientation")["result"]["focus_mm"]
            self.assertLess(in_flight, 19.0, "the move finished before it was stopped")

            started = time.monotonic()
            stop_reply = self.call("stop", timeout_s=15.0)
            elapsed = time.monotonic() - started

            self.assertTrue(stop_reply["ok"], stop_reply.get("error"))
            self.assertLess(elapsed, 5.0,
                            "stop had to wait for the move to finish, which defeats it")

            mover.join(timeout=30)
            # The interrupted move reports failure rather than claiming success.
            self.assertFalse(results["move"]["ok"])
            self.assertIn("STOP", results["move"]["error"])
            final = self.call("orientation")["result"]["focus_mm"]
            self.assertLess(final, 19.0, "the move should have been cut short")
        finally:
            for actuator, speed in zip(self.platform.cfg.actuators, original):
                actuator.velocity_raw = speed


class TestLabviewApi(unittest.TestCase):
    """The Python-node route. Nothing here may raise."""

    def setUp(self):
        self._cfg_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "_tmp_lv_config.json"
        )
        from psct_motors.config import save_config
        save_config(bench_config(), self._cfg_path)
        reply = json.loads(labview_api.lv_open(self._cfg_path, 1))
        self.assertTrue(reply["ok"], reply.get("error"))

    def tearDown(self):
        labview_api.lv_close()
        if os.path.exists(self._cfg_path):
            os.remove(self._cfg_path)

    def test_every_public_function_returns_json_or_floats(self):
        self.assertEqual(labview_api.lv_is_connected(), 1)
        for name in ("lv_status", "lv_brake_states", "lv_clear_errors"):
            with self.subTest(fn=name):
                parsed = json.loads(getattr(labview_api, name)())
                self.assertIn("ok", parsed)

    def test_orientation_array_is_five_doubles(self):
        values = labview_api.lv_orientation_array()
        self.assertEqual(len(values), 5)
        for v in values:
            self.assertIsInstance(v, float)

    def test_actuator_positions_are_three_doubles(self):
        values = labview_api.lv_actuator_positions_mm()
        self.assertEqual(len(values), 3)

    def test_move_and_verify(self):
        reply = json.loads(labview_api.lv_move(30.0, 0.1, 0.0, 1))
        self.assertTrue(reply["ok"], reply.get("error"))
        focus, tip, tilt, total, azimuth = labview_api.lv_orientation_array()
        self.assertAlmostEqual(focus, 30.0, places=2)
        self.assertAlmostEqual(tip, 0.1, places=3)

    def test_relative_and_polar_moves(self):
        self.assertTrue(json.loads(labview_api.lv_move(25.0, 0.0, 0.0, 1))["ok"])
        self.assertTrue(json.loads(labview_api.lv_move_relative(1.0, 0.0, 0.0, 1))["ok"])
        self.assertAlmostEqual(labview_api.lv_orientation_array()[0], 26.0, places=2)
        self.assertTrue(json.loads(labview_api.lv_move_polar(25.0, 0.1, 45.0, 1))["ok"])
        self.assertAlmostEqual(labview_api.lv_orientation_array()[4], 45.0, places=1)

    def test_refused_move_returns_error_not_exception(self):
        reply = json.loads(labview_api.lv_move(999.0, 0.0, 0.0, 1))
        self.assertFalse(reply["ok"])
        self.assertIn("Move refused", reply["error"])
        self.assertIn("Move refused", labview_api.lv_last_error())

    def test_calls_without_a_session_fail_cleanly(self):
        labview_api.lv_close()
        self.assertEqual(labview_api.lv_is_connected(), 0)
        reply = json.loads(labview_api.lv_status())
        self.assertFalse(reply["ok"])
        self.assertIn("lv_open", reply["error"])
        # And the array variants degrade to NaN rather than raising.
        values = labview_api.lv_orientation_array()
        self.assertEqual(len(values), 5)
        self.assertNotEqual(values[0], values[0])   # NaN

    def test_bad_brake_action_is_reported(self):
        reply = json.loads(labview_api.lv_brake("wiggle"))
        self.assertFalse(reply["ok"])
        self.assertIn("engage", reply["error"])

    def test_stop_and_passivate(self):
        self.assertTrue(json.loads(labview_api.lv_stop())["ok"])
        self.assertTrue(json.loads(labview_api.lv_passivate())["ok"])

    def test_preview_reports_limits_without_moving(self):
        before = labview_api.lv_orientation_array()[0]
        reply = json.loads(labview_api.lv_preview(999.0, 0.0, 0.0))
        self.assertTrue(reply["ok"])
        self.assertFalse(reply["result"]["within_limits"])
        self.assertAlmostEqual(labview_api.lv_orientation_array()[0], before, places=4)

    def test_log_is_available(self):
        labview_api.lv_move(26.0, 0.0, 0.0, 1)
        self.assertIn("Move to", labview_api.lv_get_log())


class TestCli(unittest.TestCase):
    def test_help_builds(self):
        from psct_motors.cli import build_parser
        parser = build_parser()
        self.assertIn("commissioning order", parser.format_help())

    def test_simulated_status_runs(self):
        from psct_motors.cli import main
        self.assertEqual(main(["--simulate", "status", "--json"]), 0)

    def test_simulated_move_runs(self):
        from psct_motors.cli import main
        self.assertEqual(main(["--simulate", "-y", "move", "--focus", "26"]), 0)

    def test_refused_move_exits_nonzero(self):
        from psct_motors.cli import main
        self.assertEqual(main(["--simulate", "-y", "move", "--focus", "900"]), 1)

    def test_preview_needs_no_hardware(self):
        from psct_motors.cli import main
        self.assertEqual(main(["preview", "--focus", "25", "--tip", "0.1"]), 0)

    def test_register_table_prints_offline(self):
        from psct_motors.cli import main
        self.assertEqual(main(["verify-registers", "--offline"]), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
