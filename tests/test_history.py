"""The position history: the only record of previous positions there is.

The motors keep none, so if this is wrong there is nothing to fall back on.
These check that every kind of move is written down with where the plane
really ended up, that a halted move is recorded as halted, that a refused
move is not recorded at all (nothing moved), that the record survives a
restart, and that "go back" is an ordinary checked move.
"""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from psct_motors import safety  # noqa: E402
from psct_motors.history import MoveRecord, PositionHistory, default_history_path  # noqa: E402
from psct_motors.kinematics import Orientation  # noqa: E402
from psct_motors.platform import FocalPlanePlatform, PlatformError  # noqa: E402


class TestPositionHistory(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.path = os.path.join(self.dir, "positions.jsonl")

    def tearDown(self):
        for name in os.listdir(self.dir):
            os.remove(os.path.join(self.dir, name))
        os.rmdir(self.dir)

    def test_records_are_kept_newest_first_and_written_to_disk(self):
        history = PositionHistory(path=self.path)
        history.record("move", Orientation(0.0), Orientation(1.0), Orientation(1.0))
        history.record("nudge", Orientation(1.0), Orientation(1.5), Orientation(1.5))
        self.assertEqual([r.kind for r in history.records()], ["nudge", "move"])
        self.assertEqual([r.kind for r in history.records(newest_first=False)],
                         ["move", "nudge"])
        with open(self.path, encoding="utf-8") as fh:
            lines = [json.loads(line) for line in fh if line.strip()]
        self.assertEqual(len(lines), 2)
        self.assertEqual(lines[1]["after"]["focus_mm"], 1.5)

    def test_the_record_survives_a_restart(self):
        PositionHistory(path=self.path).record(
            "move", Orientation(0.0, 0.1, 0.0), Orientation(2.0), Orientation(2.0),
            actuators_before={"Top": 0.0}, outcome="done", note="first")
        reloaded = PositionHistory(path=self.path)
        self.assertEqual(len(reloaded), 1)
        record = reloaded.last()
        self.assertEqual(record.note, "first")
        self.assertAlmostEqual(record.before.tip_deg, 0.1)
        self.assertEqual(record.actuators_before, {"Top": 0.0})

    def test_a_bad_line_on_disk_is_skipped_not_fatal(self):
        with open(self.path, "w", encoding="utf-8") as fh:
            fh.write("this is not json\n")
            fh.write(json.dumps(MoveRecord(
                1.0, "move", Orientation(0.0), Orientation(1.0),
                Orientation(1.0)).as_dict()) + "\n")
        messages = []
        history = PositionHistory(path=self.path, logger=messages.append)
        self.assertEqual(len(history), 1)
        self.assertTrue(any("unreadable" in m for m in messages))

    def test_go_back_wants_the_latest_record_with_a_before(self):
        history = PositionHistory()
        history.record("move", None, Orientation(1.0), Orientation(1.0))
        self.assertIsNone(history.last_with_before())
        history.record("move", Orientation(1.0), Orientation(2.0), Orientation(2.0))
        history.record("find-stop", None, None, Orientation(20.0))
        self.assertEqual(history.last_with_before().before.focus_mm, 1.0)

    def test_the_listener_hears_every_addition(self):
        heard = []
        history = PositionHistory(on_change=lambda: heard.append(1))
        history.record("move", Orientation(0.0), Orientation(1.0), Orientation(1.0))
        history.record("move", Orientation(1.0), Orientation(2.0), Orientation(2.0))
        self.assertEqual(len(heard), 2)

    def test_the_default_path_sits_beside_the_configuration(self):
        path = default_history_path("/some/where/psct_motors.json")
        self.assertEqual(os.path.dirname(path), "/some/where")
        self.assertTrue(path.endswith("positions.jsonl"))


class TestPlatformRecordsMoves(unittest.TestCase):
    def _platform(self, **kw):
        platform = FocalPlanePlatform(cfg=safety.bench_config(), simulate=True, **kw)
        platform.connect()
        self.addCleanup(platform.disconnect)
        return platform

    def test_a_move_records_before_commanded_and_after(self):
        platform = self._platform()
        start = platform.read_orientation().focus_mm
        platform.move_to_orientation(Orientation(start + 1.0, 0.0, 0.0))
        record = platform.history.last()
        self.assertEqual(record.kind, "move")
        self.assertAlmostEqual(record.before.focus_mm, start, places=3)
        self.assertAlmostEqual(record.commanded.focus_mm, start + 1.0, places=6)
        # "after" is read back from the motors, not assumed.
        self.assertAlmostEqual(record.after.focus_mm, start + 1.0, places=2)
        self.assertTrue(record.completed)
        self.assertEqual(set(record.actuators_after), {"Top", "East", "West"})

    def test_a_nudge_and_a_jog_are_recorded_with_their_kind(self):
        platform = self._platform()
        platform.move_relative(d_focus_mm=0.5)
        self.assertEqual(platform.history.last().kind, "nudge")
        self.assertIn("focus +0.5 mm", platform.history.last().note)
        platform.move_actuator_mm("Top", 0.2, relative=True)
        record = platform.history.last()
        self.assertEqual(record.kind, "jog")
        self.assertIn("Top", record.note)
        # A jog tilts the plane, and the record says so through the tilt.
        self.assertNotAlmostEqual(record.after.tip_deg, record.before.tip_deg, places=6)

    def test_a_refused_move_is_not_a_position(self):
        platform = self._platform()
        with self.assertRaises(PlatformError):
            platform.move_to_orientation(Orientation(900.0, 0.0, 0.0))
        self.assertEqual(len(platform.history), 0)

    def test_a_halted_move_is_recorded_as_halted_where_it_stopped(self):
        import threading
        import time
        # Slow enough that STOP lands mid-move: 8 mm at 2 mm/s is four
        # seconds, and the move is under the 10 mm step limit.
        cfg = safety.bench_config()
        cfg.simulated_speed_mm_per_s = 2.0
        platform = FocalPlanePlatform(cfg=cfg, simulate=True)
        platform.connect()
        self.addCleanup(platform.disconnect)
        start = platform.read_orientation().focus_mm
        errors = []

        def mover():
            try:
                platform.move_to_orientation(Orientation(start + 8.0, 0.0, 0.0))
            except PlatformError as exc:
                errors.append(exc)

        thread = threading.Thread(target=mover)
        thread.start()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if platform.read_orientation().focus_mm > start + 0.5:
                break
            time.sleep(0.05)
        platform.stop()
        thread.join(timeout=10)
        self.assertTrue(errors, "the halted move should have raised")
        record = platform.history.last()
        self.assertIsNotNone(record, "a halted move must still be recorded")
        self.assertFalse(record.completed)
        self.assertIn("failed", record.outcome)
        # Where it really stopped: somewhere between the two, not the target.
        self.assertGreater(record.after.focus_mm, start)
        self.assertLess(record.after.focus_mm, start + 8.0)

    def test_go_back_returns_to_where_it_was_and_is_itself_recorded(self):
        platform = self._platform()
        start = platform.read_orientation().focus_mm
        platform.move_to_orientation(Orientation(start + 2.0, 0.0, 0.0))
        platform.go_back()
        self.assertAlmostEqual(platform.read_orientation().focus_mm, start, places=2)
        record = platform.history.last()
        self.assertEqual(record.kind, "go-back")
        self.assertIn("before the move", record.note)

    def test_go_back_with_nothing_on_record_is_refused(self):
        platform = self._platform()
        with self.assertRaises(PlatformError) as ctx:
            platform.go_back()
        self.assertIn("no previous position", str(ctx.exception))

    def test_go_back_is_checked_like_any_other_move(self):
        """Going back is not an unchecked path: if the previous position is
        now outside the limits, it is refused with the usual message."""
        platform = self._platform()
        start = platform.read_orientation().focus_mm
        platform.move_to_orientation(Orientation(start + 2.0, 0.0, 0.0))
        platform.cfg.limits.min_focus_mm = start + 1.0
        with self.assertRaises(PlatformError) as ctx:
            platform.go_back()
        self.assertIn("outside the allowed", str(ctx.exception))

    def test_the_record_is_written_beside_the_config_when_asked(self):
        directory = tempfile.mkdtemp()
        path = os.path.join(directory, "positions.jsonl")
        try:
            platform = self._platform(history_path=path)
            start = platform.read_orientation().focus_mm
            platform.move_to_orientation(Orientation(start + 1.0, 0.0, 0.0))
            self.assertTrue(os.path.exists(path))
            with open(path, encoding="utf-8") as fh:
                self.assertEqual(len([l for l in fh if l.strip()]), 1)
        finally:
            for name in os.listdir(directory):
                os.remove(os.path.join(directory, name))
            os.rmdir(directory)

    def test_a_platform_built_without_a_path_writes_nothing(self):
        """Tests and drills build platforms by the dozen; none of them may
        leave a line in the real machine's record."""
        platform = self._platform()
        self.assertIsNone(platform.history.path)


if __name__ == "__main__":
    unittest.main(verbosity=2)
