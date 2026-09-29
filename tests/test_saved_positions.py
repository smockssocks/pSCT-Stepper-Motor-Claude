"""Named positions: saved, listed, gone back to, and still right after a re-zero.

A saved position is somewhere the plane has actually been. These check that
it is stored as encoder counts and so still means the same physical place
after somebody sets a new zero, that names do not collide by case, that the
file survives a restart, and that going to one is an ordinary checked move.
"""

import contextlib
import io
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from psct_motors import safety  # noqa: E402
from psct_motors.kinematics import Orientation  # noqa: E402
from psct_motors.platform import FocalPlanePlatform, PlatformError  # noqa: E402
from psct_motors.saved_positions import (  # noqa: E402
    SavedPositions, default_saved_positions_path, make_saved_position)


def _scratch_dir(test):
    directory = tempfile.mkdtemp()

    def cleanup():
        for name in os.listdir(directory):
            os.remove(os.path.join(directory, name))
        os.rmdir(directory)
    test.addCleanup(cleanup)
    return directory


class TestSavedPositionsStore(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(_scratch_dir(self), "saved_positions.json")

    def _position(self, name, focus=0.0):
        return make_saved_position(name, Orientation(focus), {"Top": 1, "East": 2,
                                                              "West": 3},
                                   {"Top": 0.0, "East": 0.0, "West": 0.0})

    def test_positions_survive_a_restart(self):
        store = SavedPositions(path=self.path)
        store.put(self._position("Default", 1.0))
        store.put(self._position("Window open", 2.5))
        again = SavedPositions(path=self.path)
        self.assertEqual(again.names(), ["Default", "Window open"])
        self.assertEqual(again.get("window open").orientation.focus_mm, 2.5)
        self.assertEqual(again.get("Default").actuator_counts["West"], 3)
        with open(self.path, encoding="utf-8") as fh:
            self.assertEqual(json.load(fh)["version"], 1)

    def test_the_same_name_in_another_case_replaces_rather_than_duplicates(self):
        store = SavedPositions(path=self.path)
        self.assertFalse(store.put(self._position("Default", 1.0)))
        self.assertTrue(store.put(self._position(" default ", 2.0)))
        self.assertEqual(len(store), 1)
        self.assertEqual(store.get("DEFAULT").orientation.focus_mm, 2.0)

    def test_rename_refuses_a_name_that_is_taken(self):
        store = SavedPositions(path=self.path)
        store.put(self._position("Default"))
        store.put(self._position("Window open"))
        with self.assertRaises(ValueError):
            store.rename("Default", "window OPEN")
        store.rename("Default", "Survey")
        self.assertEqual(SavedPositions(path=self.path).names(),
                         ["Survey", "Window open"])

    def test_delete_and_empty_names(self):
        store = SavedPositions(path=self.path)
        store.put(self._position("Default"))
        self.assertTrue(store.delete("default"))
        self.assertFalse(store.delete("default"))
        with self.assertRaises(ValueError):
            store.put(self._position("   "))

    def test_a_damaged_file_is_reported_not_fatal(self):
        with open(self.path, "w", encoding="utf-8") as fh:
            fh.write("{not json")
        said = []
        store = SavedPositions(path=self.path, logger=said.append)
        self.assertEqual(store.names(), [])
        self.assertTrue(any("could not be read" in s for s in said))

    def test_it_lives_beside_the_configuration(self):
        path = default_saved_positions_path("/some/where/psct_motors.json")
        self.assertEqual(path, os.path.abspath("/some/where/saved_positions.json"))


class TestSavedPositionsOnThePlatform(unittest.TestCase):
    def _platform(self):
        platform = FocalPlanePlatform(cfg=safety.bench_config(), simulate=True)
        platform.connect()
        self.addCleanup(platform.disconnect)
        return platform

    def test_save_move_away_and_go_back(self):
        platform = self._platform()
        platform.move_to_orientation(Orientation(1.0, 0.02, -0.01))
        saved, replaced = platform.save_position("Window open", note="new window")
        self.assertFalse(replaced)
        self.assertAlmostEqual(saved.orientation.tip_deg, 0.02, places=3)
        platform.move_to_orientation(Orientation(-1.0, 0.0, 0.0))
        platform.go_to_saved_position("window open")
        now = platform.read_orientation()
        self.assertAlmostEqual(now.focus_mm, 1.0, places=2)
        self.assertAlmostEqual(now.tip_deg, 0.02, places=3)
        record = platform.history.last()
        self.assertEqual((record.kind, record.note), ("saved position", "Window open"))

    def test_a_new_zero_does_not_move_a_saved_position(self):
        """Saved as counts, so it is the same physical place afterwards --
        just described by different numbers."""
        platform = self._platform()
        platform.move_to_orientation(Orientation(2.0, 0.0, 0.0))
        saved, _ = platform.save_position("Default")
        platform.move_to_orientation(Orientation(0.5, 0.0, 0.0))
        platform.set_zero_here(persist=False)
        target = platform.saved_position_target(saved)
        self.assertAlmostEqual(target.focus_mm, 1.5, places=2)
        platform.go_to_saved_position("Default")
        counts = {m.name: m.get_position_counts() for m in platform.motors}
        for name, value in saved.actuator_counts.items():
            self.assertLess(abs(counts[name] - value),
                            platform.motor(name).cfg.resolved_counts_per_mm * 0.01)

    def test_going_to_a_saved_position_is_checked_like_any_move(self):
        platform = self._platform()
        platform.move_to_orientation(Orientation(2.0, 0.0, 0.0))
        platform.save_position("Far")
        platform.move_to_orientation(Orientation(0.0, 0.0, 0.0))
        platform.cfg.limits.max_focus_mm = 1.0
        before = platform.read_orientation().focus_mm
        with self.assertRaises(PlatformError):
            platform.go_to_saved_position("Far")
        self.assertAlmostEqual(platform.read_orientation().focus_mm, before, places=4)

    def test_an_unknown_name_is_refused(self):
        with self.assertRaises(PlatformError):
            self._platform().go_to_saved_position("Nowhere")


class TestSavedPositionsCli(unittest.TestCase):
    def setUp(self):
        self._previous = os.environ.get("PSCT_MOTORS_CONFIG")
        os.environ["PSCT_MOTORS_CONFIG"] = os.path.join(_scratch_dir(self),
                                                        "psct_motors.json")

    def tearDown(self):
        if self._previous is None:
            os.environ.pop("PSCT_MOTORS_CONFIG", None)
        else:
            os.environ["PSCT_MOTORS_CONFIG"] = self._previous

    def _run(self, *argv):
        from psct_motors.cli import main
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            code = main(list(argv))
        return code, buffer.getvalue()

    def test_save_list_go_delete(self):
        code, _ = self._run("--simulate", "-y", "positions", "save", "Default",
                            "--note", "bench")
        self.assertEqual(code, 0)
        code, text = self._run("positions")
        self.assertEqual(code, 0)
        self.assertIn("Default", text)
        self.assertIn("bench", text)
        self.assertEqual(self._run("--simulate", "-y", "positions", "go",
                                   "default")[0], 0)
        self.assertEqual(self._run("--simulate", "-y", "positions", "go",
                                   "Nowhere")[0], 1)
        self.assertEqual(self._run("--simulate", "positions", "delete",
                                   "Default")[0], 0)
        self.assertNotIn("Default", self._run("positions")[1])


if __name__ == "__main__":
    unittest.main()
