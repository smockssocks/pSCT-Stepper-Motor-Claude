"""GUI tests, driven headlessly.

Skipped automatically when tkinter or a display is unavailable. To run them
where the default interpreter has no tkinter::

    xvfb-run -a python3.12 -m unittest tests.test_gui -v

The point of these is not to check pixels. It is to check the two properties
that a person cannot verify by looking at a screenshot:

  * the STOP button interrupts a move that is already in flight, rather than
    waiting for the worker thread to become free
  * the live readout reflects what the platform actually reports, including
    when a motor stops answering
"""

import gc
import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import tkinter as tk
    HAVE_TK = True
except ImportError:
    HAVE_TK = False

from psct_motors.config import BrakeConfig, PlatformLimits, default_config  # noqa: E402
from psct_motors.kinematics import Orientation  # noqa: E402


def display_available() -> bool:
    if not HAVE_TK:
        return False
    if os.name != "nt" and not os.environ.get("DISPLAY"):
        return False
    try:
        root = tk.Tk()
    except Exception:
        return False
    root.destroy()
    return True


def gui_config(velocity_raw=8000):
    cfg = default_config()
    for a in cfg.actuators:
        a.counts_per_mm = 1000.0
        a.velocity_raw = velocity_raw
        a.min_travel_mm = 0.0
        a.max_travel_mm = 50.0
        a.in_position_tol_mm = 0.01
        a.move_timeout_s = 30.0
        a.brake = BrakeConfig(mode="output", settle_s=0.0)
    cfg.limits = PlatformLimits(min_focus_mm=0.5, max_focus_mm=49.5,
                                max_tilt_deg=1.0, max_step_mm=25.0,
                                max_tilt_step_deg=1.0)
    cfg.poll_interval_s = 0.1
    cfg.validate()
    return cfg


@unittest.skipUnless(display_available(), "tkinter or a display is unavailable")
class TestGui(unittest.TestCase):
    def setUp(self):
        from psct_motors.gui import MotorApp
        from psct_motors.platform import FocalPlanePlatform

        self.root = tk.Tk()
        self.root.withdraw()
        self.app = MotorApp(self.root, simulate=True)
        # Swap in a bench platform with predictable scaling.
        self.app.cfg = gui_config()
        self.app.platform = FocalPlanePlatform(
            cfg=self.app.cfg, simulate=True, logger=self.app.log_threadsafe
        )
        self.app.platform.connect()

    def tearDown(self):
        self.app.shutdown()
        self.pump(0.1)              # let any in-flight worker finish unwinding
        self.root.destroy()
        # Drop every reference to a widget on THIS thread. If a Tk variable is
        # collected on a worker thread after the interpreter is gone, CPython
        # aborts with "Tcl_AsyncDelete: async handler deleted by the wrong
        # thread" and takes the whole test run with it.
        self.app = None
        self.root = None
        gc.collect()

    def pump(self, seconds: float) -> None:
        """Run the tk event loop for a while without blocking on mainloop."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.root.update_idletasks()
            self.root.update()
            time.sleep(0.01)

    # ---- readout ---------------------------------------------------------

    def test_readout_tracks_the_platform(self):
        self.app._start_polling()
        self.app.platform.move_to_orientation(Orientation(28.0, 0.1, -0.05))
        self.pump(0.6)
        text = self.app.orientation_var.get()
        self.assertIn("28.0", text)
        self.assertIn("0.10000", text)
        detail = self.app.orientation_detail_var.get()
        self.assertIn("arcmin", detail)

    def test_rows_show_each_actuator(self):
        self.app._start_polling()
        self.pump(0.5)
        self.assertEqual(set(self.app.rows), {"A", "B", "C"})
        for name, row in self.app.rows.items():
            self.assertIn("mm", row.position_var.get())
            self.assertIn("ct", row.counts_var.get())

    def test_lost_motor_shows_no_comms_and_hides_the_orientation(self):
        self.app._start_polling()
        self.pump(0.4)
        self.app.platform.motors[0]._transport.set_offline(True)
        self.pump(0.6)
        self.assertIn("no comms", self.app.rows["A"].mode_var.get())
        self.assertIn("unavailable", self.app.orientation_var.get())

    def test_brake_lamp_follows_the_brake(self):
        self.app._start_polling()
        self.app.platform.set_all_brakes(engaged=True)
        self.pump(0.5)
        self.assertIn("engaged", self.app.rows["A"].brake_var.get())
        self.app.platform.move_to_orientation(Orientation(26.0, 0.0, 0.0))
        self.pump(0.5)
        self.assertIn("released", self.app.rows["A"].brake_var.get())

    # ---- the property that matters ---------------------------------------

    def test_stop_button_interrupts_a_move_in_flight(self):
        for actuator in self.app.cfg.actuators:
            actuator.velocity_raw = 20              # ~2 mm/s in the simulator
        self.app.platform.move_to_orientation(Orientation(5.0, 0.0, 0.0))
        self.app._start_polling()

        done = threading.Event()

        def mover():
            try:
                self.app.platform.move_to_orientation(Orientation(20.0, 0.0, 0.0))
            except Exception:
                pass
            finally:
                done.set()

        threading.Thread(target=mover, daemon=True).start()
        self.pump(1.0)
        self.assertLess(self.app.platform.read_orientation().focus_mm, 19.0,
                        "the move finished before it could be stopped")

        started = time.monotonic()
        self.app.on_stop()                           # what the button calls
        self.pump(1.5)
        self.assertLess(time.monotonic() - started, 5.0)

        self.assertTrue(done.wait(timeout=20), "the move thread never unwound")
        self.assertLess(self.app.platform.read_orientation().focus_mm, 19.0,
                        "the move should have been cut short")
        self.assertIn("STOP", self.app.log_text.get("1.0", "end"))

    def test_stop_works_even_while_the_ui_thinks_it_is_busy(self):
        """A queued command must never be able to block the stop button."""
        self.app._set_busy(True)
        self.app.on_stop()
        self.pump(0.8)
        self.assertIn("STOP pressed", self.app.log_text.get("1.0", "end"))

    # ---- input handling --------------------------------------------------

    def test_bad_numbers_are_rejected_without_moving(self):
        self.app.focus_var.set("not a number")
        before = self.app.platform.read_actuator_positions_mm()
        # messagebox would block, so check the parse path directly.
        self.assertIsNone(self._silent(self.app._read_orientation_fields))
        after = self.app.platform.read_actuator_positions_mm()
        for b, a in zip(before, after):
            self.assertAlmostEqual(b, a, places=4)

    def _silent(self, fn):
        """Call `fn` with message boxes stubbed out."""
        from psct_motors import gui
        original = gui.messagebox
        class _Stub:
            @staticmethod
            def showerror(*a, **k): return None
            @staticmethod
            def showwarning(*a, **k): return None
            @staticmethod
            def showinfo(*a, **k): return None
            @staticmethod
            def askyesno(*a, **k): return True
        gui.messagebox = _Stub()
        try:
            return fn()
        finally:
            gui.messagebox = original


if __name__ == "__main__":
    unittest.main(verbosity=2)
