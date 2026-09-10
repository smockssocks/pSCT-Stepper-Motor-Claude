"""Tests for the single-motor bench GUI.

Driven headlessly; skipped without a display. To run where the default
interpreter has no tkinter::

    xvfb-run -a python3.12 -m unittest tests.test_single_gui -v

The properties worth testing are the ones a screenshot cannot show: that the
error panel reflects the motor, that the log records the causes of a stall,
and that STOP works while something else is running.
"""

import gc
import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import tkinter as tk
    HAVE_TK = True
except ImportError:
    HAVE_TK = False

from psct_motors.eventlog import ERROR, WARNING, read_log  # noqa: E402
from psct_motors.faults import Fault  # noqa: E402
from psct_motors.registers import MotorMode  # noqa: E402


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


@unittest.skipUnless(display_available(), "tkinter or a display is unavailable")
class TestSingleMotorGui(unittest.TestCase):
    def setUp(self):
        from psct_motors.single_gui import SingleMotorApp
        self.dir = tempfile.mkdtemp()
        self.log_path = os.path.join(self.dir, "events.jsonl")
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = SingleMotorApp(self.root, motor_name="Top", simulate=True,
                                  log_path=self.log_path)
        self.app.on_connect()
        self.pump(1.0)

    def tearDown(self):
        self.app.shutdown()
        self.pump(0.1)
        self.root.destroy()
        self.app = None
        self.root = None
        gc.collect()
        for name in os.listdir(self.dir):
            os.remove(os.path.join(self.dir, name))
        os.rmdir(self.dir)

    def pump(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.root.update_idletasks()
            self.root.update()
            time.sleep(0.01)

    def log_text(self) -> str:
        return self.app.log_text.get("1.0", "end")

    # ---- connection and readout -----------------------------------------

    def test_connects_and_starts_recording(self):
        self.assertTrue(self.app.motor.connected)
        self.assertTrue(self.app.watcher.running)
        self.assertIn("connected", self.app.conn_var.get())

    def test_readout_shows_counts_revolutions_and_degrees(self):
        self.app.motor.set_mode(MotorMode.POSITION)
        self.app.motor.command_position_counts(self.app.counts(0.5))
        self.pump(1.5)
        text = self.app.position_var.get()
        self.assertIn("ct", text)
        self.assertIn("rev", text)
        self.assertIn("°", text)

    def test_no_millimetres_anywhere_in_the_readout(self):
        """A bare shaft has no millimetres to report."""
        self.pump(0.5)
        for var in (self.app.position_var, self.app.target_var):
            self.assertNotIn(" mm", var.get())

    # ---- errors ----------------------------------------------------------

    def test_error_panel_is_green_when_healthy(self):
        from psct_motors.single_gui import COLOR_OK_BG
        self.pump(0.6)
        self.assertIn("none", self.app.error_var.get())
        self.assertEqual(self.app.error_frame.cget("bg"), COLOR_OK_BG)

    def test_injected_error_turns_the_panel_red_and_is_decoded(self):
        from psct_motors.single_gui import COLOR_ERROR_BG
        self.app.fault_var.set(Fault.ERROR_BITS.value)
        self.app.on_arm_fault()
        self.pump(1.2)
        self.assertEqual(self.app.error_frame.cget("bg"), COLOR_ERROR_BG)
        self.assertIn("0x00000002", self.app.error_var.get())
        self.assertIn("UNVERIFIED", self.app.error_var.get())
        self.assertIn("ARMED", self.app.fault_status_var.get())

    def test_clearing_the_injection_clears_the_panel(self):
        self.app.fault_var.set(Fault.ERROR_BITS.value)
        self.app.on_arm_fault()
        self.pump(1.0)
        self.app.on_clear_fault()
        self.pump(1.2)
        self.assertIn("none", self.app.error_var.get())

    def test_comms_loss_is_shown_in_the_error_panel(self):
        self.app.fault_var.set(Fault.COMMS_DROP.value)
        self.app.on_arm_fault()
        self.pump(1.2)
        self.assertIn("NO COMMUNICATION", self.app.error_var.get())
        self.app.on_clear_fault()

    # ---- the log ---------------------------------------------------------

    def test_log_records_a_drive_going_passive_on_its_own(self):
        """The reported symptom: it stops taking position commands."""
        self.app.motor.set_mode(MotorMode.POSITION)
        self.pump(0.8)
        self.app.motor.write_register("MODE_REG", int(MotorMode.PASSIVE))
        self.pump(1.2)
        errors = [e.message for e in self.app.log.events(min_severity=ERROR)]
        self.assertTrue(any("went passive on its own" in m for m in errors), errors)

    def test_log_records_velocity_being_zeroed(self):
        self.pump(0.6)
        self.app.motor.write_register("V_SOLL", 0)
        self.pump(1.2)
        errors = [e.message for e in self.app.log.events(min_severity=ERROR)]
        self.assertTrue(any("never move" in m for m in errors), errors)

    def test_log_is_written_to_disk_as_it_happens(self):
        self.pump(0.6)
        replayed = read_log(self.log_path)
        self.assertTrue(replayed)
        self.assertTrue(any("Bench GUI opened" in e.message for e in replayed))

    def test_severity_filter_hides_lower_levels(self):
        self.app.motor.write_register("V_SOLL", 0)
        self.pump(1.0)
        self.app.severity_var.set(ERROR)
        self.pump(0.6)
        body = self.log_text()
        self.assertIn("never move", body)
        self.assertNotIn("Bench GUI opened", body)

    def test_export_writes_a_readable_file(self):
        self.pump(0.5)
        out = os.path.join(self.dir, "export.txt")
        self.app.log.export_text(out)
        with open(out, encoding="utf-8") as fh:
            self.assertIn("Bench GUI opened", fh.read())

    # ---- diagnosis -------------------------------------------------------

    def test_diagnose_names_the_blocker_in_the_log(self):
        self.app.motor.write_register("MODE_REG", int(MotorMode.PASSIVE))
        self.pump(0.6)
        self.app.on_diagnose()
        self.pump(1.5)
        messages = [e.message for e in self.app.log.events(min_severity=ERROR)]
        self.assertTrue(any("Drive is passive" in m for m in messages), messages)

    # ---- stop ------------------------------------------------------------

    def test_stop_works_while_the_ui_is_busy(self):
        self.app._set_busy(True)
        self.app.on_stop()
        self.pump(1.0)
        warnings = [e.message for e in self.app.log.events(min_severity=WARNING)]
        self.assertTrue(any("STOP pressed" in m for m in warnings))

    def test_stop_interrupts_a_move(self):
        self.app.motor.cfg.velocity_raw = 20
        self.app.motor.set_mode(MotorMode.POSITION)
        self.app.motor.set_velocity(20)
        self.app.motor.command_position_counts(self.app.counts(3.0))
        self.pump(0.8)
        self.app.on_stop()
        self.pump(1.2)
        target = self.app.motor.get_target_counts()
        self.assertLess(abs(target - self.app.motor.get_position_counts()), 2000)
        self.assertLess(target, self.app.counts(3.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
