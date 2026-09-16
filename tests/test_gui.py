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
    from tkinter import ttk
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
        # The gauge was built from whatever configuration was on disk. Point it
        # at the bench config these tests use, or one test's saved hard stop
        # leaks into the next one's expectations.
        self.app._refresh_gauge_limits()

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
        self.app.platform.move_to_orientation(Orientation(2.0, 0.1, -0.05))
        self.pump(0.6)
        # Focus has its own readout now; the window is built around it.
        focus = self.app.focus_readout_var.get()
        self.assertIn("2.0", focus)
        self.assertIn("um", focus)
        self.assertIn("towards M1", focus)
        # The tilts are still shown, just smaller and secondary.
        self.assertIn("0.10000", self.app.orientation_var.get())
        self.assertIn("arcmin", self.app.orientation_detail_var.get())

    def test_gauge_follows_the_focus(self):
        self.app._start_polling()
        self.app.platform.move_to_orientation(Orientation(3.0, 0.0, 0.0))
        self.pump(0.6)
        self.assertAlmostEqual(self.app.gauge._position_mm, 3.0, places=2)
        self.assertTrue(self.app.gauge._valid)

    def test_gauge_blanks_when_a_motor_is_lost(self):
        self.app._start_polling()
        self.pump(0.4)
        self.app.platform.motors[0]._transport.set_offline(True)
        self.pump(0.6)
        self.assertFalse(self.app.gauge._valid)

    def test_tip_and_tilt_are_behind_a_menu(self):
        """The main window is about focus; the tilts are one menu away."""
        labels = []
        for index in range(self.app.menubar.index("end") + 1):
            try:
                labels.append(self.app.menubar.entrycget(index, "label"))
            except Exception:
                pass
        self.assertIn("Motion", labels)
        self.assertIn("View", labels)
        self.assertIn("Tools", labels)
        # The variables exist whether or not the dialog is open, so a move
        # command can always read them.
        self.assertEqual(self.app.tip_var.get(), "0.0")
        self.assertIsNone(self.app._tilt_window)
        self.app.on_open_tilt()
        self.pump(0.3)
        self.assertTrue(self.app._tilt_window.winfo_exists())
        self.app._tilt_window.destroy()

    def test_focal_plane_picture_shows_each_actuator(self):
        self.app._start_polling()
        self.app.on_open_plane_view()
        self.pump(0.3)
        self.app.platform.move_to_orientation(Orientation(1.0, 0.1, -0.05))
        self.pump(0.6)
        self.assertIsNotNone(self.app.plane_view)
        self.assertEqual(len(self.app.plane_view._z), 3)
        # A tip means the actuators are not all at the same height.
        self.assertGreater(max(self.app.plane_view._z) - min(self.app.plane_view._z),
                           0.1)
        self.app._plane_window.destroy()
        self.pump(0.2)
        self.assertIsNone(self.app.plane_view)

    def test_picture_refuses_to_draw_a_stale_plane(self):
        self.app._start_polling()
        self.app.on_open_plane_view()
        self.pump(0.4)
        self.app.platform.motors[1]._transport.set_offline(True)
        self.pump(0.6)
        self.assertIsNone(self.app.plane_view._z)
        self.assertIn("East", self.app.plane_view._message)
        self.app._plane_window.destroy()

    def test_connection_settings_apply_and_rebuild(self):
        """Editing an address has to rebuild the motors, or only the label
        would change."""
        self.app.cfg.actuators[0].ip = "10.1.2.3"
        self.app._refresh_addresses()
        self.assertIn("10.1.2.3", self.app.addresses_var.get())
        self.app._rebuild_platform()
        self.assertEqual(self.app.platform.motors[0].cfg.ip, "10.1.2.3")
        self.assertFalse(self.app.platform.connected)

    def test_each_motor_shows_how_hard_it_is_working(self):
        """There is no amps register on these motors. Load is Actual Torque
        over the drive's current limit, and it has to be labelled as that."""
        self.app._start_polling()
        self.pump(0.5)
        for name, row in self.app.rows.items():
            self.assertIsNotNone(row.load_bar._percent,
                                 f"{name} reports no load")
            # The simulator idles at 337/2048 = 16.5%, as the real motor does.
            self.assertAlmostEqual(row.load_bar._percent, 100 * 337 / 2048,
                                   places=1)
            self.assertIn("%", row.load_var.get())

    def test_the_load_bar_marks_the_thresholds_it_is_judged_against(self):
        row = self.app.rows["Top"]
        actuator = self.app.cfg.actuator("Top")
        self.assertEqual(row.load_bar.stall_percent, actuator.stall_torque_percent)
        self.assertEqual(row.load_bar.warn_percent, actuator.torque_warn_percent)

    def test_the_load_bar_keeps_a_peak(self):
        """The peak of a move is what says whether the stall threshold has
        margin, and it is gone by the time you look at a settled axis."""
        row = self.app.rows["Top"]
        row.load_bar.set(12.0)
        row.load_bar.set(70.0)
        row.load_bar.set(12.0)
        self.assertAlmostEqual(row.load_bar._peak, 70.0)
        row.load_bar.reset_peak()
        self.assertEqual(row.load_bar._peak, 0.0)

    def test_load_shows_amps_only_when_the_rating_is_configured(self):
        """Inventing an amps figure from a torque fraction would be inventing
        a measurement."""
        self.app._start_polling()
        self.pump(0.4)
        self.assertIn("%", self.app.rows["Top"].load_var.get())
        for actuator in self.app.cfg.actuators:
            actuator.rated_current_a = 2.0
        self.pump(0.5)
        self.assertIn("A", self.app.rows["Top"].load_var.get())
        self.assertIn("~", self.app.rows["Top"].load_var.get())

    def test_a_lost_motor_shows_no_load_rather_than_a_stale_one(self):
        self.app._start_polling()
        self.pump(0.4)
        self.app.platform.motors[0]._transport.set_offline(True)
        self.pump(0.6)
        self.assertIsNone(self.app.rows["Top"].load_bar._percent)
        self.assertEqual(self.app.rows["Top"].load_var.get(), "")

    def test_the_gauge_marks_where_the_travel_ends(self):
        """A soft limit is a setting; an end stop is the machine. Seeing how
        much room is left between them is the point."""
        gauge = self.app.gauge
        self.assertIsNone(gauge.hard_stop_low_mm)
        self.assertIsNone(gauge.hard_stop_high_mm)

        self.app.cfg.limits.hard_stop_high_mm = 52.0
        self.app.cfg.limits.hard_stop_low_mm = -2.0
        self.app._refresh_gauge_limits()
        self.pump(0.2)

        self.assertEqual(gauge.hard_stop_high_mm, 52.0)
        # The drawn range has to widen to take them in, or a stop beyond the
        # soft limit lands on top of the limit it is meant to sit outside.
        low, high = gauge.drawn_range()
        self.assertLess(low, -2.0)
        self.assertGreater(high, 52.0)

    def test_the_gauge_range_is_the_soft_limits_until_a_stop_is_found(self):
        low, high = self.app.gauge.drawn_range()
        self.assertEqual(low, self.app.cfg.limits.min_focus_mm)
        self.assertEqual(high, self.app.cfg.limits.max_focus_mm)

    def test_a_found_hard_stop_is_recorded_and_drawn(self):
        from psct_motors.platform import HardStopResult
        result = HardStopResult(
            direction=+1, stopped_by=["Top"], reasons={},
            start_mm={}, positions_mm={"Top": 48.0, "East": 48.0, "West": 48.0},
            positions_counts={}, travelled_mm={}, spread_mm=0.0,
            worst_spread_mm=0.0, peak_torque_percent={},
            stop_mm={"Top": 49.0, "East": 49.0, "West": 49.0},
        )
        # Answer "no" to the save prompt: a test must not write the live
        # configuration file.
        self._answer(False, lambda: self.app._record_hard_stop(+1, result))
        self.pump(0.2)
        self.assertAlmostEqual(self.app.cfg.limits.hard_stop_high_mm, 49.0)
        self.assertAlmostEqual(self.app.gauge.hard_stop_high_mm, 49.0)
        self.assertIn("End of travel recorded",
                      self.app.log_text.get("1.0", "end"))

    def test_motion_limits_can_be_edited_from_the_menu(self):
        """The limits ship as a guess and the numbers that replace them come
        out of find-stop, run from this same window. Making that a text-file
        edit means somebody has to find the text file, in the dark."""
        self.assertIsNone(self.app._limits_window)
        self._answer(False, self.app.on_edit_limits)
        self.pump(0.3)
        self.assertTrue(self.app._limits_window.winfo_exists())
        self.app._limits_window.destroy()
        self.app._limits_window = None

    def test_soft_limits_outside_the_hard_stops_are_refused(self):
        """A soft limit beyond the end of travel is not a limit: every move it
        allows would end by driving into the stop."""
        from psct_motors.gui import MotorApp
        limits = self.app.cfg.limits
        limits.hard_stop_high_mm = 25.4
        limits.hard_stop_low_mm = -25.4

        limits.max_focus_mm = 24.0
        MotorApp._check_limits_against_stops(limits)      # inside: fine

        limits.max_focus_mm = 26.0
        with self.assertRaises(ValueError) as ctx:
            MotorApp._check_limits_against_stops(limits)
        self.assertIn("beyond the end of travel", str(ctx.exception))

        limits.max_focus_mm = 24.0
        limits.min_focus_mm = -30.0
        with self.assertRaises(ValueError):
            MotorApp._check_limits_against_stops(limits)

    def test_limits_are_unchecked_until_the_stops_are_known(self):
        from psct_motors.gui import MotorApp
        limits = self.app.cfg.limits
        self.assertIsNone(limits.hard_stop_high_mm)
        limits.max_focus_mm = 900.0
        MotorApp._check_limits_against_stops(limits)   # nothing to check against

    def test_the_load_window_shows_each_motor(self):
        """The bars in the table are small by necessity. This is the same
        numbers with room around them, for watching during a move."""
        self.app._start_polling()
        self.app.on_open_load_view()
        self.pump(0.6)
        self.assertIsNotNone(self.app._load_window)
        self.assertEqual(set(self.app._load_rows), {"Top", "East", "West"})
        for name, (now_var, bar, peak_var, temp_var, supply_var) in \
                self.app._load_rows.items():
            self.assertIn("%", now_var.get(), name)
            self.assertIsNotNone(bar._percent, name)
            self.assertIn("%", peak_var.get(), name)
            self.assertIn("C", temp_var.get(), name)
            self.assertNotEqual(supply_var.get(), "--", name)
        self.app._load_window.destroy()

    def test_the_load_window_blanks_a_motor_that_stops_answering(self):
        self.app._start_polling()
        self.app.on_open_load_view()
        self.pump(0.4)
        self.app.platform.motors[0]._transport.set_offline(True)
        self.pump(0.6)
        now_var, bar, _peak, temp_var, _supply = self.app._load_rows["Top"]
        self.assertEqual(now_var.get(), "--")
        self.assertIsNone(bar._percent)
        self.assertEqual(temp_var.get(), "--")
        self.app._load_window.destroy()

    def test_closing_the_load_window_stops_it_being_updated(self):
        self.app._start_polling()
        self.app.on_open_load_view()
        self.pump(0.3)
        self.app._load_window.protocol("WM_DELETE_WINDOW")  # exists
        self.app._load_window = None
        self.app._load_rows = {}
        self.pump(0.4)          # a poll with no window must not raise

    def test_resetting_peaks_clears_both_views(self):
        self.app._start_polling()
        self.app.on_open_load_view()
        self.pump(0.4)
        self.app.rows["Top"].load_bar.set(80.0)
        self.app._load_rows["Top"][1].set(80.0)
        self.app._reset_load_peaks()
        self.assertEqual(self.app.rows["Top"].load_bar._peak, 0.0)
        self.assertEqual(self.app._load_rows["Top"][1]._peak, 0.0)
        self.app._load_window.destroy()

    def test_the_in_row_reading_shows_the_percentage(self):
        self.app._start_polling()
        self.pump(0.5)
        self.assertIn("%", self.app.rows["Top"].load_var.get())

    def test_rows_show_each_actuator(self):
        self.app._start_polling()
        self.pump(0.5)
        self.assertEqual(set(self.app.rows), {"Top", "East", "West"})
        for name, row in self.app.rows.items():
            self.assertIn("mm", row.position_var.get())
            self.assertIn("ct", row.counts_var.get())

    def test_lost_motor_shows_no_comms_and_hides_the_orientation(self):
        self.app._start_polling()
        self.pump(0.4)
        self.app.platform.motors[0]._transport.set_offline(True)
        self.pump(0.6)
        self.assertIn("no comms", self.app.rows["Top"].mode_var.get())
        self.assertIn("unavailable", self.app.orientation_var.get())

    def test_brake_lamp_follows_the_brake(self):
        self.app._start_polling()
        self.app.platform.set_all_brakes(engaged=True)
        self.pump(0.5)
        self.assertIn("HOLDING", self.app.rows["Top"].brake_var.get())
        self.app.platform.move_to_orientation(Orientation(26.0, 0.0, 0.0))
        self.pump(0.5)
        self.assertIn("FREE", self.app.rows["Top"].brake_var.get())

    # ---- the property that matters ---------------------------------------

    def test_stop_button_interrupts_a_move_in_flight(self):
        from psct_motors.simulator import velocity_raw_for_mm_per_s
        for actuator in self.app.cfg.actuators:
            # Slow enough that the move is still running when STOP is pressed.
            actuator.velocity_raw = velocity_raw_for_mm_per_s(actuator, 2.0)
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

    def test_emergency_acts_without_asking_and_reports_afterwards(self):
        """A safety control that opens a modal is not a safety control. It has
        to act first and say what it did after."""
        from psct_motors import gui

        class NoDialogs:
            @staticmethod
            def askyesno(*a, **k):
                raise AssertionError("EMERGENCY must not ask for confirmation")
            @staticmethod
            def showerror(*a, **k): return None
            @staticmethod
            def showwarning(*a, **k): return None
            @staticmethod
            def showinfo(*a, **k): return None

        original = gui.messagebox
        gui.messagebox = NoDialogs()
        try:
            self.app._start_polling()
            self.app.on_passivate()
            self.pump(1.2)
        finally:
            gui.messagebox = original

        # It halted and is holding. The gui_config brakes are motor outputs,
        # so the brakes can be confirmed and the drives may go off.
        for motor in self.app.platform.motors:
            self.assertLess(abs(motor.get_target_mm() - motor.get_position_mm()), 0.5)
        # ...and then said so, both on the red bar and in the log.
        self.assertIn("EMERGENCY done", self.app.action_var.get())
        log = self.app.log_text.get("1.0", "end")
        self.assertIn("halted and holding", log)
        self.assertIn("brakes:", log)
        self.assertIn("drives:", log)
        # The report names every actuator and where it came to rest, read back
        # from the motors rather than assumed.
        for name in ("Top", "East", "West"):
            self.assertIn(f"{name:<5} stopped at", log)

    def test_emergency_leaves_the_drives_on_when_nothing_else_holds(self):
        """The reported failure: EMERGENCY cut drive power on a machine whose
        brakes this software cannot command, and the camera sank."""
        from psct_motors.external_brake import BrakeController, ExternalBrakeConfig
        from psct_motors.config import BrakeConfig

        for actuator in self.app.cfg.actuators:
            actuator.brake = BrakeConfig(mode="none")
        self.app.platform.external_brake = BrakeController(
            ExternalBrakeConfig(mode="none"))

        self.app._start_polling()
        self.app.on_passivate()
        self.pump(1.5)

        for motor in self.app.platform.motors:
            self.assertEqual(motor.get_mode(), 2,
                             f"{motor.name} was passivated with nothing holding it")
        self.assertIn("still on", self.app.action_var.get())
        self.assertIn("drives: ON", self.app.log_text.get("1.0", "end"))

    def test_emergency_report_says_which_motor_did_not_answer(self):
        self.app._start_polling()
        self.pump(0.3)
        self.app.platform.motors[1]._transport.set_offline(True)
        self.app.on_passivate()
        self.pump(1.5)
        log = self.app.log_text.get("1.0", "end")
        self.assertIn("East", log)
        self.assertIn("EMERGENCY incomplete", self.app.action_var.get())

    def test_stop_reports_where_it_stopped(self):
        self.app._start_polling()
        self.app.on_stop()
        self.pump(1.0)
        self.assertIn("STOP done", self.app.action_var.get())
        self.assertIn("drives still on", self.app.action_var.get())
        self.assertIn("Top   holding at", self.app.log_text.get("1.0", "end"))

    def test_safety_controls_still_act_when_one_motor_is_lost(self):
        """`connected` is all-three. If STOP keyed off that, losing one motor
        would disarm the button for the two that are still running."""
        self.app._start_polling()
        self.pump(0.3)
        self.app.platform.motors[1]._transport.set_offline(True)
        self.assertFalse(self.app.platform.connected)
        self.assertTrue(self.app.platform.any_connected)
        self.app.on_stop()
        self.pump(1.0)
        log = self.app.log_text.get("1.0", "end")
        self.assertNotIn("no motor is reachable", log)
        self.assertIn("could NOT stop East", log)
        self.assertIn("STOP incomplete", self.app.action_var.get())
        # The two that are still there were stopped and reported; the missing
        # one is named rather than passed over in silence.
        self.assertIn("Top   holding at", log)
        self.assertIn("West  holding at", log)
        self.assertIn("East  no comms", log)

    def test_actuator_names_are_not_clipped(self):
        """`width=4` fits "Top" and "East" but cuts "West" off at the T, because
        a label's width is counted in "0"-widths, not glyphs."""
        import tkinter.font as tkfont
        from psct_motors.gui import MotorRow

        font = tkfont.Font(font=MotorRow.NAME_FONT)
        self.pump(0.2)
        for name, row in self.app.rows.items():
            needed = font.measure(name)
            self.assertGreaterEqual(
                row.name_label.winfo_reqwidth(), needed,
                f"the {name} label is narrower than the word {name}")
        # The column reserves room for the longest name, so the table does not
        # shift about, and "West" -- the widest of the three -- fits.
        reserved = self.app.actuator_frame.grid_columnconfigure(0)["minsize"]
        self.assertGreaterEqual(int(reserved), font.measure("West"))

    def test_brake_column_fits_its_longest_label(self):
        """"released (inferred)" is the widest thing that column ever holds."""
        import tkinter.font as tkfont
        from psct_motors.gui import MotorRow

        font = tkfont.Font(font="TkDefaultFont")
        reserved = int(self.app.actuator_frame.grid_columnconfigure(6)["minsize"])
        for sample in MotorRow.BRAKE_SAMPLES:
            self.assertGreaterEqual(reserved, font.measure(sample), sample)

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

    def _answer(self, answer, fn):
        """Call `fn` with every dialog answered `answer`."""
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
            def askyesno(*a, **k): return answer

        gui.messagebox = _Stub()
        try:
            return fn()
        finally:
            gui.messagebox = original

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


@unittest.skipUnless(display_available(), "tkinter or a display is unavailable")
class TestEveryControlWorks(unittest.TestCase):
    """Click everything, and check nothing raises.

    This exists because of a real near-miss: an edit removed `on_safety_drills`
    while leaving the menu entry that calls it, and nothing failed until the
    window was built. A menu command that raises is invisible until somebody
    clicks it -- which, on the day, is in front of an audience.

    So every menu entry and every button is invoked here against simulated
    motors. It is a smoke test, not a behaviour test: the assertion is simply
    that the application is still standing afterwards, with no exception on the
    UI thread and no error dialog raised.
    """

    def setUp(self):
        from psct_motors.gui import MotorApp
        from psct_motors.platform import FocalPlanePlatform

        self.root = tk.Tk()
        self.root.withdraw()
        self.app = MotorApp(self.root, simulate=True)
        self.app.cfg = gui_config()
        self.app.platform = FocalPlanePlatform(
            cfg=self.app.cfg, simulate=True, logger=self.app.log_threadsafe)
        self.app.platform.connect()
        self.app._start_polling()

        # Anything that would block on a person is answered, and anything that
        # would report a failure is recorded so the test can fail on it.
        from psct_motors import gui
        self.errors = []
        test = self

        class Dialogs:
            @staticmethod
            def showerror(title, message, **k):
                test.errors.append(f"{title}: {message}")
            @staticmethod
            def showwarning(*a, **k): return None
            @staticmethod
            def showinfo(*a, **k): return None
            @staticmethod
            def askyesno(*a, **k): return False      # never save, never destroy

        self._real_messagebox = gui.messagebox
        gui.messagebox = Dialogs()

        # Tk prints a traceback to stderr and carries on when a callback
        # raises, so a broken button looks exactly like one that had nothing
        # to do. Without this hook every test below passes on a dead control.
        self.callback_errors = []

        def record(exc_type, exc_value, exc_tb):
            self.callback_errors.append(f"{exc_type.__name__}: {exc_value}")

        self.root.report_callback_exception = record

    def tearDown(self):
        from psct_motors import gui
        gui.messagebox = self._real_messagebox
        self.app.shutdown()
        self.pump(0.2)
        for window in list(self.root.winfo_children()):
            try:
                if isinstance(window, tk.Toplevel):
                    window.destroy()
            except Exception:
                pass
        self.root.destroy()
        self.app = None
        self.root = None
        gc.collect()

    def pump(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.root.update_idletasks()
            self.root.update()
            time.sleep(0.01)

    def _menu_commands(self):
        """(label, callable) for every leaf entry in the menu bar."""
        found = []
        bar = self.app.menubar
        for index in range(bar.index("end") + 1):
            try:
                submenu_name = bar.entrycget(index, "menu")
            except Exception:
                continue
            if not submenu_name:
                continue
            submenu = self.root.nametowidget(submenu_name)
            for entry in range(submenu.index("end") + 1):
                try:
                    if submenu.type(entry) != "command":
                        continue
                    label = submenu.entrycget(entry, "label")
                except Exception:
                    continue
                found.append((label, lambda s=submenu, e=entry: s.invoke(e)))
        return found

    def test_every_menu_entry_can_be_invoked(self):
        entries = self._menu_commands()
        self.assertGreaterEqual(len(entries), 8,
                                "the menu bar looks emptier than it should be")
        for label, invoke in entries:
            with self.subTest(menu=label):
                invoke()
                self.pump(0.35)
                # Close anything it opened, so the next entry starts clean.
                for child in list(self.root.winfo_children()):
                    if isinstance(child, tk.Toplevel):
                        child.destroy()
                self.pump(0.1)
                self.assertEqual(self.callback_errors, [],
                                 f"{label} raised")
        self.assertEqual(self.errors, [])

    def test_every_button_can_be_pressed(self):
        pressed = []

        def press(widget):
            for child in widget.winfo_children():
                press(child)
            if isinstance(widget, (ttk.Button, tk.Button)):
                text = str(widget.cget("text")).replace("\n", " ")
                # Quitting mid-test would take the window out from under us.
                if "quit" in text.lower():
                    return
                widget.invoke()
                pressed.append(text)
                self.pump(0.2)
                for child in list(self.root.winfo_children()):
                    if isinstance(child, tk.Toplevel):
                        child.destroy()

        press(self.root)
        self.pump(0.5)
        self.assertGreaterEqual(len(pressed), 10,
                                f"only found {pressed}")
        self.assertEqual(self.callback_errors, [])
        self.assertEqual(self.errors, [])

    def test_the_window_is_still_polling_afterwards(self):
        """A dead poll loop looks exactly like a frozen application."""
        for label, invoke in self._menu_commands():
            invoke()
            self.pump(0.2)
            for child in list(self.root.winfo_children()):
                if isinstance(child, tk.Toplevel):
                    child.destroy()
        self.pump(0.4)
        before = self.app.rows["Top"].position_var.get()
        self.app.platform.move_to_orientation(Orientation(3.0, 0.0, 0.0))
        self.pump(0.8)
        self.assertNotEqual(before, self.app.rows["Top"].position_var.get(),
                            "the readout stopped updating")


if __name__ == "__main__":
    unittest.main(verbosity=2)
