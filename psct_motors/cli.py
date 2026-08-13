"""
Command line tool for the pSCT focal-plane actuators.

This is the commissioning interface. Everything you need to do *before* the
GUI is useful -- confirming the register map, proving the word order, working
out how many counts make a millimetre, finding which way each actuator pushes,
checking the brakes actually click -- lives here.

    python -m psct_motors.cli --help

Every command takes `--simulate` to run against the built-in fake motors, so
you can rehearse a procedure at your desk. Nothing in this file writes to a
motor without either an explicit flag or a typed confirmation.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import List, Optional

from .config import (
    default_config, default_config_path, load_config, save_config,
)
from .jvl_motor import MotorFault
from .kinematics import Orientation
from .platform import FocalPlanePlatform, PlatformError
from .registers import REGISTERS, VERIFY
from .transport import ModbusError


# --------------------------------------------------------------------------
# Output helpers
# --------------------------------------------------------------------------

def out(msg: str = "") -> None:
    print(msg, flush=True)


def rule(title: str = "") -> None:
    out("-" * 72 if not title else f"--- {title} " + "-" * max(0, 67 - len(title)))


def confirm(prompt: str, assume_yes: bool = False) -> bool:
    if assume_yes:
        out(f"{prompt} [auto-yes]")
        return True
    try:
        return input(f"{prompt} [y/N] ").strip().lower() in ("y", "yes")
    except EOFError:
        return False


def ask_float(prompt: str) -> Optional[float]:
    try:
        raw = input(f"{prompt}: ").strip()
    except EOFError:
        return None
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        out(f"'{raw}' is not a number.")
        return None


def make_platform(args) -> FocalPlanePlatform:
    cfg = load_config(args.config)
    return FocalPlanePlatform(cfg=cfg, simulate=args.simulate, logger=out,
                              config_path=args.config)


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------

def cmd_init_config(args) -> int:
    path = args.config or default_config_path()
    import os
    if os.path.exists(path) and not args.force:
        out(f"{path} already exists. Use --force to overwrite it.")
        return 1
    cfg = default_config()
    cfg.validate()
    written = save_config(cfg, path)
    out(f"Wrote a starting configuration to {written}")
    out("")
    out("Before you connect to hardware, edit that file and set:")
    out("  * ip           -- the address of each of the three motors")
    out("  * azimuth_deg  -- where each actuator sits around the optical axis")
    out("  * radius_mm    -- its distance from the axis, from the camera drawing")
    out("  * brake.mode   -- 'auto' if the drive releases the brake, 'output' if")
    out("                    it is wired to a digital output you control")
    out("")
    out("Then run, in order:")
    out("  python -m psct_motors.cli verify-registers")
    out("  python -m psct_motors.cli check-direction --motor A")
    out("  python -m psct_motors.cli calibrate --motor A")
    return 0


def cmd_show_config(args) -> int:
    cfg = load_config(args.config)
    out(json.dumps({"path": args.config or default_config_path()}, indent=2))
    from .config import config_to_dict
    out(json.dumps(config_to_dict(cfg), indent=2))
    return 0


def cmd_status(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        state = platform.read_state()
        if args.json:
            out(json.dumps(state.as_dict(), indent=2))
            return 0

        rule("Actuators")
        header = (f"{'name':<6}{'mm':>12}{'counts':>14}{'target mm':>12}  "
                  f"{'mode':<20}{'brake':<12}")
        out(header)
        for m in state.motors:
            if m.comms_error:
                out(f"{m.name:<6}  ** {m.comms_error}")
                continue
            brake = m.brake.state.value + ("?" if m.brake.inferred else "")
            out(f"{m.name:<6}{m.position_mm:>12.4f}{m.position_counts:>14}"
                f"{m.target_mm:>12.4f}  {m.mode_text:<20}{brake:<12}")
            if m.error_bits:
                out(f"        errors: {m.error_text}")

        rule("Focal plane")
        if state.orientation_valid and state.orientation:
            out("  " + state.orientation.describe())
        else:
            out("  " + (state.message or "unavailable"))
        rule()
        out(f"moving: {state.moving}   errors: {state.any_error}")
        return 0
    finally:
        platform.disconnect()


def cmd_verify_registers(args) -> int:
    """Print the register table beside live values, so VERIFY entries can be
    checked against MacTalk and promoted."""
    rule("Register map")
    out(f"{'reg':>4} {'modbus':>7}  {'name':<16}{'bits':>5} {'confidence':<11} description")
    for r in REGISTERS:
        out(f"{r.number:>4} {r.address:>7}  {r.name:<16}{r.width:>5} {r.confidence:<11} "
            f"{r.description.splitlines()[0][:60]}")
    out("")
    out("CONFIRMED = checked on these motors.  DOCUMENTED = from JVL's published")
    out("register overview.  VERIFY = plausible but unchecked -- compare the live")
    out("value below against the same field in MacTalk before trusting it.")

    if args.offline:
        return 0

    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out("")
        out(f"Could not connect: {exc}")
        out("Re-run with --offline to see the table alone.")
        return 1
    try:
        for motor in platform.motors:
            rule(f"Live values from {motor.name}")
            for name, value in motor.read_diagnostics().items():
                marker = ""
                reg = next((r for r in REGISTERS if r.name == name), None)
                if reg and reg.confidence == VERIFY:
                    marker = "   <-- VERIFY against MacTalk"
                out(f"  {name:<16} {value}{marker}")
    finally:
        platform.disconnect()
    return 0


def cmd_detect(args) -> int:
    """Read PROG_VERSION both ways and report which word order is right."""
    platform = make_platform(args)
    failures = 0
    for motor in platform.motors:
        try:
            # Connect without the word-order check, which is what we are testing.
            motor.connect(verify_word_order=False)
        except (ModbusError, MotorFault) as exc:
            out(f"{motor.name}: could not connect -- {exc}")
            failures += 1
            continue
        try:
            detected = motor.detect_word_order()
            configured = motor.word_order
            verdict = "matches config" if detected is configured else (
                f"** MISMATCH: config says {configured.value} **")
            out(f"{motor.name}: detected {detected.value}  ({verdict})")
            if detected is not configured:
                failures += 1
        except (ModbusError, MotorFault) as exc:
            out(f"{motor.name}: detection failed -- {exc}")
            failures += 1
        finally:
            motor.disconnect()
    if failures:
        out("")
        out("Fix 'word_order' in the config for any mismatched actuator; until you "
            "do, every position read from it is wrong.")
    return 1 if failures else 0


def cmd_check_direction(args) -> int:
    """Find out which way an actuator pushes the focal plane.

    Direction signs are the classic silent failure: get one wrong and a pure
    focus command becomes a tilt, with everything still reporting success.
    """
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        motor = platform.motor(args.motor)
        start = motor.get_position_mm()
        out(f"{motor.name} is at {start:.4f} mm ({motor.get_position_counts()} counts).")
        out(f"About to move it {args.mm:+.3f} mm as this software currently "
            "understands the direction.")
        if not confirm("Watch the actuator. Proceed?", args.yes):
            return 1

        platform.move_actuator_mm(motor.name, args.mm, relative=True)
        end = motor.get_position_mm()
        out(f"{motor.name} is now at {end:.4f} mm.")
        out("")
        out("Did the focal plane move in the +z direction (the direction you want")
        out("a POSITIVE focus number to mean -- normally away from the secondary)?")
        if confirm("Moved the way the software says?", args.yes):
            out(f"Good. direction stays {motor.cfg.direction:+d} for {motor.name}.")
            return 0
        motor.cfg.direction = -motor.cfg.direction
        save_config(platform.cfg, args.config)
        out(f"Flipped direction for {motor.name} to {motor.cfg.direction:+d} and saved.")
        out("Re-run this command to confirm the new sign is right.")
        return 0
    finally:
        platform.disconnect()


def cmd_calibrate(args) -> int:
    """Measure counts-per-millimetre directly, without knowing the gear ratio.

    Move a known number of counts, measure the travel with a dial indicator,
    divide. This absorbs the gearbox, the screw lead and any fixed scaling
    error in one number, which is why it beats deriving the value from a
    drivetrain you would otherwise have to chase down.
    """
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        motor = platform.motor(args.motor)
        cfg = motor.cfg
        counts = int(args.counts)

        out(f"Calibrating {motor.name}.")
        out(f"  current scale : {cfg.resolved_counts_per_mm:.4f} counts/mm "
            f"({'measured' if cfg.scale_is_measured else 'derived from the drivetrain'})")
        out(f"  test move     : {counts} counts "
            f"(about {counts / cfg.resolved_counts_per_mm:.4f} mm at the current scale)")
        out("")
        out("Set up a dial indicator against the moving flange and zero it now.")
        if not confirm("Ready to move?", args.yes):
            return 1

        motor.ensure_position_mode()
        if motor.brake_is_software_controlled:
            motor.release_brake()
        start = motor.get_position_counts()
        target_counts = start + counts * cfg.direction

        target_mm_check = cfg.counts_to_mm(target_counts)
        try:
            motor.check_travel_limit(target_mm_check)
        except MotorFault as exc:
            out(str(exc))
            out("Use a smaller --counts, or move the actuator away from its limit first.")
            return 1

        motor.set_velocity(cfg.velocity_raw)
        motor.command_position_counts(target_counts)
        if not motor.wait_for_in_position():
            out("The move did not complete. Calibration abandoned.")
            return 1
        moved_counts = motor.get_position_counts() - start
        out(f"Moved {moved_counts} counts.")

        measured = args.measured_mm
        if measured is None:
            measured = ask_float("Displacement you measured, in mm (unsigned)")
        if measured is None or measured == 0:
            out("No measurement given; nothing was changed.")
            return 1

        new_scale = abs(moved_counts) / abs(measured)
        old_scale = cfg.resolved_counts_per_mm
        out("")
        out(f"  measured      : {abs(measured):.4f} mm for {abs(moved_counts)} counts")
        out(f"  new scale     : {new_scale:.4f} counts/mm")
        out(f"  previous      : {old_scale:.4f} counts/mm "
            f"({100.0 * (new_scale - old_scale) / old_scale:+.2f}% change)")
        # A wildly different answer usually means the indicator moved, the
        # wrong actuator was watched, or the units were mixed up -- worth
        # saying out loud rather than silently saving.
        if not 0.2 < new_scale / old_scale < 5.0:
            out("")
            out("  ** That is a long way from the previous value. Check that you")
            out("     measured the right actuator, in millimetres, and that the")
            out("     indicator did not slip.")
        if not confirm("Save this scale?", args.yes):
            out("Not saved.")
            return 1
        cfg.counts_per_mm = new_scale
        save_config(platform.cfg, args.config)
        out(f"Saved counts_per_mm = {new_scale:.4f} for {motor.name}.")
        out("Repeat for the other two actuators; they are not necessarily identical.")
        return 0
    finally:
        platform.disconnect()


def cmd_probe_brake(args) -> int:
    """Toggle the configured brake output and let the operator confirm it."""
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        motor = platform.motor(args.motor)
        cfg = motor.cfg.brake
        out(f"{motor.name}: brake mode is '{cfg.mode}'.")
        if cfg.mode == "none":
            out("No brake is configured for this actuator, so there is nothing to probe.")
            out("If it does have a brake, set brake.mode to 'auto' or 'output'.")
            return 1
        if cfg.mode == "auto":
            out("In 'auto' mode the drive controls the brake and software cannot")
            out("toggle it. Watch the brake while the mode changes instead:")
            out("  it should engage when the drive goes passive and release when")
            out("  Position mode is enabled.")
            status = motor.get_brake_status()
            out(f"Currently inferred as {status.state.label} -- {status.detail}")
            return 0

        out(f"Brake is on output register {cfg.output_register}, bit {cfg.output_bit}, "
            f"{'energised = released' if cfg.energized_releases else 'energised = engaged'}.")
        out("The drive will be enabled first, so the motor holds the load while the")
        out("brake is off. Listen for the brake clicking.")
        if not confirm("Proceed?", args.yes):
            return 1

        motor.ensure_position_mode()
        motor.command_position_counts(motor.get_position_counts())  # hold still
        for _ in range(args.cycles):
            motor.release_brake()
            out(f"  released -- reads back {motor.get_brake_status().state.value}")
            time.sleep(1.0)
            motor.engage_brake()
            out(f"  engaged  -- reads back {motor.get_brake_status().state.value}")
            time.sleep(1.0)
        out("")
        if confirm("Did the brake click in time with those messages?", args.yes):
            out("Brake control confirmed for this actuator.")
            return 0
        out("Then the wiring does not match the config. Check output_register,")
        out("output_bit and energized_releases, and re-run.")
        return 1
    finally:
        platform.disconnect()


def cmd_preview(args) -> int:
    """Show what a move would do, touching no hardware."""
    cfg = load_config(args.config)
    platform = FocalPlanePlatform(cfg=cfg, simulate=True, config_path=args.config)
    target = _orientation_from_args(args)
    out(f"Target: {target.describe()}")
    rule("Actuator targets")
    for name, mm in platform.preview(target).items():
        actuator = cfg.actuator(name)
        inside = actuator.min_travel_mm <= mm <= actuator.max_travel_mm
        flag = "" if inside else "   ** OUTSIDE TRAVEL LIMITS **"
        out(f"  {name:<4} {mm:>10.4f} mm{flag}")
    rule("Travel budget")
    span = platform.geometry.actuator_span_for_tilt(target.total_tilt_deg)
    out(f"  a {target.total_tilt_deg:.4f} deg tilt costs {span:.3f} mm of "
        "peak-to-peak actuator travel")
    return 0


def _orientation_from_args(args) -> Orientation:
    if getattr(args, "tilt_magnitude", None) is not None:
        return Orientation.from_polar_tilt(
            args.focus, args.tilt_magnitude, args.azimuth or 0.0
        )
    return Orientation(args.focus, args.tip or 0.0, args.tilt or 0.0)


def cmd_move(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        current = platform.read_orientation()
        target = _orientation_from_args(args)
        out(f"Now:    {current.describe()}")
        out(f"Target: {target.describe()}")
        rule("Actuator targets")
        for name, mm in platform.preview(target).items():
            out(f"  {name:<4} {mm:>10.4f} mm")
        try:
            platform.check_orientation(target, current=current)
        except PlatformError as exc:
            out(str(exc))
            return 1
        if not confirm("Command this move?", args.yes):
            return 1
        state = platform.move_to_orientation(target, wait=not args.no_wait)
        if state.orientation:
            out(f"Done:   {state.orientation.describe()}")
        return 0
    except PlatformError as exc:
        out(f"Move failed: {exc}")
        return 1
    finally:
        platform.disconnect()


def cmd_move_relative(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        current = platform.read_orientation()
        target = current.offset_by(args.dfocus, args.dtip, args.dtilt)
        out(f"Now:    {current.describe()}")
        out(f"Target: {target.describe()}")
        try:
            platform.check_orientation(target, current=current)
        except PlatformError as exc:
            out(str(exc))
            return 1
        if not confirm("Command this move?", args.yes):
            return 1
        state = platform.move_to_orientation(target, wait=not args.no_wait)
        if state.orientation:
            out(f"Done:   {state.orientation.describe()}")
        return 0
    except PlatformError as exc:
        out(f"Move failed: {exc}")
        return 1
    finally:
        platform.disconnect()


def cmd_jog(args) -> int:
    """Move one actuator, for commissioning only."""
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        motor = platform.motor(args.motor)
        out(f"{motor.name}: {motor.get_position_mm():.4f} mm -> "
            f"{motor.get_position_mm() + args.mm:.4f} mm")
        if not confirm("Jog this single actuator?", args.yes):
            return 1
        status = platform.move_actuator_mm(motor.name, args.mm, relative=True)
        out(f"{motor.name} now at {status.position_mm:.4f} mm.")
        return 0
    except (PlatformError, MotorFault) as exc:
        out(f"Jog failed: {exc}")
        return 1
    finally:
        platform.disconnect()


def cmd_set_zero(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        out("This defines the CURRENT mechanical position as focus 0, tip 0, tilt 0.")
        out("Do it only with the focal plane at a position you have independently")
        out("established -- every later command is measured from here.")
        for motor in platform.motors:
            out(f"  {motor.name}: {motor.get_position_counts()} counts")
        if not confirm("Set this as the zero reference?", args.yes):
            return 1
        result = platform.set_zero_here(persist=True)
        out(f"Zero set: {result}")
        return 0
    finally:
        platform.disconnect()


def cmd_stop(args) -> int:
    """Controlled stop: decelerate and hold, drives stay enabled."""
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        platform.stop()
        out("All three actuators commanded to stop and hold.")
        return 0
    finally:
        platform.disconnect()


def cmd_passivate(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        out("This engages the brakes and turns the drives OFF. With the drives off,")
        out("the load is held by the brakes and screw friction alone.")
        if not confirm("Passivate all three motors?", args.yes):
            return 1
        platform.emergency_passivate()
        out("Done.")
        return 0
    finally:
        platform.disconnect()


def cmd_brake(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        if args.action == "status":
            for motor in platform.motors:
                status = motor.get_brake_status()
                tag = " (inferred)" if status.inferred else ""
                out(f"  {motor.name:<4} {status.state.label}{tag}   {status.detail}")
            return 0
        engage = args.action == "engage"
        if not engage:
            out("Releasing a brake removes the mechanical hold on the actuator.")
            out("The drives must be enabled and holding first.")
            if not confirm("Release the brakes?", args.yes):
                return 1
        results = platform.set_all_brakes(engaged=engage)
        failed = False
        for name, result in results.items():
            out(f"  {name:<4} {result}")
            failed = failed or result != "ok"
        return 1 if failed else 0
    finally:
        platform.disconnect()


def cmd_clear_errors(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        for name, value in platform.clear_all_errors().items():
            if value == 0:
                out(f"  {name:<4} clear")
            elif value < 0:
                out(f"  {name:<4} could not be read")
            else:
                out(f"  {name:<4} still 0x{value:08X} -- latched fault; use MacTalk's "
                    "clear or power-cycle the motor")
        return 0
    finally:
        platform.disconnect()


def cmd_demo(args) -> int:
    """Exercise ONE motor: capabilities, then deliberate faults.

    Deliberately does not use the platform or the kinematics -- it works on a
    single motor in counts and revolutions, so it is useful on a bench with one
    motor and no calibration done.
    """
    from .demo import DemoRunner, all_drills, build_motor, select_drills

    if args.list:
        rule("Available drills")
        out(f"{'name':<22}{'category':<12}{'flags':<22}summary")
        for drill in all_drills():
            flags = []
            if drill.needs_motion:
                flags.append("moves")
            if drill.needs_operator:
                flags.append("needs you")
            out(f"{drill.name:<22}{drill.category:<12}{','.join(flags):<22}"
                f"{drill.summary}")
        out("")
        out("Run a subset with:  --only identity,fault-comms")
        out("Or a whole category with:  --category fault")
        return 0

    try:
        drills = select_drills(
            only=[n.strip() for n in args.only.split(",")] if args.only else None,
            categories=[c.strip() for c in args.category.split(",")] if args.category else None,
            include_operator=not args.no_operator,
        )
    except ValueError as exc:
        out(str(exc))
        return 2
    if not drills:
        out("No drills matched that selection. Use --list to see what exists.")
        return 2

    motor = build_motor(args.config, args.motor, args.simulate)
    try:
        motor.connect(verify_word_order=False)
    except (ModbusError, MotorFault) as exc:
        out(f"Could not connect to motor {args.motor}: {exc}")
        out("")
        out("Check the IP address for this actuator in the config, that the motor")
        out("is powered, and that MacTalk is not holding the connection.")
        out("To try the demo with no hardware at all, add --simulate.")
        return 1

    def ask(prompt: str) -> bool:
        return confirm(prompt, args.yes)

    runner = DemoRunner(motor, out=out, ask=ask,
                        allow_motion=args.allow_motion,
                        range_revs=args.range_revs)
    try:
        return runner.run(drills)
    except KeyboardInterrupt:
        out("")
        out("Interrupted. Stopping the motor and leaving it passive.")
        try:
            runner.injector.clear()
            motor.stop()
            motor.passivate()
        except (ModbusError, MotorFault) as exc:
            out(f"Could not stop cleanly: {exc}")
            out("If the shaft is still turning, remove drive power.")
        return 130
    finally:
        motor.disconnect()


def cmd_motor_gui(args) -> int:
    """Bench GUI for a single motor."""
    from .single_gui import main as single_main
    return single_main(motor_name=args.motor, config_path=args.config,
                       simulate=args.simulate, log_path=args.log)


def cmd_diagnose(args) -> int:
    """Answer 'why is this motor not taking position commands?'"""
    from .demo import build_motor
    from .diagnostics import diagnose

    motor = build_motor(args.config, args.motor, args.simulate)
    try:
        motor.connect(verify_word_order=False)
    except (ModbusError, MotorFault) as exc:
        out(f"Could not connect to motor {args.motor}: {exc}")
        return 1
    try:
        result = diagnose(motor, probe_writes=not args.no_write_probe)
        rule(f"Diagnosis for motor {args.motor}")
        out(result.as_text())
        out("")
        if result.blockers:
            out(f"{len(result.blockers)} thing(s) would stop this motor moving.")
            return 1
        out("Nothing found that would stop this motor moving.")
        return 0
    finally:
        motor.disconnect()


def cmd_watch(args) -> int:
    """Record what the motor does, so a later hang can be explained."""
    from .demo import build_motor
    from .eventlog import EventLog, MotorWatcher, default_log_path

    path = args.log or default_log_path(args.motor)
    motor = build_motor(args.config, args.motor, args.simulate)
    try:
        motor.connect(verify_word_order=False)
    except (ModbusError, MotorFault) as exc:
        out(f"Could not connect to motor {args.motor}: {exc}")
        return 1

    log = EventLog(path=path, on_event=lambda e: out(e.as_line()))
    watcher = MotorWatcher(motor, log, interval_s=args.interval,
                           slow_transaction_s=args.slow_threshold)
    out(f"Recording motor {args.motor} to {path}")
    out("Only changes are logged, not every poll. Press Ctrl-C to stop.")
    out("")
    watcher.start()
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        out("")
    finally:
        watcher.stop()
        log.close()
        motor.disconnect()
        out(f"Recording saved to {path}")
    return 0


def cmd_show_log(args) -> int:
    """Replay a recorded log, newest last."""
    from .eventlog import read_log
    events = read_log(args.file)
    if not events:
        out(f"No events in {args.file}")
        return 1
    from .eventlog import _SEVERITY_ORDER
    floor = _SEVERITY_ORDER.get(args.severity, 0)
    shown = [e for e in events if _SEVERITY_ORDER.get(e.severity, 0) >= floor]
    rule(f"{args.file}  ({len(shown)} of {len(events)} events)")
    for event in shown:
        out(event.as_line())
        for key, value in event.data.items():
            out(f"{'':>12}{key} = {value}")
    return 0


def cmd_gui(args) -> int:
    from .gui import main as gui_main
    return gui_main(config_path=args.config, simulate=args.simulate)


def cmd_server(args) -> int:
    from .server import serve
    return serve(host=args.host, port=args.port, config_path=args.config,
                 simulate=args.simulate)


# --------------------------------------------------------------------------
# Argument parsing
# --------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m psct_motors.cli",
        description="Control and commissioning for the pSCT focal-plane actuators.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
commissioning order
-------------------
  init-config          write a starting config, then edit it
  detect               confirm the Modbus word order on each motor
  verify-registers     check the register map against MacTalk
  check-direction      confirm which way each actuator pushes  (per motor)
  calibrate            measure counts per millimetre           (per motor)
  probe-brake          confirm brake control and polarity      (per motor)
  set-zero             define the reference orientation
  status / move        normal operation

one motor on a bench
--------------------
  motor-gui            live GUI: state, errors, fault injection, event log
  demo                 exercise a single motor and deliberately provoke
                       faults, to see the error handling work
  diagnose             explain why a motor is not taking position commands
  watch                record changes to a log file, to explain a later hang
  show-log             replay a recording
""",
    )
    parser.add_argument("--config", help="path to the configuration JSON")
    parser.add_argument("--simulate", action="store_true",
                        help="run against built-in fake motors, no hardware needed")
    parser.add_argument("-y", "--yes", action="store_true",
                        help="answer yes to confirmations (for scripts)")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("init-config", help="write a starting configuration file")
    p.add_argument("--force", action="store_true", help="overwrite an existing file")
    p.set_defaults(func=cmd_init_config)

    p = sub.add_parser("show-config", help="print the active configuration")
    p.set_defaults(func=cmd_show_config)

    p = sub.add_parser("status", help="read positions, orientation, brakes and errors")
    p.add_argument("--json", action="store_true", help="machine-readable output")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("verify-registers",
                       help="print the register map beside live values")
    p.add_argument("--offline", action="store_true",
                   help="print the table only, without connecting")
    p.set_defaults(func=cmd_verify_registers)

    p = sub.add_parser("detect", help="detect the Modbus word order on each motor")
    p.set_defaults(func=cmd_detect)

    p = sub.add_parser("check-direction",
                       help="confirm which way an actuator moves the focal plane")
    p.add_argument("--motor", required=True, help="actuator name, e.g. A")
    p.add_argument("--mm", type=float, default=0.5,
                   help="test move size in mm (default 0.5)")
    p.set_defaults(func=cmd_check_direction)

    p = sub.add_parser("calibrate", help="measure counts per millimetre")
    p.add_argument("--motor", required=True, help="actuator name, e.g. A")
    p.add_argument("--counts", type=int, default=409600,
                   help="counts to move for the test (default 409600, one motor rev)")
    p.add_argument("--measured-mm", type=float,
                   help="measured displacement, if you already have it")
    p.set_defaults(func=cmd_calibrate)

    p = sub.add_parser("probe-brake", help="toggle a brake and confirm it responds")
    p.add_argument("--motor", required=True)
    p.add_argument("--cycles", type=int, default=2)
    p.set_defaults(func=cmd_probe_brake)

    p = sub.add_parser("preview", help="show a move's actuator targets without moving")
    _add_orientation_args(p)
    p.set_defaults(func=cmd_preview)

    p = sub.add_parser("move", help="move to an absolute orientation")
    _add_orientation_args(p)
    p.add_argument("--no-wait", action="store_true",
                   help="return as soon as the move is commanded")
    p.set_defaults(func=cmd_move)

    p = sub.add_parser("move-rel", help="move relative to the current orientation")
    p.add_argument("--dfocus", type=float, default=0.0, help="mm")
    p.add_argument("--dtip", type=float, default=0.0, help="degrees about +x")
    p.add_argument("--dtilt", type=float, default=0.0, help="degrees about +y")
    p.add_argument("--no-wait", action="store_true")
    p.set_defaults(func=cmd_move_relative)

    p = sub.add_parser("jog", help="move a single actuator (commissioning)")
    p.add_argument("--motor", required=True)
    p.add_argument("--mm", type=float, required=True, help="relative move in mm")
    p.set_defaults(func=cmd_jog)

    p = sub.add_parser("set-zero", help="define the current position as the reference")
    p.set_defaults(func=cmd_set_zero)

    p = sub.add_parser("stop", help="controlled stop: decelerate and hold")
    p.set_defaults(func=cmd_stop)

    p = sub.add_parser("passivate", help="brakes on, drives off")
    p.set_defaults(func=cmd_passivate)

    p = sub.add_parser("brake", help="engage, release or report the brakes")
    p.add_argument("action", choices=["status", "engage", "release"])
    p.set_defaults(func=cmd_brake)

    p = sub.add_parser("clear-errors", help="best-effort error clear on all motors")
    p.set_defaults(func=cmd_clear_errors)

    p = sub.add_parser(
        "demo",
        help="exercise ONE motor: capabilities and deliberate faults",
        description=(
            "Single-motor demo and fault drills. Works on a bench with one "
            "motor, in counts and revolutions, with no calibration needed. "
            "Nothing turns the shaft unless you pass --allow-motion."
        ),
    )
    p.add_argument("--motor", default="A", help="actuator name from the config (default A)")
    p.add_argument("--allow-motion", action="store_true",
                   help="permit drills that turn the shaft")
    p.add_argument("--range-revs", type=float, default=2.0,
                   help="how far the shaft may turn either way, in revolutions "
                        "(default 2)")
    p.add_argument("--only", help="comma-separated drill names to run")
    p.add_argument("--category", help="comma-separated categories: capability, "
                                      "fault, real-fault")
    p.add_argument("--no-operator", action="store_true",
                   help="skip drills that ask you to unplug things")
    p.add_argument("--list", action="store_true", help="list the drills and exit")
    p.set_defaults(func=cmd_demo)

    p = sub.add_parser(
        "motor-gui",
        help="bench GUI for ONE motor: live state, errors, fault injection, log",
        description=(
            "Single-motor bench GUI. Works in revolutions, needs no "
            "calibration, records an event log to disk as it runs."
        ),
    )
    p.add_argument("--motor", default="A", help="actuator name (default A)")
    p.add_argument("--log", help="path for the JSONL event log")
    p.set_defaults(func=cmd_motor_gui)

    p = sub.add_parser(
        "diagnose",
        help="explain why a motor is not taking position commands",
    )
    p.add_argument("--motor", default="A")
    p.add_argument("--no-write-probe", action="store_true",
                   help="skip the P_SOLL readback probe (which commands the "
                        "position the motor is already at, so cannot move it)")
    p.set_defaults(func=cmd_diagnose)

    p = sub.add_parser(
        "watch",
        help="record mode, error, target and timing changes to a log file",
    )
    p.add_argument("--motor", default="A")
    p.add_argument("--log", help="path for the JSONL event log")
    p.add_argument("--interval", type=float, default=0.5,
                   help="seconds between polls (default 0.5)")
    p.add_argument("--slow-threshold", type=float, default=1.0,
                   help="log any poll slower than this many seconds (default 1)")
    p.set_defaults(func=cmd_watch)

    p = sub.add_parser("show-log", help="replay a recorded event log")
    p.add_argument("file")
    p.add_argument("--severity", default="DEBUG",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    p.set_defaults(func=cmd_show_log)

    p = sub.add_parser("gui", help="launch the three-motor focal-plane application")
    p.set_defaults(func=cmd_gui)

    p = sub.add_parser("server", help="run the JSON-over-TCP bridge (for LabVIEW)")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5020)
    p.set_defaults(func=cmd_server)

    return parser


def _add_orientation_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--focus", type=float, required=True, help="mm along the optical axis")
    p.add_argument("--tip", type=float, help="degrees about +x")
    p.add_argument("--tilt", type=float, help="degrees about +y")
    p.add_argument("--tilt-magnitude", type=float,
                   help="total tilt in degrees (polar form; use with --azimuth)")
    p.add_argument("--azimuth", type=float,
                   help="azimuth of the tilt in degrees CCW from +x")


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except KeyboardInterrupt:
        out("")
        out("Interrupted. Note that this did NOT stop a move already running on the")
        out("motors -- run `stop` if anything is still in motion.")
        return 130
    except (PlatformError, MotorFault, ModbusError) as exc:
        out(f"Error: {exc}")
        return 1
    except (ValueError, KeyError) as exc:
        out(f"Configuration error: {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
