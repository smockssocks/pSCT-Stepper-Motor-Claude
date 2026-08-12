# pSCT focal-plane actuator control

Control software for the three JVL MIS232 integrated stepper motors that
position the pSCT camera's focal plane.

You give it an orientation — focus, tip and tilt. It works out which motors
move and by how much, checks the move is safe, and drives all three together.
No hand calculation, no working out which motor to nudge.

```
python -m psct_motors.cli gui --simulate      # try it, no hardware needed
```

---

## Contents

- [What this replaces](#what-this-replaces)
- [The mechanism](#the-mechanism)
- [Install](#install)
- [Quick start](#quick-start)
- [Commissioning](#commissioning-do-this-before-trusting-anything)
- [Using it](#using-it)
- [LabVIEW](#labview)
- [Safety design](#safety-design)
- [What is verified and what is not](#what-is-verified-and-what-is-not)
- [Layout](#layout)
- [Testing](#testing)

---

## What this replaces

Before: MacTalk, one motor at a time, with someone working out by hand how far
each motor had to turn to produce the tilt they wanted, given the screw and
which motors to move.

Now: type the orientation you want. The software does the geometry, refuses
moves that would run an actuator out of travel, moves all three axes so they
arrive together, and shows you where the focal plane actually is.

The starting point for this was a single-motor Tkinter script that already
talked to a motor over Modbus TCP. What carried over from it, because it was
right and had been checked against real hardware:

- JVL register numbers map to Modbus addresses by **doubling** them.
- Every access is **two 16-bit words**, even for registers that are natively
  16-bit — a one-word write to `MODE_REG` gets rejected.
- The word order on these motors is **low word first**.
- `MODE_REG` = 2, `P_SOLL` = 3, `P_IST` = 10, `V_SOLL` = 5, `ERR_BITS` = 35,
  and 409600 counts per revolution.

Everything else is new.

---

## The mechanism

The camera's inner structure, which carries the focal plane, hangs off the
outer structure on **three ball pin joints**. Behind each joint a motor turns a
drive screw that pushes or pulls a flange, and a rail system constrains that
flange to move only along **z**, the optical axis.

So each actuator contributes exactly one degree of freedom, and three of them
determine the plane completely:

| you ask for | it means |
|---|---|
| **focus** (mm) | translation along the optical axis — distance to the secondary |
| **tip** (deg) | rotation about +x; positive tip raises the +y side |
| **tilt** (deg) | rotation about +y; positive tilt lowers the +x side |

A plane through three points has a closed-form solution, so the conversion both
ways is exact — no iteration, no fitting. Commanded and reported orientations
mean the same thing, which matters because otherwise every closed-loop
adjustment drifts.

Tilt is also available in polar form — *"0.3 degrees, sloping up towards 45
degrees azimuth"* — which is often easier to think about at the telescope.

**Lever arms matter.** With actuators on a 500 mm radius, one degree of tilt
costs about 17 mm of peak-to-peak actuator travel, a third of the total range.
`cli preview` tells you the cost before you commit.

---

## Install

Python 3.8 or newer.

```
pip install pymodbus
```

That is the only dependency, and only for real hardware — the simulator, the
kinematics and the tests need nothing beyond the standard library. Tkinter
comes with Python on Windows and macOS; on Debian/Ubuntu it is
`sudo apt install python3-tk`.

---

## Quick start

```
# 1. Write a starting configuration
python -m psct_motors.cli init-config

# 2. Edit config/psct_motors.json -- IPs, geometry, brake wiring

# 3. Try the GUI against fake motors first
python -m psct_motors.cli gui --simulate

# 4. When the config is right, connect for real
python -m psct_motors.cli status
```

Every command takes `--simulate`. Get comfortable there first; nothing can move
until you drop the flag.

---

## Commissioning: do this before trusting anything

The defaults are a starting point, not the truth about your hardware. Work
through this in order. It is quick, and each step catches a failure that is
otherwise silent.

### 1. Confirm the word order

```
python -m psct_motors.cli detect
```

Reads `PROG_VERSION` from each motor both ways. The wrong word order turns a
firmware version into a huge number, which is unmistakable. If a motor
mismatches, fix `word_order` in the config — until you do, every position from
that motor is wrong. Connecting checks this automatically and refuses to
proceed on a mismatch.

### 2. Check the register map

```
python -m psct_motors.cli verify-registers
```

Prints every register this software uses with a confidence marker, next to the
live value. Compare the `VERIFY` rows against the same fields in MacTalk and
update the markers in `psct_motors/registers.py` as you confirm them. See
[what is verified](#what-is-verified-and-what-is-not).

### 3. Check each actuator's direction

```
python -m psct_motors.cli check-direction --motor A
```

Moves the actuator and asks whether the focal plane went the way the software
thinks. Answer honestly; if it is wrong, the command flips `direction` and
saves it.

**Why this matters:** with one direction sign wrong, a pure focus command
becomes a tilt, and everything still reports success. This is the single most
important step here.

Repeat for B and C.

### 4. Measure counts per millimetre

```
python -m psct_motors.cli calibrate --motor A
```

Moves a known number of counts and asks what displacement you measured with a
dial indicator. It divides, and saves the result.

**You do not need to know the gear ratio.** That is the point of measuring:
one number absorbs the gearbox, the screw lead and any fixed scaling error.
Chasing the drivetrain down on paper and hoping the numbers are right is the
slower and less reliable path.

If you leave `counts_per_mm` unset, the software falls back to deriving it:

```
counts_per_mm = counts_per_rev * gear_ratio / screw_lead_mm
```

The default `screw_lead_mm` of 2.54 is an **inference**, not a measurement: the
published pSCT camera description gives 12.7 µm of z travel per motor step and
5.08 cm (2 in) of total travel, and 12.7 µm per 1.8° full step implies a
2.54 mm/rev lead with no reduction. Plausible, unverified, and irrelevant once
you have measured. The GUI warns on startup for any actuator still using a
derived scale.

Repeat for B and C — they are not necessarily identical.

### 5. Confirm brake control

```
python -m psct_motors.cli probe-brake --motor A
```

Toggles the brake with the drive enabled and holding, and asks you to confirm
you heard it click. See [brakes](#brakes) for the modes.

### 6. Set the geometry

Edit `azimuth_deg` and `radius_mm` for each actuator from the camera drawing.
`azimuth_deg` is measured counter-clockwise from +x looking along −z;
`radius_mm` is the ball joint's distance from the optical axis.

Getting `radius_mm` wrong scales all your tilt angles by a constant factor, so
it is worth measuring rather than guessing.

### 7. Set the zero reference

With the focal plane at a position you have independently established:

```
python -m psct_motors.cli set-zero
```

Every later command is measured from here. The offset is stored in the config
file, not written into the motors, so MacTalk and this software never disagree
about the motor's own position.

---

## Using it

### GUI

```
python -m psct_motors.cli gui
```

- **STOP** across the top, always live.
- Current orientation in mm, degrees, arcmin and arcsec.
- Absolute moves, with **Preview** to see the actuator targets first.
- Nudge buttons for relative focus/tip/tilt.
- Per-actuator position, mode, brake lamp, brake buttons and jog.
- A timestamped log of everything the application did.

### Command line

```
python -m psct_motors.cli status
python -m psct_motors.cli preview --focus 25 --tip 0.1        # touches nothing
python -m psct_motors.cli move --focus 25 --tip 0.1 --tilt 0
python -m psct_motors.cli move --focus 25 --tilt-magnitude 0.2 --azimuth 45
python -m psct_motors.cli move-rel --dfocus 0.5
python -m psct_motors.cli stop
python -m psct_motors.cli brake status
```

`-y` skips confirmations, for scripts. `status --json` is machine-readable.

### Python

```python
from psct_motors import FocalPlanePlatform, Orientation

with FocalPlanePlatform() as platform:
    print(platform.read_orientation().describe())
    platform.move_to_orientation(Orientation(focus_mm=25.0, tip_deg=0.1))
    platform.move_relative(d_focus_mm=0.5)
```

---

## LabVIEW

See **[labview/README.md](labview/README.md)**.

Short version: run the bridge and talk to it over TCP.

```
python -m psct_motors.cli server --simulate
```

One JSON object per line, one back. LabVIEW's TCP primitives do not care which
Python you have installed or whether it is 32- or 64-bit — which is the usual
reason a LabVIEW/Python integration refuses to load. A `lv_*` function API for
LabVIEW's native Python node is also provided if you prefer that route, with
its trade-offs documented.

---

## Safety design

This drives a real optical assembly. A few decisions are deliberate.

### STOP is not "drive off"

Two separate controls, because they do different things:

| | what it does | when |
|---|---|---|
| **STOP** | Sets each target to its current position. Motors decelerate on their own ramp and **actively hold**. | Normal "stop now". |
| **EMERGENCY** | Engages the brakes, then sets `MODE_REG = 0`. Drive output off. | You want the drive electrically off. |

Cutting the drive is the more drastic action, not the safer one: with the drive
passive the motor holds nothing, and the load rests on the brakes and screw
friction. STOP keeps the axis under control.

STOP never queues. `FocalPlanePlatform.stop()` takes no move lock, the GUI runs
it on its own thread regardless of what else is busy, and the TCP bridge exempts
it from the command lock. A stop button that waits its turn is not a stop
button. There is a test for this on both the bridge and the GUI.

### Nothing moves until the whole move is checked

Before a single register is written, a move is checked against:

- the commanded focus, against the platform's focus limits
- the **total** tilt, against the tilt limit — tip and tilt can each be small
  while their combination is not
- each actuator's resulting target, against that actuator's travel limits
- the size of the step, against the single-step limits

All problems are reported at once, and if there is any problem, **nothing is
commanded**. A partially executed combined move is exactly the state that racks
the ball joints.

### The three axes arrive together

Velocities are scaled by distance so all three finish at the same moment.
Without that, the shortest move lands first and the plate sits at an
orientation nobody asked for until the last axis catches up, pivoting on its
joints the whole time.

### Brakes

Three modes, per actuator, because how the brake is wired is an installation
choice:

| `brake.mode` | meaning |
|---|---|
| `auto` (default) | The drive controls the brake: released when the drive is enabled, engaged when passive. Software cannot command it. The GUI shows an **inferred** state and says so. |
| `output` | The brake is on a digital output. Software controls it and **reads the state back**. |
| `none` | No brake under software control. Shown as unknown. |

Interlocks:

- Releasing a brake while the drive is **passive** is refused — nothing would be
  holding the actuator.
- Brakes are released automatically before a move and given time to physically
  release before motion is commanded.
- `passivate` engages the brakes **before** cutting the drive, not after.

### Mode changes are verified

Every `MODE_REG` write is read back. If MacTalk is still connected it fights for
control and quietly reverts the mode; the symptom is a move that is accepted and
then does nothing at all. The error message names that cause, because it is
almost always the cause.

### Orientation is withheld rather than guessed

If any actuator cannot be read, the orientation is reported as unavailable
rather than computed from two live positions and one stale one. A confidently
wrong tilt is worse than a blank.

---

## What is verified and what is not

Written without access to a copy of the JVL manual, so every register carries a
confidence marker in `psct_motors/registers.py`:

| marker | meaning |
|---|---|
| `CONFIRMED` | Verified against these motors and/or MacTalk. |
| `DOCUMENTED` | From JVL's published MIS23x/SMC75 register overview. |
| `VERIFY` | Plausible from JVL's conventions, **not** checked on hardware. |

Confirmed: `MODE_REG` (2), `P_SOLL` (3), `P_IST` (10), `V_SOLL` (5),
`ERR_BITS` (35), 409600 counts/rev, low-high word order, register doubling,
two-word access.

Documented: `PROG_VERSION` (1), `A_SOLL` (6), `RUN_CURRENT` (7),
`STANDBY_TIME` (8), `STANDBY_CURRENT` (9), `V_IST` (12).

**Needs verification before you rely on it:**

- `P_NEW` (4), `FLWERR` (20), `STATUSBITS` (25), `WARN_BITS` (36), `P_HOME` (38).
- The **individual bit meanings** in `ERROR_BITS` and `STATUS_BITS`. The
  registers themselves are right — `ERR_BITS` of 0 means healthy — but the
  bit-to-text mapping is unchecked. Raw hex is always shown next to the decoded
  text so a mis-mapped bit is still visible.
- The **brake output register** (19 by default) and its bit and polarity. This
  is an installation detail; `probe-brake` is how you confirm it.
- `screw_lead_mm` and `gear_ratio`. Superseded by `calibrate`.
- `radius_mm` and `azimuth_deg` — placeholders until read off the drawings.

Run `verify-registers` beside MacTalk and promote the markers as you confirm
them. Nothing here fails silently on an unverified value: reads that cannot be
made are reported, not defaulted.

---

## Layout

```
psct_motors/
  registers.py    JVL register numbers, word order, bit decoding
  transport.py    Modbus TCP wire; pymodbus version differences absorbed
  jvl_motor.py    one motor: position, mode, brake, errors, motion complete
  kinematics.py   three actuator heights <-> (focus, tip, tilt)
  platform.py     all three driven together, with limits and interlocks
  config.py       the configuration model, loaded from one JSON file
  simulator.py    a fake motor at the transport boundary
  gui.py          desktop application
  cli.py          commissioning, calibration and scripted moves
  server.py       JSON-over-TCP bridge
  labview_api.py  flat function API for LabVIEW's Python node
labview/          how to wire it up in LabVIEW
tests/            unit and end-to-end tests
config/           your configuration lives here
```

The simulator substitutes at the **transport** boundary, so everything above it
— register doubling, word-order packing, mode verification, move sequencing,
limit checks — is the real production code whether or not hardware is attached.

`pymodbus` renamed the keyword identifying the target device three times
(`unit=` → `slave=` → `device_id=`) and moved `count` between positional and
keyword-only. `transport.py` inspects the installed client's signature once at
construction and adapts, so this works across pymodbus 2.x, 3.x and 4.x.

---

## Testing

```
python -m unittest discover -s tests -v
```

120 tests, no hardware needed. The GUI tests skip automatically without a
display; to run them headlessly:

```
xvfb-run -a python3 -m unittest tests.test_gui -v
```

Coverage worth knowing about:

- Kinematics round-trip exactly, on symmetric and lopsided triangles.
- Sign conventions are pinned: positive tip raises +y, positive tilt lowers +x.
- Refused moves command **nothing** — actuator positions are checked unchanged.
- STOP interrupts an in-flight move, on both the TCP bridge and the GUI, and
  the interrupted move reports failure rather than claiming success.
- A wrong word order is caught at connect.
- A mode that will not stick is reported as a fighting client.
- Brake polarity, both ways round, plus the passive-drive interlock.
- Losing a motor mid-poll withholds the orientation instead of guessing it.
- Malformed JSON on the bridge gets an error reply and the connection survives.
