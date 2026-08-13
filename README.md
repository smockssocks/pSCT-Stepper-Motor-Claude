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
- [One motor on the bench](#one-motor-on-the-bench)
- [When it stops taking commands](#when-it-stops-taking-commands)
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

## One motor on the bench

You do not need all three motors to make progress. The demo exercises **one**
motor on its own — no kinematics, no platform, no calibration — and then breaks
things on purpose so you can watch the error handling work.

```
python -m psct_motors.cli demo --list                    # what it will do
python -m psct_motors.cli demo --motor A                 # read-only drills
python -m psct_motors.cli demo --motor A --allow-motion  # include moving ones
```

It reports in **counts, revolutions and degrees of motor shaft** — never
millimetres. On a bare shaft there is nothing to convert millimetres to, so
inventing a number would be worse than leaving it out. That is also why this
works before you have measured anything.

### What it runs

**Capability drills** — firmware and word order, a full register dump with
confidence markers, position noise floor, mode changes, a quarter-turn move,
four-cycle repeatability, a timed speed comparison, STOP mid-move, an
out-of-range refusal, and the brake with its interlock.

**Fault drills** — a nonexistent register, comms loss, a drive fault injected
*during* a move, another client overriding the mode, an axis that will not
move, swapped register words, and a link losing packets at random.

**Real-fault drills** — these ask you to physically unplug the Ethernet cable,
or to open MacTalk alongside this software. Skip them with `--no-operator`, or
decline individually at the prompt.

Each drill prints what it is about to do, what the software is *supposed* to
do, what actually happened, and what to do if you meet it for real on the
telescope. It exits non-zero if anything failed.

### How the faults are made

Most are injected: `faults.py` wraps the live connection and tampers with the
register traffic on the way past, so the motor *appears* to have faulted and
everything above reacts exactly as it would in the field.

Be clear about what that proves. It proves your **handling** is right — that a
fault is noticed, the move is abandoned, the axis is halted, the operator is
told something useful, and recovery works. That is the part with bugs in it.
It does **not** prove the motor sets the bit you think it sets; only the
hardware can tell you that, which is why the bit meanings are marked VERIFY and
raw hex is always printed next to the decoded text.

Injection only ever tampers with values read back and with whether a
transaction succeeds. It never invents a write, never changes a target, and
never enables a drive.

### Safety on the bench

- Nothing turns the shaft without `--allow-motion` **and** a typed confirmation.
- Every motion drill stays inside a band around wherever the shaft starts,
  `--range-revs` wide (2 revolutions by default), fixed once at the start.
- Drills that interrupt a move wait on *observed progress*, not a fixed delay,
  so they work whatever speed your motor runs at — and say so plainly if the
  move finished too quickly to interrupt, rather than passing without having
  tested anything.
- The shaft is returned to where it started and left passive at the end,
  including after a failure or Ctrl-C.

### Running a subset

```
python -m psct_motors.cli demo --category fault           # just the fault drills
python -m psct_motors.cli demo --only identity,fault-comms
python -m psct_motors.cli demo --no-operator              # nothing to unplug
python -m psct_motors.cli demo --simulate --allow-motion  # no hardware at all
```

`--simulate` runs the whole thing against a fake motor, which is the way to see
what the output looks like before pointing it at hardware.

---

## When it stops taking commands

A motor that works and then quietly stops responding to position commands is
the hardest kind of fault to chase, because **the drive almost never refuses
the command**. It accepts the write, returns success, and does nothing. From
the application's side these all look identical:

- the drive is Passive, so the output is off
- an error is latched
- `V_SOLL` is 0, so it approaches the target at zero speed
- `RUN_CURRENT` is 0, so there is no torque
- something else is overwriting `P_SOLL`

Nothing in the Modbus response distinguishes them. Three tools here do.

### The bench GUI

```
python -m psct_motors.cli motor-gui --motor A
python -m psct_motors.cli motor-gui --simulate      # no hardware
```

One motor, in revolutions and counts, no calibration needed. Live position,
target, mode, `V_SOLL` and brake; a prominent error panel that turns red and
decodes `ERR_BITS`, with **Clear errors** next to it; move and jog controls;
a **STOP** that works while anything else is running; and a fault-injection
panel that arms any of the simulated faults against the live link so you can
watch the error handling do its job.

Underneath it runs the event log, recording to disk from the moment it opens.

### "Why is it not moving?"

The button in the GUI, or from the command line:

```
python -m psct_motors.cli diagnose --motor A
```

Runs every check that could independently stop the motor, in the order they
occur, and classifies each **BLOCKING / SUSPECT / OK / UNKNOWN** with a
concrete remedy:

```
Motion is blocked: Drive is passive (and 1 more)

[BLOCKING] Drive is passive
           MODE_REG = 0 (Passive). The drive output is off. Writes to P_SOLL
           succeed and are ignored, which is the single most common reason a
           motor 'stops taking position commands'.
           -> Enable Position mode. If it will not stick, something else is
              writing the register -- MacTalk still being connected is the
              usual cause.
[BLOCKING] Velocity limit is zero
           V_SOLL = 0. The motor will accept a target and approach it at zero
           speed -- that is, never move. Nothing reports an error.
```

It is read-only apart from one probe: it writes `P_SOLL` with the position the
motor is **already at**, which cannot cause motion and is the only way to find
out whether writes are landing at all. `--no-write-probe` turns even that off.

### The event log

The catch with a stall is that you notice it minutes after it started. The log
records **changes**, not samples, so a quiet motor produces a quiet file and
the interesting moment stands out:

```
python -m psct_motors.cli watch --motor A          # record until Ctrl-C
python -m psct_motors.cli show-log logs/motor-A-20260813-190000.jsonl
```

```
[19:22:14.595] INFO    mode     MODE_REG Passive (drive off) -> Position
[19:22:15.596] INFO    motion   P_SOLL 0 -> 204800
[19:22:18.598] ERROR   error    ERR_BITS set: 0x00000002: Follow error ...
[19:22:22.100] ERROR   mode     MODE_REG Position -> Passive (drive off) --
                                the drive went passive on its own. Position
                                commands will be accepted and do nothing.
[19:22:23.601] ERROR   config   V_SOLL is 0. Position commands will be
                                accepted and the motor will never move.
```

It also times every poll and flags any that ran long:

```
[19:31:02.114] WARNING timing   Poll took 4.02s (threshold 1s). A stalled-but-
                                successful transaction is what an application
                                hang usually is.
```

That line matters because a slow-but-successful transaction leaves **no error
behind at all** — there is nothing to find afterwards except the timing.

Events go to a JSONL file, line-buffered, so a recording survives the process
being killed. Leave `watch` running next to whatever you are testing and the
answer is usually already in the file, above the point where you noticed.

### One fix already made from this

`pymodbus` defaults to `retries=3`, so **one** failed read blocks for
`timeout x 4` before it reports — eight seconds at our old settings, twelve at
pymodbus's own defaults. A poll loop that hits that does not look like an
error, it looks like the application has frozen. Attempts are now capped at 1
by default (`modbus_retries` in the config), so failures are reported promptly
instead of stalling. If the previous script hung rather than erroring, this is
a strong candidate for why.

---

## Commissioning: do this before trusting anything

The defaults are a starting point, not the truth about your hardware. Work
through this in order. It is quick, and each step catches a failure that is
otherwise silent.

### 1. Confirm the word order

```
python -m psct_motors.cli detect
```

Several JVL registers hold small, non-negative configuration values — currents,
ramp times, the velocity limit. Any value below 65536 has a high word of zero,
so whichever of the two Modbus words comes back consistently zero *is* the high
word. That fixes the order from the structure of the data rather than from a
guess about what any value should be, and several registers vote so that one
that happens to be zero or unexpectedly large cannot mislead it.

If a motor contradicts the config, fix `word_order` — until you do, every
position from that motor is wrong, and connecting refuses to proceed. An
*inconclusive* probe is reported and allowed through: a diagnostic that cannot
reach a verdict must not become the reason you cannot connect.

> An earlier version of this compared register 1 against a "looks like a
> firmware version" range. On the real pSCT motor register 1 reads 540777,
> which needs 20 bits and so is not the 16-bit version field JVL's overview
> describes — probably that register is something else on this firmware. The
> check then declared it could not determine the word order on a motor whose
> word order was provably correct.

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

### A fault stops every axis, not just the faulty one

When one motor faults or times out mid-move, the other two are halted as well
before the error is reported. Two actuators continuing to a target the third
will never reach is precisely how the plate gets racked about its ball joints.

Halting means commanding each motor to hold the position it reports, so the
drives stay enabled and keep holding. There is one case this cannot cover
honestly: if an *encoder* dies while its shaft still turns, the reported
position is stale and the halt commands that axis back to it. Nothing readable
over Modbus distinguishes that from a genuinely seized axis, so the demo says
so explicitly rather than pretending otherwise, and a frozen position with no
error bits is worth a physical look before you command it again.

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

**Contradicted by the hardware** — these read values on the real pSCT motor
that do not match what the register name implies, so treat the names as wrong
until MacTalk says otherwise:

| register | reads | why that is odd |
|---|---|---|
| 1 `PROG_VERSION` | 540777 | needs 20 bits; not a 16-bit version field |
| 4 `P_NEW` | 0x06080000 | not position-shaped while `P_IST` is 600 |
| 25 `STATUSBITS` | 0x8A476C14 | on a *passive, stationary* motor |

Nothing in this package acts on any of the three. Register 25 carries no bit
names at all now: a guessed layout decoded that value as "Decelerating, Motion
running" for a motor that was doing nothing, and a confident wrong answer is
worse than no answer.

**Needs verification before you rely on it:**

- `P_NEW` (4), `FLWERR` (20), `STATUSBITS` (25), `WARN_BITS` (36), `P_HOME` (38).
- The **individual bit meanings** in `ERROR_BITS`. The register itself is
  right — `ERR_BITS` of 0 means healthy on the real motor — but the bit-to-text
  mapping is unchecked, so every decoded error is printed with raw hex first
  and an explicit `[bit names UNVERIFIED]` marker.
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
  faults.py       fault injection over a live link, for testing error handling
  eventlog.py     timestamped change recorder, for explaining a later hang
  diagnostics.py  "why is it not moving" -- ordered checks with remedies
  demo.py         single-motor exerciser and fault drills
  gui.py          three-motor focal-plane application
  single_gui.py   one-motor bench GUI: errors, fault injection, live log
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

233 tests, no hardware needed. The GUI tests skip automatically without a
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
- A fault detected mid-move halts **all three** axes, not just the faulting one.
- Injected fault values are encoded in the motor's own word order, so a drill
  exercises the error bit it claims to.
- The demo fails, rather than passing, when the software or config is actually
  wrong -- e.g. a broken word order makes the `identity` drill FAIL.
- Word-order detection is pinned against the real motor's register values,
  including the register-1 reading that broke the previous heuristic.
- An inconclusive word-order probe does not block connecting.
- `reconnect()` builds a fresh client, so a link that comes back is usable.
- Every cause of a silent stall -- passive drive, zero V_SOLL/A_SOLL/
  RUN_CURRENT, latched errors, engaged brake, an overwritten target -- is set
  up in turn and the diagnosis must name it.
- The diagnosis write-probe commands only the current position, and is proven
  unable to move the shaft.
- The watcher logs changes rather than samples, and flags slow transactions.
- A recording survives without being closed, and corrupt lines are skipped.
