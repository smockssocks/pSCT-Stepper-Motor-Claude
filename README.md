# pSCT focal-plane actuator control

Control software for the three JVL MIS232 integrated stepper motors that
position the pSCT camera's focal plane.

You give it a focus position — or a tip and tilt, if you need one. It works out
which motors move and by how much, checks the move is safe, and drives all
three together. No hand calculation, no working out which motor to nudge.

```
python -m psct_motors.cli --simulate gui      # try it, no hardware needed
```

Three things below: **[set it up](#set-it-up)**, **[use it](#use-it)**,
**[run the simulation](#run-the-simulation)**. Everything else is in
[docs/](#more-detail).

---

## Set it up

Python 3.8 or newer. One dependency, and only for real hardware:

```
pip install pymodbus
```

The simulator, the kinematics and the tests need nothing beyond the standard
library. Tkinter comes with Python on Windows and macOS; on Debian/Ubuntu it is
`sudo apt install python3-tk`.

Then write a configuration and edit it:

```
python -m psct_motors.cli init-config          # writes config/psct_motors.json
```

Four things in that file have to be right before any number the software
reports means anything:

| setting | what it is | how to get it |
|---|---|---|
| `ip`, `port` | each motor's address | from the site network setup |
| `counts_per_mm` | motor counts per mm of focal-plane travel | `cli calibrate` |
| `direction` | which way + counts move the camera | `cli check-direction` |
| `radius_mm`, `azimuth_deg` | where each ball joint sits | camera drawings |

The defaults are the site's measured scale (169492 counts/mm, from +10,000
counts moving the camera 0.059 mm) and the actuator layout from the motion
procedure — Top, East and West at 120°. They are a starting point, not a
substitute for measuring.

Work through **[docs/commissioning.md](docs/commissioning.md)** before trusting
it: word order, register map, direction, scale, brakes, geometry, zero. It is
seven steps and each one is a single command.

Check the connection:

```
python -m psct_motors.cli status
```

---

## Use it

### The window

```
python -m psct_motors.cli gui
```

It is built around **focus**, because that is what this mechanism is: the
site's procedure motorises only the optical axis, and X and Z are manual screw
drives.

- **STOP** across the top, always live. Neither STOP nor EMERGENCY asks for
  confirmation — they act, then say what they did on the red bar and in the log.
- Focus readout in mm and microns, and which way it is from zero.
- Absolute focus moves with **Preview**, nudge buttons, and one-click step
  sizes down to 1 µm.
- A **distance-from-zero gauge** down the right: 0 in the middle, + towards M1
  above, − towards M2 below, travel limits marked, target shown while moving.
- Per-actuator position, mode, brake lamp and jog.
- A timestamped log of everything the application did.

Behind the menus, so the main window stays about the job:

| where | what |
|---|---|
| **Motion → Tip and tilt** | the two tilt angles, with nudges and a Level button |
| **View → Focal plane picture** | live drawing of the plate on its three actuators |
| **Tools → Connection settings** | edit each motor's IP and port, use now or save |
| **Tools → Find hard stop** | run the actuators out to the end of travel |
| **Tools → Run safety drills** | prove the guards still fire (simulated, safe any time) |

**Addresses change.** *Tools → Connection settings* edits each motor's IP and
port. "Use for this session" applies them until you close the window; "Use and
save" writes them to the configuration file. Either way the connection is
rebuilt, because a motor object holds the address it was created with — editing
only the label would change nothing.

### Finding the end of travel

The site calibrates by running the actuators out until they stop.
*Tools → Find hard stop*, or:

```
python -m psct_motors.cli find-stop                    # all three, together
python -m psct_motors.cli find-stop --direction -      # the other way
```

All three go out **together**, in small steps, with each motor's torque watched
after every step. The first axis to reach its stop halts the other two in the
same poll, and they are then backed off to match it so the plate ends flat.
Running a single actuator into its stop tilts the focal plane about the other
two ball joints, so it takes an explicit `--motor Top` and warns before it does
anything.

### Command line

```
python -m psct_motors.cli status
python -m psct_motors.cli preview --focus 2 --tip 0.1         # touches nothing
python -m psct_motors.cli move --focus 2 --tip 0.1 --tilt 0
python -m psct_motors.cli move --focus 2 --tilt-magnitude 0.2 --azimuth 45
python -m psct_motors.cli move-rel --dfocus 0.5
python -m psct_motors.cli stop
python -m psct_motors.cli brake status
```

`-y` skips confirmations, for scripts. `status --json` is machine-readable.
`--simulate`, `--config` and `-y` work on either side of the command name.
`cli --help` lists everything.

### From Python

```python
from psct_motors import FocalPlanePlatform, Orientation

with FocalPlanePlatform() as platform:
    print(platform.read_orientation().describe())
    platform.move_to_orientation(Orientation(focus_mm=2.0, tip_deg=0.1))
    platform.move_relative(d_focus_mm=0.5)
```

### From LabVIEW

Run the bridge and talk to it over TCP — one JSON object per line, one back:

```
python -m psct_motors.cli server --simulate
```

LabVIEW's TCP primitives do not care which Python is installed or whether it is
32- or 64-bit, which is the usual reason a LabVIEW/Python integration refuses to
load. Details, and a `lv_*` function API for LabVIEW's native Python node, in
**[labview/README.md](labview/README.md)**.

---

## Run the simulation

Every command takes `--simulate`, which stands in three dummy motors that
behave like the real ones: same register map, same 231-count following error,
same end stops, brakes that actually hold the shaft, and a load that falls if
nothing is holding it.

```
python -m psct_motors.cli --simulate gui
```

Nothing can touch hardware with that flag set, so it is the place to learn the
window, rehearse a procedure, or show someone what a fault looks like.

Worth trying:

- **View → Focal plane picture** — the plate on its three actuators. The dashed
  triangle is the zero plane, the solid one is where the focal plane is now,
  and the orange posts are each actuator's extension. Vertical travel is
  exaggerated by the labelled factor: the plate is about a metre across and
  moves millimetres, so a true 1:1 drawing would be a flat line.
- **Tools → Find hard stop** — the simulated actuators have end stops a little
  past their soft limits and torque climbs against them, so the calibration
  can be rehearsed exactly as it will be run.
- **The brakes** — they start engaged, as spring-applied brakes do. Try a move
  and watch the software release them first; engage them by hand and watch a
  move be refused.

### Proving the guards still work

```
python -m psct_motors.cli safety-check
```

Eleven drills, each one setting up a situation that could damage the camera and
checking the software refuses it, with a message an operator can act on:
brakes on, brake supply off, no drive power, a brake released with nothing
holding the load, focus and tilt and step limits, an obstruction, a
hard stop, and a motor unplugged mid-move. It runs against its own simulated
platform, so it is safe to run while connected to the telescope — and
*Tools → Run safety drills* does the same from the window.

What it does not prove: the brakes it uses are simulated, because the real
brake device's protocol is not known yet. It shows the interlock logic is
right, not that the wiring is.

### One motor on a bench

```
python -m psct_motors.cli motor-gui --motor Top       # live state and errors
python -m psct_motors.cli demo --motor Top            # drills, incl. real faults
```

Both work on a single motor with no calibration, in counts and revolutions, and
can inject faults so you can see the error handling work.
See **[docs/troubleshooting.md](docs/troubleshooting.md)**.

---

## More detail

| | |
|---|---|
| **[docs/commissioning.md](docs/commissioning.md)** | the seven steps to do before trusting any reading |
| **[docs/troubleshooting.md](docs/troubleshooting.md)** | when a motor stops taking commands: diagnose, event log, bench tools |
| **[docs/safety.md](docs/safety.md)** | what each guard is for, and what is verified against hardware and what is not |
| **[docs/reference.md](docs/reference.md)** | the mechanism, the code layout, running the tests |
| **[labview/README.md](labview/README.md)** | the TCP bridge and the LabVIEW function API |

---

## Still needed from the site

- **The brake device.** The brakes are switched by a separate box with its own
  web page, not by the motors — which is why the motor's Brake Output register
  reads 0. To drive them from here, one of: its make and model, whether it
  answers Modbus TCP and on which coil, or the URL its own buttons POST to (a
  browser's network tab shows this in a minute). Until then the GUI says the
  brakes are not under software control rather than showing a state it cannot
  read, and the release/engage buttons explain what is missing.
- **The other two motors.** This talks Modbus TCP over Ethernet. The site
  currently drives the motors over serial COM4/5/6 from MacTalk, so the other
  two need Ethernet modules, or this needs Modbus RTU support adding.
