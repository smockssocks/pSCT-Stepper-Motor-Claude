# pSCT focal-plane actuator control

Control software for the three JVL MIS232 integrated stepper motors that
position the pSCT camera's focal plane.

You give it a focus position — or a tip and tilt, if you need one. It works out
which motors move and by how much, checks the move is safe, and drives all
three together. No hand calculation, no working out which motor to move.

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
it: word order, register map, direction, scale, brakes, supply, geometry, zero.
It is eight steps and each one is a single command.

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
  **EMERGENCY halts and keeps holding.** It engages the brakes and only removes
  drive power if they read back engaged, because on this telescope the drives
  are usually the only thing holding the camera.
- A **load bar per motor**: torque as a percentage of the drive's current
  limit, with the warning and stall thresholds marked and the peak of the last
  move held. These motors report no amps; set `rated_current_a` from the data
  plate and an approximate figure appears too.
- Focus readout in mm and microns, and which way it is from zero.
- Absolute focus moves with **Preview**, **fine adjust** buttons, and
  one-click step sizes down to 1 µm.
- A **position gauge** down the right: + towards M1 above, − towards M2 below,
  travel limits and found ends of travel marked, target shown while moving.
  The chooser above it sets what the numbers are measured **from**: the zero
  set by `set-zero` (the default, and what every command is expressed in),
  or the distance to M1 or to M2. The last two need the distance from zero
  to that mirror, which nobody has yet — enter it under *Tools → Distances*
  (or the button under the gauge) once it is known; until then the gauge says
  "distance not set" rather than showing a made-up number.
- Per-actuator position, mode, brake, load and **supply voltage**, with
  Release/Engage for each brake. The brake says **ENGAGED** (blue) or
  **DISENGAGED** (amber), deliberately not green/red: a disengaged brake is
  not "good", it means the camera is hanging on the drives. Supply shows volts
  once `cli supply` has recorded the scale, and the raw register value until
  then. Jogging a single actuator is not on the main window; it is in
  *Motion → Focal plane: tilt and jog*.
- A timestamped log of everything the application did.

Behind the menus, so the main window stays about the job:

| where | what |
|---|---|
| **Motion → Focal plane: tilt and jog** | one window with a live drawing of the plate on its three actuators, the two tilt angles with fine adjust buttons and Level, and a jog for each actuator |
| **Motion → Go back to the previous position** | return to where the focal plane was before the last move — an ordinary checked move, with confirmation |
| **View → Position log** | every move, newest first: where the plane was, where it was sent, where it ended up; select a line and go back to it |
| **View → Load and torque** | how hard each motor is working, big enough to read across a room, with peaks, temperature and supply |
| **Tools → Connection settings** | edit each motor's IP and port, use now or save |
| **Tools → Motion limits** | focus, tilt and step limits; set them from the ends of travel that Find hard stop discovered |
| **Tools → Distances from zero to M1 and M2** | the two numbers the gauge needs to show distance to a mirror instead of distance from zero |
| **Tools → Find hard stop** | run the actuators out to the end of travel |
| **Tools → Run safety drills** | prove the guards still fire (simulated, safe any time) |

**Addresses change.** *Tools → Connection settings* edits each motor's IP and
port. "Use for this session" applies them until you close the window; "Use and
save" writes them to the configuration file. Either way the connection is
rebuilt, because a motor object holds the address it was created with — editing
only the label would change nothing.

### Checking the over-torque protection

The stall limit ships at 45%, which is a guess from one motor's idle reading.
Measure it on your machine instead:

```
python -m psct_motors.cli torque-profile --mm 0.5              # safe anywhere
python -m psct_motors.cli torque-profile --to-stop             # drives to the end
```

It reports what torque reads at rest, moving freely and pressed against the
stop, and recommends a threshold from the gap between them — or says plainly
that there is no gap, in which case torque alone cannot find the stop and the
"commanded a step and barely moved" check is what does.

### Telling the software what a healthy supply looks like

Run this once per motor, with the supply on and the motor behaving:

```
python -m psct_motors.cli supply --volts 48                    # all three
python -m psct_motors.cli --bench Top supply --motor Top --volts 48
```

On the bench, pass `--motor`: a reading taken from a simulated stand-in is not
a measurement of anything, and writing one into the configuration would give
the other two axes a baseline nobody measured.

It reads the drive's bus-voltage register and records that raw number beside
the voltage you measured. Two things then start working: the readouts show
actual volts instead of a raw count, and a move is refused when that register
later reads far below what was recorded — which is what a failed supply looks
like from software. A JVL with no main supply still answers Modbus from its
control supply: it accepts a target and quietly does nothing, which presents
as the software being broken.

It is compared against **its own** recorded reading, never against the drive's
"Acceptance Voltage" register. Both are in the drive's raw units, but nothing
establishes that they share a scale — and on the pSCT bench motor they read
1794 and 2054, so comparing them would refuse every move on a motor running
perfectly well at 48 V.

Until it has been run there is nothing trustworthy to compare against, so the
software says so once per session and lets moves proceed rather than blocking
on a guess.

### How fast the readouts update

Position, torque, mode and following error refresh every `poll_interval_s`
(0.15 s) while anything is moving and every `idle_poll_interval_s` (0.5 s)
when nothing is. Bus voltage and temperature change over minutes, so they are
read once every `slow_poll_every` polls and shown from the last reading in
between — polling them at the position rate would nearly double the traffic
for numbers that have not moved.

```
python -m psct_motors.cli --poll 0.1 gui       # faster, for this run only
```

Faster is not free: every poll is a set of Modbus TCP transactions per motor,
three motors at a time on the telescope. If the event log starts reporting
slow transactions, ease it back — a poll that takes longer than the interval
is not giving you fresher numbers, it is queueing.

### Where the focal plane has been

The motors keep no history at all — there is no event log or fault buffer
anywhere in the drive — so the software writes one. Every move it commands
goes into `config/positions.jsonl`, beside the configuration: when, what kind
of move (move, fine adjust, jog, find-stop, go-back), where the focal plane was
before, where it was sent, where it actually ended up, and whether it
finished. A halted move is recorded too, because its "after" is where the
plane really is.

*View → Position log* shows it, newest first. Select a line and press **Go to
selected position** (or double-click it) to return to where that move ended
up; *Motion → Go back to the previous position* undoes the last move in one
click. Each of those is an ordinary move: it is checked against the limits and
the step size, previewed, and confirmed, exactly as a typed one is. Going back
is never an unchecked path.

```
python -m psct_motors.cli history              # the same record, in text
python -m psct_motors.cli go-back              # back to before the last move
```

### Finding the end of travel

The site calibrates by running the actuators out until they stop.
*Tools → Find hard stop*, or:

```
python -m psct_motors.cli find-stop                    # all three, together
python -m psct_motors.cli find-stop --direction -      # the other way
```

All three run out **together and continuously**, at a speed matched in
millimetres per second so the plate stays flat the whole way. Torque and
progress are watched throughout; the first axis to reach its stop halts the
other two in the same instant, they are backed off to match it, and then all
three retreat half a millimetre so nothing is left resting on the stop.

There is no way to do this with one actuator. Driving one into its end stop
tilts the focal plane about the other two ball joints.

**The end it finds becomes the limit.** The soft limits ship as a guess; a hard
stop is a measurement, so the upper focus limit is set to the stop less
`safety_margin_mm` (0.5 mm). If `total_travel_mm` is configured — 50.8 mm is the
published pSCT figure — the far end follows from it, clearly marked as derived
rather than measured until you run the search downwards too.

Both ends are drawn on the gauge as solid red lines outside the dashed soft
limits, so you can see how much room is left.

A move that brings the focal plane **back inside** the limits is never blocked
for being too large. The search leaves the plate just outside the limit by
construction, and without that rule every move home is a step bigger than the
single-step limit — which stranded the plate with no way back except editing the
configuration file.

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
The global flags — `--simulate`, `--config`, `-y`, `--bench`, `--sim-speed`,
`--poll` — work on either side of the command name. `cli --help` lists
everything.

### From Python

```python
from psct_motors import FocalPlanePlatform, Orientation

with FocalPlanePlatform() as platform:
    print(platform.read_orientation().describe())
    platform.move_to_orientation(Orientation(focus_mm=2.0, tip_deg=0.1))
    platform.move_relative(d_focus_mm=0.5)
```

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

**How fast it moves is a setting**, because it has to be chosen by somebody:
the drive's own velocity units have never been measured against millimetres on
this mechanism. A simulated actuator runs at `simulated_speed_mm_per_s` (2 mm/s)
at full velocity, which is slow enough to watch the gauge and the load bars
move. Change it in the configuration, or for one run:

```
python -m psct_motors.cli --simulate --sim-speed 0.5 gui    # slow, to watch closely
python -m psct_motors.cli --simulate --sim-speed 20 gui     # quick, to get through it
```

It only affects simulated axes. It cannot change what a real motor does.

Worth trying:

- **Motion → Focal plane: tilt and jog** — the plate on its three actuators, with
  the tilt and jog controls beside it. The dashed
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

Thirteen drills, each one setting up a situation that could damage the camera
and checking the software refuses it, with a message an operator can act on:
EMERGENCY over a camera nothing else is holding, brakes on, brake supply off,
no drive power, a brake released with nothing holding the load, focus and tilt
and step limits, an obstruction, a hard stop, and a motor unplugged mid-move. It runs against its own simulated
platform, so it is safe to run while connected to the telescope — and
*Tools → Run safety drills* does the same from the window.

What it does not prove: the brakes it uses are simulated, because the real
brake device's protocol is not known yet. It shows the interlock logic is
right, not that the wiring is.

### One motor on a bench

You do not need three motors to exercise the three-motor application. `--bench`
makes the motor you name real and stands in the other two:

```
python -m psct_motors.cli --bench Top gui
python -m psct_motors.cli --bench Top find-stop
python -m psct_motors.cli --bench Top safety-check
```

Everything above the driver then runs for real against your one motor:
kinematics, coordinated moves, the hard-stop search, the emergency interlocks,
the load bars. The two stood-in axes answer instantly and truthfully-looking,
so the title bar says **[BENCH — only Top is real]** and the log says it too.
Do not read anything into what East and West report.

The single-motor tools are still there, and need no calibration at all:

```
python -m psct_motors.cli motor-gui --motor Top       # live state and errors
python -m psct_motors.cli demo --motor Top            # drills, incl. real faults
```

Both work on a single motor with no calibration, in counts and revolutions, and
can inject faults so you can see the error handling work. `motor-gui` shows
live torque — the percentage, the raw `337 / 2048` pair for comparing against
MacTalk, and a bar with the stall threshold marked — because the bench is where
you find out what "working normally" reads before trusting that threshold on
the telescope.
See **[docs/troubleshooting.md](docs/troubleshooting.md)**.

### The brake PLC (ControlByWeb X-432)

The brakes are switched by a ControlByWeb X-432, a web PLC with 16 relays and
18 digital inputs. The software talks to it over HTTP, the same way its own web
page does: it reads `http://<PLC>/state.json` and sets a relay with
`state.json?relay1=1`. How the relays are wired to the brakes is not assumed.
You tell it which relay, and which way round.

**Three ways to bench test it**, from least to most hardware:

```
python -m psct_motors.cli --simulate --real-brakes gui   # no motors, real PLC
python -m psct_motors.cli --bench Top gui                # one real motor, real PLC
python -m psct_motors.cli gui                            # all three, real PLC
```

`--simulate` on its own never touches the PLC; `--real-brakes` is what lets it.
With stood-in motors, the stand-ins obey the PLC: while it says the brakes are
on, a simulated axis will not turn, just as a real one would not. The title bar
says which parts are real.

**Setting it up:**

1. Open `http://<PLC IP>/state.json` in a browser. You should see `relay1` to
   `relay16` and `digitalInput1` to `digitalInput18`. Anything missing has no
   "Local I/O Number" set on the PLC and has to be given one there first.
2. In the GUI, *Tools → Brake controller (PLC)* (or **Brakes…** in the
   Connection box): enter the IP, and which relay drives the brakes: one relay
   for all three, or one per actuator. With a relay per actuator the brakes
   are **separate**: each row's Release/Engage switches only that brake, and a
   jog releases only the brake of the actuator it moves (a focus move still
   releases all three). Add the user and password if the PLC asks for one.
3. Press **Read the PLC** and tick **keep reading**. Every relay and input is
   shown live. Switch relays from the PLC's own web page and watch which light
   changes, and which brake clicks. This window only reads, and never switches
   anything itself.
4. Leave **relay ON releases the brakes** ticked if the brakes are fail-safe
   (a dead-man's arrangement: they clamp when they lose power, so powering
   them is what releases them). The site describes them that way. Check it
   once by hand: release from the GUI, and the brake should be free.
5. **Use and save.**

Then run the sequence you would use on the telescope, and watch the **Brakes:**
line in the Connection box:

- **Connect**, then **Enable drives**. The drives come on holding exactly where
  they are.
- **Release all brakes**. The relay switches and the line says *disengaged*.
  Releasing is refused while any drive is off, because then nothing would be
  holding the camera.
- Move or fine adjust focus. The brakes are released first if they are on.
- **Jog** one actuator (▲/▼ in *Motion → Focal plane: tilt and jog*). Before any brake comes off it checks, in order:
  no drive reports an error, the supply has not failed (once `cli supply` has
  recorded a healthy reading), every drive is on and reads back as holding.
  Only then is the brake released (just that actuator's, if they are
  separate). If any check fails, nothing is released and the log says which.
- **Engage all brakes**, then try **EMERGENCY**. It engages the brakes but
  **keeps the drives on**, and the log says why (see below).
- `python -m psct_motors.cli plc` prints every relay and input from the command
  line. It only reads.

**Relay state vs brake state.** A relay reading "off" means the PLC was told to
turn it off. It does not prove the brake clamped: reversed polarity, a blown
fuse or a loose wire all read the same. So unless a digital input reports what
the brake actually did (a switch on the brake, or a sense on its supply), the
GUI marks the brake state with **?** and says *relay state*, and EMERGENCY will
not turn the drives off, because it cannot confirm anything else is holding the
camera. If the X-432 has a spare input wired to such a signal, set it under
*Brake feedback inputs* and both of those change.

If the PLC's own logic (a task or script on the X-432) switches the brake
relay, the software notices: a relay that does not end up where it was sent is
an error, not a success. Check the PLC's logic before assuming the wiring is
wrong.

---

## More detail

| | |
|---|---|
| **[docs/how-it-works.md](docs/how-it-works.md)** | how the software is put together, the decisions behind it, and what is not verified — read before explaining it to anyone |
| **[docs/verification.md](docs/verification.md)** | how to know it is alright: what to check, in what order, before it runs unattended |
| **[docs/commissioning.md](docs/commissioning.md)** | the eight steps to do before trusting any reading |
| **[docs/troubleshooting.md](docs/troubleshooting.md)** | when a motor stops taking commands: diagnose, event log, bench tools |
| **[docs/safety.md](docs/safety.md)** | what each guard is for, and what is verified against hardware and what is not |
| **[docs/reference.md](docs/reference.md)** | the mechanism, the code layout, running the tests |

---

## Still needed from the site

- **The brake wiring.** The brakes are on a ControlByWeb X-432 (see
  [The brake PLC](#the-brake-plc-controlbyweb-x-432)). The driver is written
  and tested against a stand-in, but which relay drives which brake, which way
  round, and whether any input reports the brake's real state all have to be
  found on the actual unit.
- **The other two motors.** This talks Modbus TCP over Ethernet. The site
  currently drives the motors over serial COM4/5/6 from MacTalk, so the other
  two need Ethernet modules, or this needs Modbus RTU support adding.
