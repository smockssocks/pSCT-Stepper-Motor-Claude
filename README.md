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
- **Saved positions**: pick one ("Default", "Window open", anything you name)
  and press **Go to**, or **Save current as...** to remember where the plane
  is now. See [Saved positions](#saved-positions).
- A **position gauge** down the right: + towards M1 above, − towards M2 below,
  travel limits and ends of travel marked, target shown while moving. Under
  it, the distance to M1 and to M2 once those are entered (*Distances to
  M1 / M2...*; nothing is shown until they are known).
- **Where 0 is** is set in *Tools → Motion settings* → *Show positions from*:
  **motor zero**, **top stop = 0** or **bottom stop = 0**. Everything in the
  window then reads from there: the focus readout, the gauge, the actuator
  positions, **Go to**, previews and confirmations, saved positions, the
  position log, the plane picture and the move lines in the log. With the top
  stop as 0, sitting on the top stop reads 0 everywhere and 10 mm below it reads
  −10; typing −10 in Go to goes there. With the bottom stop as 0, typing 10
  goes 10 mm above it. A stop has to be known (found by Find hard stop, or
  entered in Motion settings) to be used. Only what is shown and typed changes:
  the motors, the limits, saved positions and the record on disk stay in the
  motors' own zero, so changing it never moves anything.
- Per-actuator position, mode, brake, load and **supply voltage**, with
  Release/Engage for each brake. The brake says **ENGAGED** (blue) or
  **DISENGAGED** (amber), deliberately not green/red: a disengaged brake is
  not "good", it means the camera is hanging on the drives. Supply is in volts,
  using the scale measured against MacTalk (1804 raw = 48.0 V). Jogging a
  single actuator is not on the main window; it is in *Motion → Focal plane:
  tilt and jog*.
- A timestamped log of everything the application did.

Behind the menus, so the main window stays about the job:

| where | what |
|---|---|
| **Motion → Focal plane: tilt and jog** | password protected. One window with a live drawing of the plate on its three actuators, the two tilt angles with fine adjust buttons and Level, and a jog for each actuator |
| **Motion → Save current position / Saved positions** | name the current position, and see, rename, delete or go to the saved ones |
| **Motion → Go back to the previous position** | return to where the focal plane was before the last move — an ordinary checked move, with confirmation |
| **View → Position log** | every move, newest first: where the plane was, where it was sent, where it ended up; select a line and go back to it |
| **View → Load and torque** | how hard each motor is working, big enough to read across a room, with peaks, temperature and supply; how many readings per second; and the torque limit (changing it needs the password) |
| **Tools → Connection settings** | edit each motor's IP and port, use now or save |
| **Tools → Motion settings** | where 0 is (motor zero, top stop or bottom stop); focus, tilt and step limits; the ends of travel (found by Find hard stop, or typed in behind the password); set the focus limits from the ends of travel with a margin |
| **Tools → Distances from zero to M1 and M2** | the two numbers the gauge needs to show distance to a mirror instead of distance from zero |
| **Tools → Supply voltage** | only if a motor's volts disagree with MacTalk: enter what the supply is really at, and that motor uses its own reading from then on |
| **Tools → Find hard stop** | run the actuators out to the end of travel |
| **Tools → Run safety drills** | prove the guards still fire (simulated, safe any time) |

**Addresses change.** *Tools → Connection settings* edits each motor's IP and
port. "Use for this session" applies them until you close the window; "Use and
save" writes them to the configuration file. Either way the connection is
rebuilt, because a motor object holds the address it was created with — editing
only the label would change nothing.

### Checking the over-torque protection

The stall limit ships at 45%, which is a guess from one motor's idle reading.
Change it in *View → Load and torque* (**Change...**, then the password): the
amber warning level and the level at which a move is stopped, for all three
motors, saved to the configuration. Better still, measure it first:

```
python -m psct_motors.cli torque-profile --mm 0.5              # safe anywhere
python -m psct_motors.cli torque-profile --to-stop             # drives to the end
```

It reports what torque reads at rest, moving freely and pressed against the
stop, and recommends a threshold from the gap between them — or says plainly
that there is no gap, in which case torque alone cannot find the stop and the
"commanded a step and barely moved" check is what does.

### Keeping the three together, settling, and the big-error stop

Every coordinated move is watched while it runs, and checked when it ends.

- **Staying in step.** The three speeds are scaled so they arrive together,
  but a motor under more load can fall behind, and the plate tilts on the way.
  Each actuator's progress along its own move is compared every tenth of a
  second. One more than `sync_pause_mm` (0.05 mm) ahead of the slowest is held
  where it is until the slowest catches up, then sent on a fifth slower. If
  they get `sync_abort_mm` (0.5 mm) out of step, or the slow one has not caught
  up after `sync_max_wait_s` (5 s), the move is stopped.
- **Settling.** Under load a stepper sits slightly behind its command (the
  bench motor sat 231 counts, about 1.4 µm, short). After a move each motor's
  encoder is compared with its target; one more than `settle_deadband_counts`
  (50 counts, 0.3 µm) off is sent the difference, up to `settle_max_tries`
  (3) times. The log says what it did: `Settled at the target (Top +231 -> +4
  counts, ...)`. The motor takes commands in 1/409,600 of a turn (2,048 per
  full step, about 6 nm of travel here), but it does not land that finely,
  which is why there is a deadband and a try limit rather than chasing every
  count. Single-actuator jogs settle too.
- **The big-error stop.** An actuator more than `max_position_error_mm`
  (0.1 mm) from where it is being driven, during the move or after it, means
  it has slipped or is blocked: a stepper that is more than a step or two
  (12.7 µm a full step) behind has lost its grip. All three are halted,
  holding, and the brakes are applied; the message says which motor and by
  how much.

- **Falling watch.** On every status poll, any axis the drive is not
  driving (passive, any mode other than Position, or holding with its move
  finished) is watched. If it moves more than `fall_limit_mm` (0.05 mm) on its
  own, it is falling or slipping: every brake is engaged at once, anything
  being driven is stopped, every passive drive takes hold where it now is, and
  the red bar says which motor moved and how far. It resets whenever a drive
  is driving its axis, so moves, STOP and the hard-stop search never trip it.
  It needs nobody at the controls, only the GUI (or anything polling) running.
- **Already straining.** A move or jog is refused before anything happens
  if any motor is already at or over its torque stop level while standing
  still: something is pushing against it, and forcing a move could damage the
  telescope. Nothing is commanded and the brakes are not touched. The check
  is made with the drives on and holding, just before the brakes would come
  off.

The out-of-step stop and the big-error stop both leave the drives on and
holding *and* the brakes on. All of the numbers above are in the
configuration file; `settle_enabled: false` turns settling off.

### Resting on the brakes (Disable drives)

Once the focal plane is where it should be, **Disable drives** (under the
actuator table, or *Tools*) leaves it held by the brakes alone, so the motors
make no small corrections:

1. The brakes are engaged and read back.
2. The drives are turned off.
3. For `rest_watch_s` (1.5 s) the encoders are watched. If any motor moves
   more than `rest_sink_limit_mm` (0.01 mm), the brakes are not holding: the
   drives are turned straight back on, holding where they are, and the log
   says which motor moved and how far.

If the PLC does not report the brakes engaged, the drives stay on. (With
`trust_relay_state` set to false, the GUI also asks once per session before
relying on the relay reading.)

Tick **Disable drives after each move** to do this automatically when every
move or jog finishes (for this session). The next move turns the drives back on
and releases the brakes itself, as usual. If resting fails after a move, the
move still counts as done and the log says why the drives were left on.

From the command line: `python -m psct_motors.cli disable-drives`
(`--trust-relay` to accept the relay's reading without being asked).

### Supply voltage

The drives report their supply on register 97 in their own units. On the pSCT
motor 1804 of them read exactly 48.0 V on MacTalk's display (an earlier
reading, 1794, is 47.7 V on the same scale), so that is the scale used: the
supply shows in volts with nothing to set up.

A move is refused when the supply reads below 80% of 48 V (about 38 V). A JVL
with no main supply still answers Modbus from its control supply: it accepts a
target and quietly does nothing, which presents as the software being broken.
The check names the supply instead.

It is never compared against the drive's "Acceptance Voltage" register (139),
which on the bench motor reads 2054 and is not known to be on the same scale.

If a motor ever disagrees with MacTalk or a meter, record its own reading: in
the GUI, *Tools → Supply voltage* (or click a supply reading), enter the voltage
the supply is really at, and press **Record**; or from the command line:

```
python -m psct_motors.cli supply --volts 48                    # all three
python -m psct_motors.cli --bench Top supply --motor Top --volts 48
```

That motor then uses its own reading instead of the measured scale. On the
bench, pass `--motor`: a reading from a simulated stand-in measures nothing.

### How fast the readouts update

Position, torque, mode and following error refresh every `poll_interval_s`
(0.1 s) while anything is moving and every `idle_poll_interval_s` (0.2 s)
when nothing is. The three motors are read side by side, each over its own
connection, so a poll takes about as long as reading one motor. *View → Load
and torque* has a **Readings per second** setting (2, 5 or 10 while idle),
which is saved. A configuration still carrying the old 0.15 s / 0.5 s
defaults gets the new ones automatically. Bus voltage and temperature change over minutes, so they are
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

### Saved positions

Some positions get used again and again: the focus everyone agreed on, the
focus with the new camera window fitted, a survey position. Save each under a
name and anyone can go back to it without knowing the numbers.

- **Save current as...** (in the Focus box, or *Motion → Save current
  position*): pick "Default" or "Window open" from the list or type any name,
  add a note if it helps, and Save. Saving under a name that already exists
  asks before replacing it.
- **Go to**: choose a name in the Focus box and press Go to. It is an ordinary
  move: checked against the limits and step size, previewed, and confirmed.
- **All saved...** (or *Motion → Saved positions*): every saved position with
  its focus, tip, tilt, when it was saved and its note. Select one to see
  exactly where it will send each actuator. Rename, Delete and Go to selected
  are there too.

They are kept in `config/saved_positions.json`, beside the configuration. Each
one is stored as the motors' own encoder counts as well as focus/tip/tilt, so
setting a new zero later does not move a saved position: it still goes to the
same physical place, and the list says the numbers are now measured from the
new zero.

```
python -m psct_motors.cli positions                       # list
python -m psct_motors.cli positions save "Window open" --note "new window"
python -m psct_motors.cli positions go "Window open"
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
the load bars. The title bar says **[BENCH — only Top is real]** and the log
says it too.

The two stand-ins are **copies of the real motor**: they sit wherever it is
and go wherever it is sent, so the plane stays flat and moves only in focus,
and they do not sag with the drives off. Because of that, tip, tilt and
jogging East or West on their own are refused in bench mode; jog Top and the
copies follow. To get independent simulated axes back, set
`"bench_stand_ins_copy_real": false` in the configuration.

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
  no drive reports an error, the supply has not failed, every drive is on and reads back as holding.
  Only then is the brake released (just that actuator's, if they are
  separate). If any check fails, nothing is released and the log says which.
- **Engage all brakes**, then try **EMERGENCY**. It engages the brakes but
  **keeps the drives on**, and the log says why (see below).
- `python -m psct_motors.cli plc` prints every relay and input from the command
  line. It only reads.

**If the PLC drops out now and then.** The GUI gives a status read 1.5 s, and
tries once more straight away if the PLC misses one; only two misses in a row
show the brakes as NOT READABLE (for 2 s, then it tries again). The log says
`Lost the PLC: <reason>` and `PLC answering again.`, so the reason is on
record. To find the cause, close the GUI and run:

```
python -m psct_motors.cli plc --watch 600        # ten minutes, reads only
```

It reads the PLC every 0.4 s (what the GUI does), times every reply, prints
every failure with its reason, and ends with a summary that says what the
pattern points to. Run `ping -t <PLC address>` in a second window at the same
time. Then:

- **Ping drops too:** the network or the PLC's power. Check the cable and
  switch port, turn off power saving on the laptop's Ethernet adapter
  (Device Manager → adapter → Power Management, and "Energy Efficient
  Ethernet" under Advanced), and check the PLC's supply. If it happens when
  relays switch, a supply that sags when the coils pull in will reboot it.
- **Ping is clean but reads time out or are refused:** the PLC is busy or
  out of connections. Close any browser tab showing the PLC's own page (it
  refreshes itself constantly) and make sure only one copy of the GUI is
  running.
- **Two IP addresses on the laptop's adapter** (one for the motors, one for
  the PLC) works, but moving the PLC onto the motors' network (for example
  192.168.0.60) is simpler and removes one thing that can go wrong.

**The PLC's report is the brake state.** The brakes are fail-safe (power
releases them, no power clamps them), and the PLC reports its relays
correctly, so a relay reading "off" is taken as a clamped brake
(`trust_relay_state`, on by default). EMERGENCY and Disable drives turn the
drives off once the PLC reports the brakes engaged; if the PLC cannot be read,
the drives stay on and holding. Disable drives also watches the encoders after
the drives go off and turns them straight back on if anything moves, so a brake
that does not actually hold is caught. If a switch on the brake is ever wired to
a PLC input, set it under *Brake feedback inputs* and that reading is used
instead.

**If the PLC drops a reply.** The display reads the PLC in the background and
keeps the last good reading for up to 3 s, so a slow reply does not show up as
the brakes going unreadable. Anything that acts on the brakes (a move, a
release, EMERGENCY) reads the PLC there and then. The log only says so if the
PLC has not answered for 3 s, and again when it is back.

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
