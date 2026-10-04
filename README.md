# pSCT focal-plane control

Software for the three JVL MIS232 stepper motors that position the pSCT
camera's focal plane. You ask for a focus position (or a tip and tilt); it
works out each motor's move, checks it is safe, and drives all three together.
It also switches the actuator brakes through the ControlByWeb X-432 PLC.

```
python -m psct_motors.cli --simulate gui      # try it with no hardware
python -m psct_motors.cli gui                 # the real thing
```

---

## Setting up

Python 3.8+. For real hardware: `pip install pymodbus`. The simulator and
tests need nothing else. (On Debian/Ubuntu, `sudo apt install python3-tk` for
the window.)

1. `python -m psct_motors.cli init-config` writes `config/psct_motors.json`.
2. Set each motor's `ip` in that file (or later in *Setup → Motor
   connections*). The motors are 192.168.0.52 / .53 / .54 by default.
3. Work through **[docs/commissioning.md](docs/commissioning.md)**: direction,
   scale, brakes, geometry. Each step is one command.
4. `python -m psct_motors.cli status` to check the connection.

**Close MacTalk before connecting.** Two programs talking to a motor at once
fight over its mode.

---

## The window

| | |
|---|---|
| **STOP** | decelerate and hold, drives on. Always works, never asks. |
| **EMERGENCY** | STOP, brakes on, and once the PLC reports the brakes engaged, drives off. If it cannot confirm the brakes, the drives stay on and holding. |
| **Connection** | Connect, Clear errors, motor addresses, and the brake status line. |
| **Focus** | the current focus; **Go to** a focus (Preview / Move); **Fine adjust** by a step; **Saved position** (Go to, Save current as..., All saved...). |
| **Actuators** | each motor's position, mode, brake, load and supply voltage, with Release/Engage per brake; **Enable drives**, **Disable drives**, Release/Engage all brakes, and *Disable drives after each move*. |
| **Gauge** | where the plate is between the ends of travel, the target while moving, and the distance to M1/M2 once those are entered. |
| **Log** | everything that happened, with times. |

The menus hold everything else, each thing in one place:

| menu | |
|---|---|
| **Motion** | *Focal plane: tilt and jog* (password) · *Go back to the previous position* · *Copy current position into Go to* · *Find hard stop* · *Motion settings* |
| **View** | *Position log* · *Load and torque* (torque limit behind the password; readings per second) |
| **Setup** | *Motor connections* · *Brake controller (PLC)* · *Supply voltage* · *Distances to M1 and M2* |

Changing tilt, the torque limit or the ends of travel needs the password (ask
the team).

---

## Everyday use

**Moving.** Type a focus in **Go to**, press **Move**, check the confirmation,
and confirm. The brakes are released first and every move is checked against
the limits. For small steps use **Fine adjust**; the buttons set the step from
1 µm to 0.5 mm.

**Where 0 is.** *Motion → Motion settings → Show positions from*: **motor
zero**, **top stop = 0** or **bottom stop = 0**. Every number in the window
(readout, gauge, actuators, Go to, saved positions, log) then reads from
there. With the top stop as 0, −10 is 10 mm below it. Changing this never moves
anything.

**Saved positions.** **Save current as...** remembers where the plane is under
a name ("Default", "Window open", anything). Pick one and press **Go to** to
return; **All saved...** lists, renames and deletes them. They are stored as
the motors' own counts in `config/saved_positions.json`.

**Position log.** *View → Position log* lists every move: where the plane was,
where it was sent, where it ended up. Select a line to go back to it, or use
*Motion → Go back to the previous position*. Stored in `config/positions.jsonl`.

**Parking on the brakes.** **Disable drives** engages the brakes, turns the
drives off so the motors make no small corrections, then watches the motors
for 1.5 s. If anything moves, the brakes are not holding and the drives come
straight back on. Tick *Disable drives after each move* to do this after
every move.

**Tilt and single actuators.** *Motion → Focal plane: tilt and jog* (password):
a live picture of the plate, tip/tilt with fine adjust and **Level**, and a
jog for each actuator. A jog moves one actuator and tilts the plate.

**Ends of travel.** *Motion → Find hard stop* runs all three out together until
they stop, then backs off. The end it finds becomes the limit.

---

## Limits and safety

**Limits.** One set, used for the focus and every actuator:

- where an end of travel is known, the limit is the stop less the margin
  (*Stay inside the ends of travel by*, 0.5 mm, in Motion settings);
- where it is not known yet, the *Lowest / Highest position* in Motion settings;
- plus the **max single step** (10 mm) and the tilt limits.

Asking for a spot between the limit and the stop offers the closest allowed
position instead.

**Guards** (all settings in the configuration file):

| | what happens |
|---|---|
| Torque over the limit (45%) | the move stops; the load bar shows amber above 30% |
| Already over the limit before moving | the move is refused, nothing touched |
| Motors out of step during a move | the leader waits for the slowest (0.05 mm); 0.5 mm apart or 5 s waiting, everything stops and the brakes go on |
| A motor 0.1 mm from where it is driven | everything stops and the brakes go on |
| A motor nobody is driving moves 0.05 mm | **falling**: all brakes on at once, drives take hold, alarm on the red bar |
| Supply below 80% of 48 V | the move is refused |
| Brake PLC not set up or not answering | in the window, moves need the password and an "are you sure the brakes are released?" |
| Lost contact with a motor mid-move | the other two stop |

After a move, each motor is nudged onto its target if it is more than 50
counts (0.3 µm) off. Details: **[docs/safety.md](docs/safety.md)**.

---

## The brake PLC (ControlByWeb X-432)

Set it up in *Setup → Brake controller (PLC)*: the PLC's IP, and which relay
drives which brake. The default is **separate brakes on relays 1, 2 and 3**
(Top, East, West). Power releases the brakes (they clamp when unpowered), and
the PLC's relay reading is taken as the brake state. **Read the PLC** with
*keep reading* shows every relay and input live.

To test it on the bench:

```
python -m psct_motors.cli --simulate --real-brakes gui   # no motors, real PLC
python -m psct_motors.cli --bench Top gui                # one real motor, real PLC
```

If the PLC drops out now and then, see
[docs/troubleshooting.md](docs/troubleshooting.md#the-plc-drops-out-now-and-then)
and run `python -m psct_motors.cli plc --watch 600`.

---

## Simulation and the bench

`--simulate` stands in three motors that behave like the real ones, brakes and
end stops included; it never touches hardware. `--sim-speed 20` speeds it up.

`--bench Top` makes Top real and the other two **copies** of it, so the plate
stays flat and moves only in focus (tip, tilt and jogging the copies are
refused). For one motor on its own, `motor-gui --motor Top` and
`demo --motor Top` work in counts with no calibration.

---

## Command line

```
python -m psct_motors.cli status                 # where everything is
python -m psct_motors.cli move --focus 2         # (also --tip, --tilt)
python -m psct_motors.cli move-rel --dfocus 0.5
python -m psct_motors.cli preview --focus 2      # touches nothing
python -m psct_motors.cli stop
python -m psct_motors.cli positions [save|go|delete] NAME
python -m psct_motors.cli history | go-back
python -m psct_motors.cli find-stop [--direction -]
python -m psct_motors.cli enable-drives | disable-drives
python -m psct_motors.cli plc [--watch SECONDS]  # read the brake PLC
python -m psct_motors.cli safety-check           # simulated safety drills
```

`-y` skips confirmations; `--help` lists everything. The command line works in
motor zero.

From Python:

```python
from psct_motors import FocalPlanePlatform, Orientation

with FocalPlanePlatform() as platform:
    platform.move_to_orientation(Orientation(focus_mm=2.0))
```

---

## More detail

| | |
|---|---|
| [docs/commissioning.md](docs/commissioning.md) | steps before trusting any reading |
| [docs/safety.md](docs/safety.md) | every guard, how it works, what is verified on hardware |
| [docs/troubleshooting.md](docs/troubleshooting.md) | a motor not taking commands, the PLC dropping out, supply voltage, update rate |
| [docs/how-it-works.md](docs/how-it-works.md) | how the software is put together, and why |
| [docs/verification.md](docs/verification.md) | what to check before it runs unattended |
| [docs/reference.md](docs/reference.md) | the mechanism, code layout, running the tests |

**Still to do on site:** confirm the brake wiring on the real X-432 (relay per
brake, polarity), and connect the other two motors (Modbus TCP over Ethernet;
the site currently drives them over serial from MacTalk).
