# Reference

The mechanism, the code layout, and how to run the tests.

Back to the [README](../README.md).

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
- `MODE_REG` = 2, `P_SOLL` = 3, `V_SOLL` = 5, `ERR_BITS` = 35, and 409600
  counts per revolution. (Register 10 carried over too, but as the *actual*
  position — a MacTalk dump later showed it is the projected one. See
  [what is verified](safety.md#what-is-verified-and-what-is-not).)

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
  external_brake.py  the pSCT brakes are on a separate device, not the motors
  gui.py          three-motor focal-plane application
  focus_gauge.py  the distance-from-zero indicator
  plane_view.py   live picture of the plate on its three actuators
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

312 tests, no hardware needed. The GUI tests skip automatically without a
display; to run them headlessly:

```
xvfb-run -a python3 -m unittest tests.test_gui -v
```

`python -m psct_motors.cli safety-check` is the same idea aimed at an operator
rather than at CI: it provokes each dangerous situation and reports whether the
guard fired and whether its message was useful.

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
- Position comes from the encoder, not the profile output, and a large
  following error is not reported as a completed move.
- `stop()` freezes the profile output, so a stop cannot itself cause a step.
- The brake configuration is checked against register 179, so claiming control
  of a brake no output drives is caught.
- Torque percent matches the real motor's 337/2048, a stall aborts a move and
  halts the axis, and a brief spike does not.
- `find-stop` finds the end, backs the command off, works in both directions,
  and says so honestly when there is no stop to find.
- The coordinated hard-stop search moves all three, halts them all on the first
  stop, levels the plate afterwards, and abandons the search when the axes
  drift apart -- a lagging axis is not reported as a hard stop.
- A simulated brake holds the shaft, so "the brake did not release" is a
  condition the tests can actually create; releasing one with the drives
  passive, and moving with the brake supply or the 60 V off, are each refused
  with a message naming the cause.
- Removing a guard makes its safety drill fail, so a drill that stopped
  provoking anything cannot keep reporting PASS.
- The scale reproduces the measured 0.059 mm per 10,000 counts.
- Bus voltage below the drive's acceptance threshold blocks, which is the
  documented "the motor will not move without its 60 V supply".
