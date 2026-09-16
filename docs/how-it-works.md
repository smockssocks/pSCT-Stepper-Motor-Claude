# How it works

A briefing, for explaining and defending this software to people who did not
write it either.

Read it once before the demonstration. The point is not to memorise answers but
to know where the real seams are, so that a question you cannot answer is one
you can say something honest about.

Back to the [README](../README.md).

---

## What the machine is

The camera's inner structure hangs off the outer structure on **three ball pin
joints**. Behind each joint a motor turns a drive screw that pushes or pulls a
flange, and a rail constrains that flange to move only along the optical axis.

So each actuator contributes exactly one degree of freedom, and three of them
determine the plane completely: **focus** (all three together), **tip** and
**tilt** (rotations about the two in-plane axes).

The site motorises only the optical axis. X and Z are manual screw drives. That
is why the window is built around focus and the tilts are behind a menu.

---

## The shape of the code

Seven layers, each one only allowed to know about the one below it. About 13,500
lines, with 4,400 of tests.

| file | what it knows |
|---|---|
| `transport.py` | Modbus TCP wire. Nothing about motors. |
| `registers.py` | Which JVL register is which, how they are packed |
| `jvl_motor.py` | One motor: position, mode, brake, errors, torque |
| `kinematics.py` | Three heights ↔ focus/tip/tilt. Pure geometry, no I/O |
| `platform.py` | Three motors as one machine: limits, interlocks, sequencing |
| `gui.py`, `single_gui.py`, `cli.py` | What a person sees |
| `simulator.py` | A fake motor, substituted at the `transport.py` boundary |

The important consequence: **nothing above `jvl_motor.py` knows about Modbus**,
and nothing above `kinematics.py` knows about screws. If somebody asks "how hard
would it be to add a fourth actuator", the answer is that `kinematics.py` would
need new maths and nothing else would change.

### Three facts about talking to these motors

These were inherited from the working script and confirmed against hardware:

1. **JVL register N is Modbus address 2N.** Registers are doubled.
2. **Every access is two 16-bit words**, even for registers that are natively
   16-bit. A one-word write to `MODE_REG` is rejected.
3. **Low word first.** The software detects this at connect and refuses to
   continue if it disagrees with the configuration.

If you remember one thing from this section, remember the doubling. It is the
first question anybody who has driven Modbus before will ask.

### The geometry

A plane through three points has a **closed-form solution**, so converting
between actuator heights and (focus, tip, tilt) is exact and reversible — no
iteration, no fitting. That matters because otherwise every closed-loop
adjustment would drift: the orientation you command and the orientation you read
back would mean slightly different things.

**Lever arms are the surprise.** With the actuators on a 500 mm radius, one
degree of tilt costs about 15 mm of actuator span — nearly a third of the total
50.8 mm travel. `cli preview` shows the cost before committing to a move.

---

## The decisions somebody will ask about

### Why not LabVIEW?

LabVIEW could not be made to talk to the motors. Python + pymodbus could. There
was a bridge so LabVIEW could drive this over TCP; it has been removed because
nobody needed it.

### Why simulate at all?

Because there is one motor on a bench and three on the telescope, and because
several of the things worth testing are things you cannot do to a real camera —
turn the supply off mid-move, unplug a motor, drop the load.

The simulator substitutes at the **transport** boundary, so everything above it
is the real code: the same register doubling, the same word order, the same
32-bit packing. Only the wire is fake. That is why a bug found in simulation is
usually a real bug.

`--bench Top` makes one motor real and the other two simulated, which is how the
three-axis code gets exercised against actual hardware.

### Why does EMERGENCY not cut the power?

**This is the most important question, and the most interesting answer.**

It used to. It wrote `MODE_REG = 0`, which removes drive power. On this machine
that was the worst thing it could do: the camera hangs on three screws, the
brakes are on a separate device this software cannot command, so cutting drive
power removed the only thing holding it. It sank, back-driving the screws, and
the encoder counted down until it ran out of travel.

It now goes in the order **hold first, let go only once something else has taken
over**:

1. Stop every axis and keep it powered and holding. Unconditional, and first.
2. Engage the brakes, if anything here can engage them.
3. Read them back — and passivate *only* if every one reads engaged.

When the brakes cannot be confirmed, the drives stay on and the window says so.
A powered drive holding position is a safe state. An unpowered drive over a
falling camera is not.

### Why torque and not amps?

There is no register on these motors that reports amps. There is **Actual
Torque** (register 217) and **CL: Current Max** (212), which read 337 and 2048 —
about 16.5% — on the site's own motor. That ratio is the only measure of effort
the drives publish, so that is what is displayed, labelled "load" rather than
"current".

### Why is the hard-stop search all three at once?

Driving one actuator into its end stop tilts the focal plane about the other two
ball joints, and the site's own experience is that this can break something. All
three run out continuously at a matched speed; the first to stop halts the other
two in the same poll; they are levelled and backed off.

There is deliberately no option to do it with one.

### Why are the soft limits derived from the hard stops?

Because a soft limit typed into a configuration file is a guess, and an end stop
found by running into it is a measurement. After `find-stop`, the limit becomes
the stop less a margin.

---

## What is NOT verified

Say this first, not last. It is the part that makes the rest credible.

* **The brakes.** The brake device's protocol is unknown, so
  `external_brake.mode` is `none`: the software cannot read or command them.
  Every brake interlock has been tested against a *simulated* brake only. The
  test that settles it takes two minutes: engage the brakes by hand, command a
  0.1 mm move, and see whether the motors move anyway.
* **The error bit meanings.** `ERR_BITS` is confirmed as the register, and 0 is
  confirmed as healthy. Which bit means what comes from JVL's conventions and
  has never been checked against a real fault. That is why every decoded error
  prints the raw hex first and says `[bit names UNVERIFIED]`.
* **Two of the three motors.** Only the bench motor has ever answered this
  software. The site drives the others over serial from MacTalk.
* **The scale.** 169492 counts/mm comes from one measurement — 10,000 counts
  moving the camera 0.059 mm. It agrees with the published 0.055 mm and with a
  10 TPI lead. It disagrees with another figure in the same document, 0.02 inch,
  which is 8.6× larger and is almost certainly a lost decimal point.
* **Real speed.** Nobody has measured what the drive's velocity units come to in
  millimetres per second, which is why the *simulated* speed is a setting rather
  than a claim.

---

## Questions you are likely to get

**"How do you know it won't drive the camera into something?"**
Four independent guards, checked before any register is written: the commanded
orientation against focus and tilt limits; each actuator target against its own
travel limits; the size of the step; and, during the move, torque and following
error on every axis. `cli safety-check` provokes thirteen dangerous situations
and checks each one is refused — run it in front of them, it takes a minute.

**"What happens if a motor drops off the network mid-move?"**
The other two are halted before the error is reported. Two actuators continuing
to a target the third will never reach is how the plate gets racked about its
ball joints. That one was a real bug: the error escaped the wait loop past the
halt. The safety drills found it.

**"What if the position reading is wrong?"**
Position comes from the encoder (register 16), not the profile generator
(register 10) — an early version used 10, which always reaches its target by
construction and would have reported success on a stalled motor. The two are
shown side by side, and the difference between them is the following error.

**"Can it detect a wrong calibration?"**
No, and this is worth saying plainly. The command and the readback use the same
`counts_per_mm`, so an actuator that physically travels half as far as it says
still reports the right number. Only a dial indicator catches that.

**"What is the worst thing that could still happen?"**
The brakes not being what we think they are. Every interlock that mentions
brakes rests on an assumption nobody has tested on the real device.

---

## On this being written by a language model

You have said it was. Here is what is actually true about it, which is more
interesting than either "the AI did it" or "the AI can't".

**What it did well.** The parts with clear rules and known answers: the register
map, the packing, the closed-form geometry, the threading discipline, the error
paths, the tests. There is more test code here than most projects this size
have, and the documentation is more honest about its own gaps than most.

**What it got wrong, and how that was caught.** The EMERGENCY button removed the
only thing holding a suspended camera. That is not a subtle bug — it is the
worst thing in the program — and it survived review, tests and documentation
that *described the hazard in writing* two files away. It was caught because
somebody ran it on the telescope and watched the motors run away.

The same pattern repeats smaller: a stop button that stepped the axis by the
following error; a demo that ended by dropping a loaded axis; a hard-stop search
that would have reported a stop that was not there on a slow axis; simulated
bounds that disagreed with the configured ones.

**What that means.** The model was good at writing code and bad at knowing which
of its assumptions were load-bearing on a specific machine it could not see. The
value was not "an LLM wrote it" — it was that every failure got turned into a
check that fires: thirteen drills, each provoking a situation that could damage
the camera, plus a test that removes a guard on purpose and fails if the drill
still passes.

If somebody in that room thinks a language model could have produced this
straight off, the honest answer is that one did produce *most* of it straight
off, and the part that mattered was the eight rounds of somebody running it
against real hardware and saying "that's wrong".

**The thing to be able to demonstrate:** run `cli safety-check`, and when it
prints thirteen passes, say which of them exist because the software got it
wrong first.
