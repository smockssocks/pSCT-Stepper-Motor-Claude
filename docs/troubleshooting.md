# Troubleshooting

For the symptom that started this project: a motor that works and then stops
taking position commands. Also covers the single-motor bench tools.

Back to the [README](../README.md).


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
python -m psct_motors.cli motor-gui --motor Top
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
python -m psct_motors.cli diagnose --motor Top
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

### Supply voltage: what the software knows and what it does not

Register 97 `Bus voltage` reads in the drive's own raw units. Nothing in the
register map says what those are worth, so the scale was measured: 1804 raw
read exactly 48.0 V on MacTalk's display. Everything is a straight ratio from
that pair. If a motor ever disagrees with MacTalk or a meter, `cli supply`
records that motor's own pair.

Two consequences worth knowing before anyone asks:

- **Register 139 `Acceptance Voltage` is never compared against register 97.**
  Both are raw, but nothing establishes a shared scale, and on the pSCT bench
  motor they read **1794** and **2054**. An earlier version of this software
  treated 97 < 139 as "supply failed" and would have refused every move on a
  motor running perfectly well at 48 V. 139 is now printed in the report and
  used for nothing.
- **Without a recorded reading, a failed supply cannot be detected.** The
  software says so once per session rather than blocking, because refusing
  every move on an unproven comparison is worse than not checking.

What voltage the motor should be on is a hardware question, not one this
software can answer from the registers. The JVL MIS23x family is specified for
a nominal **12–48 VDC** main supply, so a bench motor measured at 48 V is at
the top of its documented range and a few volts either way is unremarkable —
but treat that as a data-sheet figure to confirm against the motor's own
documentation and the pSCT wiring, not as something this software has
verified. What the software does check is that the reading has not collapsed
relative to the one recorded when the motor was known to be working, which is
the failure mode that actually presents as "the software is broken": a JVL
with no main supply still answers Modbus from its control supply, accepts a
target, and does nothing.

The default alarm point is 80% of the recorded reading (`supply_low_fraction`).
At a recorded 48 V that refuses moves below about 38 V.

### What the motor itself remembers

Nothing, essentially — and that is worth knowing before you go looking.

A MacTalk dump of the pSCT motor (serial 314852, firmware 6.09.00) shows all
254 registers. There is **no error history, event log or fault buffer**.
Registers 35 `Errors` and 36 `Warnings` are instantaneous bit fields: a fault
that has cleared leaves no trace in the drive at all.

Two registers *are* latched extremes, and they are the closest thing to a
black box:

| register | what it holds |
|---|---|
| 22 `Follow Error Max` | the worst lag ever seen between the commanded profile and the encoder |
| 98 `Bus Voltage Min` | the lowest supply voltage ever seen |

Both survive a cleared error and a completed move, so after an intermittent
fault they are often the only evidence left in the motor. Both can be reset by
writing 0, which turns a value with no timestamp into one with a known
starting point.

```
python -m psct_motors.cli motor-report --motor Top
```

prints everything the motor reports, organised by what you would be looking
for, with those two called out. The GUI has the same under **Motor history**,
with a button to reset both.

Everything finer-grained has to be recorded externally — which is what the
event log below is for.

### The event log

The catch with a stall is that you notice it minutes after it started. The log
records **changes**, not samples, so a quiet motor produces a quiet file and
the interesting moment stands out:

```
python -m psct_motors.cli watch --motor Top          # record until Ctrl-C
python -m psct_motors.cli show-log logs/motor-Top-20260813-190000.jsonl
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


## One motor on the bench

You do not need all three motors to make progress. The demo exercises **one**
motor on its own — no kinematics, no platform, no calibration — and then breaks
things on purpose so you can watch the error handling work.

```
python -m psct_motors.cli demo --list                    # what it will do
python -m psct_motors.cli demo --motor Top               # read-only drills
python -m psct_motors.cli demo --motor Top --allow-motion  # include moving ones
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
hardware can tell you that, which is why every decoded error prints raw hex
first with an explicit `[bit names UNVERIFIED]` marker.

The names come from JVL's MIS23x/SMC75 user manual (LB0053, the "Err_Bits"
description of register 35, and "Warn_Bits" for register 36):

| bit | Err_Bits (35) | Warn_Bits (36) |
|---|---|---|
| 0 | General error | General warning |
| 1 | Follow error | Positive limit switch active |
| 2 | Output driver error (an output short-circuited) | Negative limit switch active |
| 3 | Position limit error | Positive limit has been active |
| 4 | Low bus voltage error | Negative limit has been active |
| 5 | Over voltage error | Low bus voltage |
| 6 | Temperature too high | Temperature above 80 C |
| 7 | Internal error | Driver overload |
| 8–10 | Encoder: lost position, reed error, communication error | — |

Bit 2 is the one independently corroborated: the manual's I/O chapter says a
short-circuited output "will show as Error Output Driver and Bit2 will be set
in Err_Bits". To confirm the rest, provoke one fault with MacTalk connected
and note which bit lights — or check the table against the register
description in your copy of LB0053. `cli status` and `motor-report` decode
both registers.

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



---

## The PLC drops out now and then

The GUI reads the PLC in the background and keeps the last good reading for up
to 3 s, so a slow or missed reply does not show as the brakes going
unreadable, and never holds up the motor readouts. Anything that acts on the
brakes (a move, a release, EMERGENCY) asks the PLC there and then. The log only
mentions it if the PLC has not answered for 3 s, and again when it is back.
To find the cause, close the GUI and run:

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

## Supply voltage

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
the GUI, *Setup → Supply voltage* (or click a supply reading), enter the voltage
the supply is really at, and press **Record**; or from the command line:

```
python -m psct_motors.cli supply --volts 48                    # all three
python -m psct_motors.cli --bench Top supply --motor Top --volts 48
```

That motor then uses its own reading instead of the measured scale. On the
bench, pass `--motor`: a reading from a simulated stand-in measures nothing.

## How fast the readouts update

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
