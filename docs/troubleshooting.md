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

