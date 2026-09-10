# Safety design

What each guard is for, and what it will and will not catch.

Back to the [README](../README.md).


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

**Neither asks for confirmation.** A safety control that stops to ask a
question is not a safety control. Both act immediately and then report what
they actually did — read back from the motors, not assumed — on a banner in the
red bar and in the log: where each actuator came to rest, its mode, its brake,
and any motor that did not answer.

Both are guarded on *any* motor being reachable, not all three. One motor
dropping off the network must not disarm STOP for the two still running.

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

When one motor faults, stalls, stops answering, or times out mid-move, the
other two are halted as well before the error is reported. Two actuators
continuing to a target the third will never reach is precisely how the plate
gets racked about its ball joints.

The "stops answering" case was missing until the safety drills went looking for
it: a Modbus error raised straight out of the wait loop, past the halt, so the
other two carried on. Which is why the drills exist — a guard nobody has
watched fire is a guard nobody should trust.

Halting means commanding each motor to hold the position it reports, so the
drives stay enabled and keep holding. There is one case this cannot cover
honestly: if an *encoder* dies while its shaft still turns, the reported
position is stale and the halt commands that axis back to it. Nothing readable
over Modbus distinguishes that from a genuinely seized axis, so the demo says
so explicitly rather than pretending otherwise, and a frozen position with no
error bits is worth a physical look before you command it again.

### Something resisting stops the motor

The site calibrates these actuators by running one out until it stops. That
only works safely if something notices the resistance, so every move watches
the motor's torque — Actual Torque (register 217) against CL: Current Max
(212), which reads 337/2048 ≈ 16% on a healthy pSCT motor and matches the
"less than 15%, sometimes 30%" the site's own procedure records.

A move aborts if torque passes `stall_torque_percent` (45% by default, clear of
normal operation including the more heavily loaded top motor) for several
consecutive readings. Several, not one, because torque spikes briefly on every
acceleration.

The check runs during coordinated moves as well as single-axis ones. Without
it, an obstruction was only caught by the move timeout, with the drive pushing
against it the whole time and the other two axes still travelling.

`cli find-stop` and **Tools → Find hard stop** turn that into the calibration
procedure itself: walk out in small steps, stop when torque rises *or* a step
barely moves, then back the command off so nothing is left pressed against the
end. Each step ends when the axis stops making progress rather than after a
fixed delay — a fixed delay cannot know how fast your motor is, and one that is
too short reports a hard stop that is not there.

**All three actuators go out together.** Driving one into its end stop on its
own tilts the focal plane about the other two ball joints, which the site says
can break it. The first axis to stop halts the other two in the same poll, and
they are backed off afterwards to match it — the step in which the first one
stopped left them up to a step ahead, and that difference is tilt. How far the
three have drifted apart is checked between steps against
`limits.max_hard_stop_spread_mm`, so an axis that keeps falling short ends the
search instead of tilting the plate further. Single-axis seeking is still there
for a deliberate small tilt, behind `--motor` and a warning.

One thing it cannot see: a wrong `counts_per_mm`. The command and the readback
use the same scale, so an axis that physically travels the wrong distance still
reports the right one. Only a measurement catches that, which is what
`cli calibrate` is for.

### The three axes arrive together

Velocities are scaled by distance so all three finish at the same moment.
Without that, the shortest move lands first and the plate sits at an
orientation nobody asked for until the last axis catches up, pivoting on its
joints the whole time.

### Brakes

**On the pSCT the brakes are not on the motors.** The site's procedure switches
them from a separate device with its own web page, and the motors' own Brake
Output register (179) reads 0 — no motor output drives a brake. That is why the
per-actuator brake mode defaults to `none`: showing an inferred brake state
with nothing behind it would be worse than showing none.

`external_brake` in the configuration is where that device goes, and
`psct_motors/external_brake.py` supports Modbus TCP or an HTTP endpoint. It is
unconfigured today because the procedure gives the page's address but not its
protocol, and names two different addresses on different slides. To finish it,
one of these is needed:

- the make and model of the device behind that page, or
- whether it answers Modbus TCP, and on which coil, or
- the URL its own buttons POST to — a browser's network tab shows this in a
  minute.

Until then the GUI says the brakes are not under software control and why,
rather than guessing a URL and firing writes at an unknown device that holds a
suspended camera. Releasing is interlocked either way: it refuses unless the
drives are confirmed enabled and holding.

**In simulation a stand-in device fills that gap**, so the interlocks can be
rehearsed and tested even though the real protocol is unknown. It holds the
shaft: a move commanded against an engaged simulated brake does not turn, and
torque climbs, as it would on the telescope. It is spring-applied like the real
ones — it starts engaged, and cutting its power clamps it rather than releasing
it. `cli safety-check` drives it through every refusal.

Before any move, the platform also refuses to start when:

- the drive supply is below the motor's own acceptance voltage. A JVL with no
  60 V still answers Modbus from its control supply, accepts a target, and does
  nothing — which presents as the software being broken.
- the brakes read as engaged and will not release, or are commanded to release
  and still read back engaged.
- the brakes are engaged and the drives are not holding, so releasing them
  would leave the focal plane held by nothing.

A single-axis move enables all three drives before the brakes come off, because
one switch releases all three brakes and two passive drives would be holding
nothing.

For a motor that *does* drive its own brake, three modes are available per
actuator:

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

Confirmed: `MODE_REG` (2), `P_SOLL` (3), `P_ENCODER` (16),
`P_PROJECTED` (10), `V_SOLL` (5), `ERR_BITS` (35), 409600 counts/rev, low-high word order, register doubling,
two-word access.

Documented: `PROG_VERSION` (1), `A_SOLL` (6), `RUN_CURRENT` (7),
`STANDBY_TIME` (8), `STANDBY_CURRENT` (9), `V_IST` (12).

A MacTalk register dump of the pSCT motor settled every register this
software uses, so the names in `registers.py` are now JVL's own rather than
inferences. Three corrections came out of it, all of which had been silently
wrong:

| register | I had it as | it actually is |
|---|---|---|
| 10 | the actual position | **`Projected Position`** — the profile generator's output |
| 16 | *(not used)* | **`Actual Encoder Position`** — where the shaft really is |
| 179 | *(not used)* | **`Brake Output`** — which output drives the brake; reads 0 |
| 4 | `P_NEW` | unnamed even by JVL — not a position register at all |

The first one mattered. Register 10 reaches the requested position *by
construction*, so checking arrival against it can never fail. On the dumped
motor it read 204800 — exactly the target — while the encoder read 204569, a
standing following error of 231 counts. Positions now come from register 16,
arrival requires the following error to be inside a window as well, and
`stop()` still freezes register 10 because writing the encoder reading as the
target would command a step equal to that standing error.

**Still not verified:**

- The **individual bit meanings** in `ERROR_BITS`. The register is confirmed —
  0 means healthy — but MacTalk's list does not publish the bit layout, so
  every decoded error prints raw hex first with an explicit
  `[bit names UNVERIFIED]` marker.
- The **bit layout of `Status Bits`** (25). Confirmed as the status word, but
  it reads `0x8A47xxxx` on a passive, stationary motor, so no bit names are
  claimed and the raw value is shown alone. A guessed layout previously decoded
  that as "Decelerating, Motion running" for a motor doing nothing.
- The **scale of the voltage registers**. `Bus voltage` reads 1794 while
  `Acceptance Voltage` reads 2054 on a healthy motor, so they are not on a
  common scale and the diagnostics deliberately do not compare them. Register
  98 is only ever compared against register 97 — same quantity, same scale.
- `screw_lead_mm` and `gear_ratio`. Superseded by `calibrate`.
- `radius_mm` and `azimuth_deg` — placeholders until read off the drawings.

---

