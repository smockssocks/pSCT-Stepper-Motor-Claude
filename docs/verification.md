# Verification: how to know this is alright

Work down this list at the telescope. Each step either passes or tells you
something you did not know. Nothing here is optional before the software drives
the camera unattended.

The order matters: everything above a step has to pass before the step below it
means anything.

Back to the [README](../README.md).

---

## 0. First, the thing that just bit us

EMERGENCY used to write `MODE_REG = 0` immediately. On this machine that
removed the only thing holding the camera and it sank. If you are running any
copy of this software from before that fix, **stop using the EMERGENCY button**
until you have updated.

Check you have the fix:

```
python -m psct_motors.cli --simulate safety-check --only emergency
```

Two drills must pass. The first proves EMERGENCY keeps holding when nothing
else can; the second proves it still turns the drives off when the brakes do
read back engaged.

---

## 1. Before touching the telescope

All of this runs against simulated motors and takes about two minutes.

```
python -m psct_motors.cli --simulate safety-check
python -m unittest discover -s tests
```

`safety-check` provokes thirteen dangerous situations and checks the software
refuses each one *and* says something useful. The unit tests cover the parts an
operator cannot see: kinematics, register decoding, threading, the STOP path.

If either fails, do not run against hardware. A failing drill means a guard is
missing or has stopped working.

---

## 2. One motor on the bench

Prove the software can talk to a motor and handle it going wrong, on a motor
that is not holding anything up.

```
python -m psct_motors.cli detect --motor Top          # word order
python -m psct_motors.cli verify-registers            # register map vs live
python -m psct_motors.cli demo --motor Top --allow-motion
```

The demo deliberately breaks things: pulls the register map out from under the
driver, injects a drive fault mid-move, has another client fight for the mode,
freezes the position. Watch each one get caught. It exits non-zero if anything
failed.

**What this proves:** the error handling works. **What it does not prove:** that
the motor sets the error bit you think it does. Only hardware tells you that,
which is why every decoded error prints raw hex first.

---

## 3. On the telescope, before any motion

With the motors connected and the camera hanging on them:

```
python -m psct_motors.cli status
python -m psct_motors.cli diagnose --motor Top
```

Check by eye, against the readouts:

| check | why |
|---|---|
| all three report a position | a motor that answers but reads 0 is not connected to what you think |
| bus voltage above acceptance | a JVL with no 60 V accepts targets and ignores them |
| no error bits on any motor | clear them before moving, not after |
| brake state matches reality | look at the brakes; the software cannot |
| positions agree with a ruler | `counts_per_mm` wrong by a factor is invisible in software |

That last one is the one people skip. The software cannot detect a wrong
`counts_per_mm`: the command and the readback use the same number, so an
actuator that physically travels half as far as it says still reports the right
figure. Measure it with an indicator once, against `cli calibrate`.

---

## 4. Prove the brakes, by watching them

Everything about the brakes is currently unverified, because the brake device's
protocol is unknown and `external_brake.mode` is `none`. Until that is fixed:

1. Release the brakes from the brake page, as the written procedure says.
2. Command a 0.1 mm move. It should move.
3. Engage the brakes from the brake page.
4. Command another 0.1 mm move. **It should not move, and the motors should
   report a stall or a following error.**

Step 4 is the important one. If the motors quietly move anyway, the brakes are
not doing what you think, and every interlock in this software that says
"brakes" is resting on a false premise.

Write down what you see. Then fill in `external_brake` in the config so the
software can read the state instead of assuming it, which needs one of:

* the make and model of the device behind the brake page, or
* whether it answers Modbus TCP, and on which coil, or
* the URL its own buttons POST to — a browser's network tab shows this in a
  minute.

---

## 5. Set the torque threshold from measurement, not from the default

The stall limit ships at 45%. That number came from one motor's idle reading of
337/2048, not from your machine under load. Measure it:

```
python -m psct_motors.cli torque-profile --mm 0.5
python -m psct_motors.cli torque-profile --mm 0.5 --to-stop --budget-mm 30
```

The first is safe anywhere — it moves 0.5 mm and watches. The second drives to
the end of travel, so do it once, deliberately, with somebody watching the
camera.

It prints what torque reads at rest, moving freely, and pressed against the
stop, and recommends a threshold from the gap. Three outcomes:

* **Clear separation** (stop torque well above moving torque) — take the
  recommendation, put it in the config, done.
* **They overlap** — torque cannot distinguish "at the stop" from "moving" on
  this machine. The `find-stop` search will still work, because it also checks
  whether a commanded step actually moved. Do not lower the threshold to try to
  make torque fire; that only aborts ordinary moves.
* **No torque readings at all** — set `stall_protection: false` rather than
  leaving it on and trusting it.

Then watch the load bars in the GUI during a normal move. Green during ordinary
motion, with the peak mark well short of the red line, is what "correctly set"
looks like.

---

## 6. The calibration run, watched

```
python -m psct_motors.cli find-stop --step-mm 0.2 --budget-mm 30
```

All three go out together. Somebody watches the camera; somebody watches the
load bars. Expect:

* all three positions advancing in step, the "apart by" figure staying under
  0.1 mm;
* torque flat until something meets the end;
* the moment one stops, all three stop;
* the other two backed off to match it, ending flat.

Stop it by hand if the "apart by" figure grows — that is the plate tilting, and
it is the failure mode the site says can break something.

---

## 7. Only then, unattended

Before leaving it to run:

- [ ] `safety-check` passes on the machine it will run on
- [ ] a real brake test has been done by eye (step 4)
- [ ] `stall_torque_percent` comes from `torque-profile`, not the default
- [ ] `counts_per_mm` has been measured against an indicator
- [ ] soft focus limits are inside the hard stops you actually found
- [ ] the event log is being recorded (`cli watch --motor Top --log run.jsonl`)
- [ ] somebody knows that EMERGENCY now *holds*, and that the brake page is
      what cuts power

---

## What is still unverified

Say this out loud to whoever signs off:

* **The brake device.** Not configured, not readable, not commanded. Every
  brake interlock is tested against a simulated brake only.
* **The error bit meanings.** `ERR_BITS` is confirmed as the register and 0 is
  confirmed as healthy. Which bit means what is from JVL's conventions and has
  never been checked against a real fault on these motors. Decoded text is
  always printed next to the raw hex for this reason.
* **The other two motors.** This software talks Modbus TCP. The site drives
  the motors over serial from MacTalk, so only the bench motor has ever
  answered it. Two of the three actuators in every screenshot are simulated.
* **The scale.** 169492 counts/mm comes from one measurement of 0.059 mm per
  10,000 counts. It agrees with the PDF's 0.055 mm and a 10 TPI lead, and
  disagrees with the PDF's other figure of 0.02 inch, which is a lost decimal
  point. One more measurement would settle it.
