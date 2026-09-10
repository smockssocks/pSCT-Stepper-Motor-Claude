# Commissioning

Do this before trusting any number the software reports. Every step here
exists because a value in the configuration is a guess until it is measured.

Back to the [README](../README.md).


The defaults are a starting point, not the truth about your hardware. Work
through this in order. It is quick, and each step catches a failure that is
otherwise silent.

### 1. Confirm the word order

```
python -m psct_motors.cli detect
```

Several JVL registers hold small, non-negative configuration values — currents,
ramp times, the velocity limit. Any value below 65536 has a high word of zero,
so whichever of the two Modbus words comes back consistently zero *is* the high
word. That fixes the order from the structure of the data rather than from a
guess about what any value should be, and several registers vote so that one
that happens to be zero or unexpectedly large cannot mislead it.

If a motor contradicts the config, fix `word_order` — until you do, every
position from that motor is wrong, and connecting refuses to proceed. An
*inconclusive* probe is reported and allowed through: a diagnostic that cannot
reach a verdict must not become the reason you cannot connect.

> An earlier version of this compared register 1 against a "looks like a
> firmware version" range. On the real pSCT motor register 1 reads 540777,
> which needs 20 bits and so is not the 16-bit version field JVL's overview
> describes — probably that register is something else on this firmware. The
> check then declared it could not determine the word order on a motor whose
> word order was provably correct.

### 2. Check the register map

```
python -m psct_motors.cli verify-registers
```

Prints every register this software uses with a confidence marker, next to the
live value. Compare the `VERIFY` rows against the same fields in MacTalk and
update the markers in `psct_motors/registers.py` as you confirm them. See
[what is verified](safety.md#what-is-verified-and-what-is-not).

### 3. Check each actuator's direction

```
python -m psct_motors.cli check-direction --motor Top
```

Moves the actuator and asks whether the focal plane went the way the software
thinks. Answer honestly; if it is wrong, the command flips `direction` and
saves it.

**Why this matters:** with one direction sign wrong, a pure focus command
becomes a tilt, and everything still reports success. This is the single most
important step here.

Repeat for B and C.

### 4. Measure counts per millimetre

```
python -m psct_motors.cli calibrate --motor Top
```

Moves a known number of counts and asks what displacement you measured with a
dial indicator. It divides, and saves the result.

**You do not need to know the gear ratio.** That is the point of measuring:
one number absorbs the gearbox, the screw lead and any fixed scaling error.
Chasing the drivetrain down on paper and hoping the numbers are right is the
slower and less reliable path.

If you leave `counts_per_mm` unset, the software falls back to deriving it:

```
counts_per_mm = counts_per_rev * gear_ratio / screw_lead_mm
```

The default `screw_lead_mm` of 2.54 is an **inference**, not a measurement: the
published pSCT camera description gives 12.7 µm of z travel per motor step and
5.08 cm (2 in) of total travel, and 12.7 µm per 1.8° full step implies a
2.54 mm/rev lead with no reduction. Plausible, unverified, and irrelevant once
you have measured. The GUI warns on startup for any actuator still using a
derived scale.

Repeat for B and C — they are not necessarily identical.

### 5. Confirm brake control

```
python -m psct_motors.cli probe-brake --motor Top
```

Toggles the brake with the drive enabled and holding, and asks you to confirm
you heard it click. See [brakes](safety.md#brakes) for the modes.

### 6. Set the geometry

Edit `azimuth_deg` and `radius_mm` for each actuator from the camera drawing.
`azimuth_deg` is measured counter-clockwise from +x looking along −z;
`radius_mm` is the ball joint's distance from the optical axis.

Getting `radius_mm` wrong scales all your tilt angles by a constant factor, so
it is worth measuring rather than guessing.

### 7. Set the zero reference

With the focal plane at a position you have independently established:

```
python -m psct_motors.cli set-zero
```

Every later command is measured from here. The offset is stored in the config
file, not written into the motors, so MacTalk and this software never disagree
about the motor's own position.

---

