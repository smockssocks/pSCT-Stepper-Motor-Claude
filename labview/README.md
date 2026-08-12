# Driving the focal-plane actuators from LabVIEW

There are two ways in. **Use Route A.** Route B is documented because some
people prefer it and because it is what most "call Python from LabVIEW" guides
describe, but Route A is the one that will not fight you.

---

## Route A — TCP bridge (recommended)

A small Python process owns the motors and answers JSON requests on a socket.
LabVIEW talks to it with the TCP primitives that have been in LabVIEW forever.

### Why this one

The Python Node cares which Python version and bitness LabVIEW was built
against, and the error you get when they disagree is unhelpful. That mismatch
is the usual reason a LabVIEW/Python integration refuses to work. A TCP socket
has none of that coupling:

- Works with any LabVIEW version that has TCP (all of them).
- Works with any Python version.
- 32-bit vs 64-bit is irrelevant.
- The Python process can run on a different machine from LabVIEW.
- You can test the whole protocol by hand with `netcat` or the CLI before
  building a single VI, so when something misbehaves you know which side it is
  on.
- If LabVIEW crashes, the motors are still owned by a process that is still
  running and still knows where everything is.

### Start the bridge

```
python -m psct_motors.cli server --host 127.0.0.1 --port 5020
```

Add `--simulate` to run against fake motors while you build the VI. Nothing
you do can move real hardware until you take that flag off, which makes it the
right way to develop.

```
python -m psct_motors.cli server --simulate
```

### The protocol

One JSON object per line, one JSON object back per line. Send a line, read a
line terminated by `\n`.

```
--> {"command": "ping"}
<-- {"ok": true, "result": {"protocol_version": 1, "connected": true, ...}}

--> {"command": "move", "args": {"focus_mm": 25.0, "tip_deg": 0.1, "tilt_deg": 0.0}}
<-- {"ok": true, "result": {"motors": [...], "orientation": {...}}}

--> {"command": "move", "args": {"focus_mm": 999}}
<-- {"ok": false, "error": "Move refused, nothing was commanded: ..."}
```

Every reply has an `ok` boolean. Failures are always a well-formed reply, never
a dropped connection, so a LabVIEW **TCP Read** never sits waiting for an
answer that is not coming.

### Commands

| command | args | does |
|---|---|---|
| `ping` | — | protocol version, connection state, actuator names |
| `connect` | — | open the Modbus connections |
| `disconnect` | — | close them; does not stop motion |
| `status` | — | all three motors plus the orientation |
| `orientation` | — | just the orientation |
| `preview` | `focus_mm`, `tip_deg`, `tilt_deg` | actuator targets and limit check, moves nothing |
| `move` | `focus_mm`, `tip_deg`, `tilt_deg`, `wait` | absolute move |
| `move_relative` | `d_focus_mm`, `d_tip_deg`, `d_tilt_deg`, `wait` | relative move |
| `move_polar` | `focus_mm`, `total_tilt_deg`, `azimuth_deg`, `wait` | tilt as magnitude + direction |
| `jog_actuator` | `name`, `mm`, `relative`, `wait` | one actuator; commissioning only |
| `stop` | — | controlled stop, drives stay on and holding |
| `passivate` | — | brakes on, drives off |
| `brake` | `action` = `status`/`engage`/`release` | brake control |
| `clear_errors` | — | best-effort error clear |
| `set_zero` | `persist` | define the current position as the reference |

`wait` defaults to true: the reply does not come back until the move finishes.
Set it to `false` if you would rather poll `status` yourself.

### Two connections, not one

Open **two** TCP connections from LabVIEW:

1. A command connection, for moves. A `move` with `wait: true` holds this
   connection until the move finishes.
2. A status/stop connection, polled from a parallel loop.

`ping`, `status`, `orientation`, `preview` and `stop` are exempt from the
bridge's command lock, so they answer immediately even while a move is running
on the other connection. That is what lets a STOP button on your front panel
work, and what lets a progress indicator update during a move. If you use one
connection for everything, your stop request queues behind the move it is
supposed to interrupt.

### Building the VI

Command loop:

1. **TCP Open Connection** — `127.0.0.1`, port `5020`.
2. Build the request string. `Format Into String` is enough for fixed shapes:
   `{"command":"move","args":{"focus_mm":%.4f,"tip_deg":%.5f,"tilt_deg":%.5f}}`
   Remember the trailing `\n` (Concatenate Strings with a newline constant, or
   use `\n` in a string constant set to *"\\" Codes Display*).
3. **TCP Write**.
4. **TCP Read**, mode **CRLF** — despite the name it returns on a bare LF, so
   it reads exactly one reply. Set a timeout longer than your longest move
   (60000 ms is a sensible start), or use `wait: false` and poll.
5. Parse with **Unflatten From JSON**, wiring a cluster that mirrors the reply
   shape. Check `ok` first; if it is false, show `error` to the operator.
6. **TCP Close Connection** on shutdown.

Status loop, in parallel, on its own connection: send `{"command":"status"}\n`
every 200–500 ms and wire the result to your indicators.

Stop button: send `{"command":"stop"}\n` on the status connection. Do not
route it through the command loop.

### Try it before you build anything

```
python -m psct_motors.cli server --simulate          # terminal 1

printf '{"command":"ping"}\n' | nc 127.0.0.1 5020    # terminal 2
printf '{"command":"status"}\n' | nc 127.0.0.1 5020
printf '{"command":"move","args":{"focus_mm":26,"tip_deg":0.05}}\n' | nc 127.0.0.1 5020
```

If those work and your VI does not, the problem is in the VI, and you have
halved the search space.

### Security

The bridge has no authentication. Bound to `127.0.0.1` it is reachable only
from the same machine, which is the intended setup: run LabVIEW and the bridge
on the same box. If you bind it to a routable address, anything that can reach
the port can move the focal plane — put it behind the instrument network's own
access control.

---

## Route B — LabVIEW's native Python node

Point the Python Node at `psct_motors/labview_api.py` and call the `lv_*`
functions. They take and return only strings, doubles and int32s, and none of
them raises: errors come back inside the returned JSON as
`{"ok": false, "error": "..."}`.

### Before you start

- **Python version.** LabVIEW supports a specific set of Python versions per
  release. Check your LabVIEW's documentation and install a matching Python.
- **Bitness must match.** 64-bit LabVIEW needs 64-bit Python.
- If the Python node cannot load the module at all, it is almost always one of
  those two. Route A has neither constraint.

### Session

```
lv_open(config_path, simulate)   -> JSON string; check "ok"
...
lv_close()                       -> JSON string
```

`config_path` may be `""` to use the default location. `simulate` is `1` for
fake motors, `0` for real hardware. One session per LabVIEW process.

### Functions

| function | returns |
|---|---|
| `lv_open(config_path, simulate)` | JSON |
| `lv_close()` | JSON |
| `lv_is_connected()` | int32, 1 or 0 |
| `lv_last_error()` | string |
| `lv_get_log()` | string |
| `lv_status()` | JSON |
| `lv_orientation_array()` | 5 doubles: focus, tip, tilt, total tilt, azimuth |
| `lv_actuator_positions_mm()` | 3 doubles |
| `lv_brake_states()` | JSON |
| `lv_preview(focus, tip, tilt)` | JSON |
| `lv_move(focus, tip, tilt, wait)` | JSON |
| `lv_move_relative(dfocus, dtip, dtilt, wait)` | JSON |
| `lv_move_polar(focus, total_tilt, azimuth, wait)` | JSON |
| `lv_jog_actuator(name, mm, wait)` | JSON |
| `lv_stop()` | JSON |
| `lv_passivate()` | JSON |
| `lv_brake(action)` | JSON |
| `lv_clear_errors()` | JSON |
| `lv_set_zero()` | JSON |

`lv_orientation_array` and `lv_actuator_positions_mm` return arrays of doubles
so you can wire them straight to indicators without parsing JSON. They return
NaN on failure — check `lv_last_error()`.

### The catch with Route B

`lv_stop()` runs on the platform's unlocked stop path, so it *can* interrupt a
move. But LabVIEW's Python Node serialises calls into one Python session, so a
`lv_stop()` in a parallel loop may not get a chance to execute while a
`lv_move(..., wait=1)` is still in flight.

Work around it either way:

- Call `lv_move(..., wait=0)` and poll `lv_orientation_array()` yourself, so no
  call blocks for long, **or**
- use Route A, where the stop travels on its own socket and is not affected.

This is the concrete reason Route A is recommended: on a machine with a moving
optical assembly, the stop button should not depend on the language runtime
being free.

---

## What "focus, tip and tilt" mean

- **focus_mm** — position of the focal plane along the optical axis, measured
  on the axis itself, relative to the zero set by `set_zero`.
- **tip_deg** — right-handed rotation about +x. Positive tip raises the +y side.
- **tilt_deg** — right-handed rotation about +y. Positive tilt lowers the +x side.
- **total_tilt_deg** / **tilt_azimuth_deg** — the same two angles in polar form:
  how far the plane is tilted, and which way it slopes. Watch `total_tilt_deg`
  against your limit, since tip and tilt can each be small while their
  combination is not.

The software converts these to three actuator positions. Nothing on the LabVIEW
side needs to know about screws, gear ratios or which motor sits where.
