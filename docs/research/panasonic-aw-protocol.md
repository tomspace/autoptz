# Panasonic AW protocol — implementation notes

Notes behind `autoptz/engine/ptz/panasonic_aw.py`. The authoritative reference is
Panasonic's *HD Integrated Camera Interface Specifications* (English edition),
available from Panasonic's
[professional AV support site](https://eww.pass.panasonic.co.jp/pro-av/support/content/guide/EN/top.html).
It is not vendored here because it is Panasonic's copyrighted document.

## Transport

- One HTTP GET per command, no keep-alive:
  `http://<host>[:port]/cgi-bin/aw_ptz?cmd=%23<CMD>&res=1` for the pan-tilt head,
  `.../cgi-bin/aw_cam?cmd=<CMD>&res=1` (no `#`) for the camera block.
- The body echoes the command in lower case (`#PTS5050` → `pTS5050`) or returns
  `ER1` (unsupported), `ER2` (busy) or `ER3` (out of range).
- **At least 130 ms between commands** (spec §3.1, restriction 1). The backend
  measures the gap from the end of the previous reply.
- Settings commands write the camera's EEPROM, so they must not be sent
  periodically. The backend sends none; presets are written only on request.

## Commands used

| Purpose | Command | Notes |
| --- | --- | --- |
| Probe / power | `#O` → `p0`/`p1`/`p3` | Used to confirm a Panasonic camera on connect; `p0` = standby. |
| Pan/tilt speed | `#PTS<pp><tt>` | `01`–`99`, `50` = stop. Pan `01` = left max; tilt `01` = down max. |
| Zoom speed | `#Z<zz>` | `01` = wide max … `99` = tele max, `50` = stop. |
| Absolute pan/tilt | `#APC<pppp><tttt>` / query `#APC` | Hex, `8000` = centre, ~121.4 counts/degree. Pan ±175° = `2D09`–`D2F5`. Tilt runs *inverted*: `5555` = +90° (up), `8E38` = −30°. |
| Absolute zoom | `#AXZ<zzz>` / query `#GZ` | `555` (wide) … `FFF` (tele); `#GZ` returns `---` in standby. |
| Presets | `#M<nn>` save, `#R<nn>` recall | `00`–`99` (preset 1–100). |
| Home | `#APC80008000` | |
| OSD menu | `aw_cam` `QUS` → `OUS:<0/1>`, `DUS:<0/1>` | Toggled from the queried state. |

## Why the backend is structured the way it is

The AutoPTZ controller ticks at 20 Hz and calls `move_velocity()` synchronously,
but a Panasonic head accepts at most ~7 commands/s, each needing a fresh TCP
connection. So a single sender thread owns the camera:

- pan/tilt and zoom are latest-wins slots, quantised to speed codes and sent only
  when the code changes; when both are pending they alternate;
- `stop()` jumps the queue; a stop rejected with `ER2` is retried;
- `get_position()` returns a cache refreshed in idle slots only while it is being
  asked for;
- after a dropped link, the first commands on the restored link are stops.

## Speed ramping

At ~7 commands/s every speed change lands on the head as an instant step. The
controller limits acceleration but not deceleration, and detection noise makes
the commanded speed wobble by a code or two per update, so moves looked jerky on
an AW-HE130. A closed-loop simulation (the real `PTZController`, a stiff-head
plant, 120 ms detection latency, Gaussian detection noise) compared fixes:

| Variant | Peak accel | Tracking error (RMS) |
| --- | --- | --- |
| 20 Hz ideal (no pacing) | 7.5 | 0.294 |
| Panasonic pacing, no ramp | 13.7 | 0.297 |
| Ignore ±1–2 code changes | 13.6–14.1 | 0.295–0.298 |
| Higher `aim_smoothing` (0.7 / 0.85) | 14.9 / 15.7 | 0.309 / 0.338 |
| Symmetric ramp, 4 codes/command (≈27 codes/s) | 8.3 | 0.323 |
| Symmetric ramp, 3 codes/command | 6.3 | 0.451 (overshoots) |

The backend ramps each axis by at most `27 codes/s × Δt` (both up and down),
using the measured time between sends so the profile holds when commands are
delayed; explicit stops bypass it. End-to-end against a fake camera the largest
single speed jump fell from 16 codes to 5. Tunable via `AUTOPTZ_PANASONIC_RAMP`.

The remaining limit is the ~7 Hz update rate itself. A further step would be
position-based moves (`#APS`, absolute position + speed, supported on
AW-HE130/HE40/UE70/HE42/HR140/UE150) so the camera's own motion profile does the
easing.

## Model notes

- `#PTV` (pan/tilt/zoom/focus/iris in one reply) is AW-UE150 only, so it isn't used.
- `#APS`/`#RPC` relative and speed-controlled moves are not on the HE50/HE60/HE120.
- The AK-UB300 has no pan-tilt head and rejects these commands.
- AW-HEA10 (wide "tap-assist" companion for the HE130/HE40) exposes only HDMI
  video publicly; its tablet control protocol is undocumented.
