"""Panasonic AW-series PTZ backend (native HTTP "AW protocol").

Speaks the CGI interface documented in Panasonic's *HD Integrated Camera
Interface Specifications* (AW-HE40/HE50/HE60/HE70/HE120/HE130, AW-UE70/UE150,
AW-HR140 and the many models that share those command sets)::

    GET http://<host>[:port]/cgi-bin/aw_ptz?cmd=%23<CMD>&res=1   (pan-tilt head)
    GET http://<host>[:port]/cgi-bin/aw_cam?cmd=<CMD>&res=1      (camera block)

The response body echoes the command in lower case (``#PTS5050`` → ``pTS5050``)
or carries an error (``ER1`` unsupported, ``ER2`` busy, ``ER3`` out of range).

Command pacing
--------------
The spec requires **at least 130 ms between commands** and forbids keep-alive
(one TCP connection per command).  The AutoPTZ controller ticks at ~20 Hz and
calls :meth:`move_velocity` synchronously, so this backend never does network
I/O on the caller's thread.  Instead a single sender thread owns the camera:

* Pan/tilt and zoom are *latest-wins* slots.  Speeds are quantised to the
  camera's integer codes (01–99, 50 = stop) and an axis is only sent when its
  code actually changed, so the controller's small float jitter costs nothing.
* When both slots are dirty they alternate, so a constantly-changing pan/tilt
  cannot starve zoom.
* :meth:`stop` jumps ahead of everything else and is never ramped.
* One-shot commands (presets, home, absolute moves, OSD) queue FIFO between
  the velocity slots.
* :meth:`get_position` never blocks: it returns a cached :class:`PTZState`
  refreshed by the sender only while someone has asked for it recently, and only
  in otherwise idle slots.

Speed ramping (ease in / ease out)
----------------------------------
At ~7 commands/s each speed change reaches the head as an instant step, so the
controller's 20 Hz ramp arrives as a few coarse jumps and a "speed → 0" from the
framing deadband lands as an abrupt stop.  The sender therefore limits how far
each axis's speed code may move per second (both up *and* down), measured from
the actual time between sends so the profile holds when commands are delayed.
Explicit :meth:`stop` calls (track loss, manual stop, shutdown) bypass the ramp.
Closed-loop simulation against a stiff head put the default (27 codes/s) at
~40 % lower peak acceleration for a small tracking-error cost; below ~20 the
extra lag starts to cause overshoot.

Safety: after a transport outage the first command on the restored link is a
full stop (halt-on-reconnect, mirroring :mod:`visca_ip`), and :meth:`close`
sends a synchronous stop before tearing the thread down.
"""

from __future__ import annotations

import collections
import logging
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass

from autoptz.engine.ptz.base import PTZBackend, PTZCaps, PTZState
from autoptz.engine.ptz.reconnect import ReconnectPolicy

log = logging.getLogger(__name__)

# Minimum gap between the end of one command and the start of the next (spec §3.1
# restriction 1 — "send the commands with a gap of 130 ms between each command").
MIN_GAP_S: float = 0.130

# How long a get_position() request keeps background polling alive, and how stale
# the cached position may get before an idle slot refreshes it.
_POS_INTEREST_S: float = 2.0
_POS_MAX_AGE_S: float = 0.5

# Default speed ramp, in speed codes per second (0 disables).  Speed codes span
# 49 per direction, so 27 codes/s ≈ rest → full speed in ~1.8 s; typical
# tracking speeds (codes 52–65) are reached within a few hundred ms.
DEFAULT_RAMP_CODES_PER_S: float = 27.0
# Clamp on the time step fed to the ramp so a long idle gap can't license a big
# first jump.
_RAMP_DT_MAX_S: float = 0.2

# Throttle for transport-error WARNINGs.
_LOG_THROTTLE_S: float = 5.0

# ── wire-value mapping ────────────────────────────────────────────────────────

# Speed codes: 01 = max negative, 50 = stop, 99 = max positive.
_SPEED_STOP = 50
_SPEED_SPAN = 49

# Pan/tilt absolute position: 0x8000 = centre, ~121.4 counts per degree
# (29.7 arc-second resolution).  Pan ±175° → 0x2D09…0xD2F5.  Tilt runs the
# *other* way: smaller values are UP (0x5555 = +90°, 0x8E38 = −30°).
_PT_CENTRE = 0x8000
_PAN_HALF_SPAN = 0xD2F5 - _PT_CENTRE  # ±175°
_TILT_HALF_SPAN = _PT_CENTRE - 0x5555  # ±90°
_PT_MIN, _PT_MAX = 0x0000, 0xFFFF

# Zoom absolute position: 0x555 = wide … 0xFFF = tele.
_ZOOM_WIDE = 0x555
_ZOOM_TELE = 0xFFF

_NUM_PRESETS = 100


def speed_code(v: float) -> int:
    """Normalised velocity [-1, 1] → Panasonic speed code 1…99 (50 = stop).

    Any non-negligible request maps to at least the minimum speed (49/51), so a
    gentle correction never silently rounds to a stop.
    """
    if abs(v) < 0.01:
        return _SPEED_STOP
    mag = max(1, min(_SPEED_SPAN, round(abs(v) * _SPEED_SPAN)))
    return _SPEED_STOP + mag if v > 0 else _SPEED_STOP - mag


def ramp_code(sent: int | None, want: int, max_step: int) -> int:
    """Move speed code *sent* toward *want* by at most *max_step* (symmetric).

    ``max_step<=0`` disables the ramp.  ``sent=None`` (nothing sent yet, or the
    link was just restored — both of which begin with a stop) ramps from 50.
    Crossing through 50 on a direction reversal is ramped like any other change.
    """
    if max_step <= 0:
        return want
    if sent is None:
        sent = _SPEED_STOP
    if sent == want:
        return want
    return sent + max(-max_step, min(max_step, want - sent))


def pan_to_raw(pan: float) -> int:
    return _clamp_int(round(_PT_CENTRE + _clamp(pan, -1.0, 1.0) * _PAN_HALF_SPAN))


def tilt_to_raw(tilt: float) -> int:
    # positive (up) → smaller raw value
    return _clamp_int(round(_PT_CENTRE - _clamp(tilt, -1.0, 1.0) * _TILT_HALF_SPAN))


def raw_to_pan(raw: int) -> float:
    return _clamp((raw - _PT_CENTRE) / _PAN_HALF_SPAN, -1.0, 1.0)


def raw_to_tilt(raw: int) -> float:
    return _clamp((_PT_CENTRE - raw) / _TILT_HALF_SPAN, -1.0, 1.0)


def zoom_to_raw(zoom: float) -> int:
    return round(_ZOOM_WIDE + _clamp(zoom, 0.0, 1.0) * (_ZOOM_TELE - _ZOOM_WIDE))


def raw_to_zoom(raw: int) -> float:
    return _clamp((raw - _ZOOM_WIDE) / (_ZOOM_TELE - _ZOOM_WIDE), 0.0, 1.0)


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _clamp_int(v: int) -> int:
    return _PT_MIN if v < _PT_MIN else _PT_MAX if v > _PT_MAX else v


# ── transport ─────────────────────────────────────────────────────────────────


class AWError(Exception):
    """The camera answered with ``ER1``/``ER2``/``ER3``."""


@dataclass(frozen=True)
class _Cmd:
    """One request: ``ptz`` → aw_ptz (``#`` prefixed), else aw_cam."""

    text: str
    ptz: bool = True
    # query commands parse their reply in the sender thread
    kind: str = "control"  # "control" | "pos_pt" | "pos_zoom" | "osd_query"


class AWHttpTransport:
    """Blocking one-request-per-connection HTTP client for the AW CGI."""

    def __init__(
        self,
        host: str,
        port: int = 80,
        username: str = "",
        password: str = "",
        timeout: float = 1.0,
    ) -> None:
        self._base = f"http://{host}" if port == 80 else f"http://{host}:{port}"
        self._timeout = timeout
        handlers: list[urllib.request.BaseHandler] = []
        if username:
            mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()
            mgr.add_password(None, self._base, username, password)
            handlers += [
                urllib.request.HTTPDigestAuthHandler(mgr),
                urllib.request.HTTPBasicAuthHandler(mgr),
            ]
        # Never route LAN camera traffic through a system HTTP proxy.
        handlers.append(urllib.request.ProxyHandler({}))
        self._opener = urllib.request.build_opener(*handlers)

    def url_for(self, cmd: _Cmd) -> str:
        cgi = "aw_ptz" if cmd.ptz else "aw_cam"
        text = ("#" + cmd.text) if cmd.ptz else cmd.text
        return f"{self._base}/cgi-bin/{cgi}?cmd={urllib.parse.quote(text, safe=':')}&res=1"

    def request(self, cmd: _Cmd) -> str:
        """Send *cmd*; return the response body.  Raises OSError / AWError."""
        req = urllib.request.Request(self.url_for(cmd), headers={"Connection": "close"})
        try:
            with self._opener.open(req, timeout=self._timeout) as resp:
                body = resp.read(256).decode("ascii", "replace").strip().strip('"')
        except urllib.error.HTTPError as exc:
            # 401/403/404 are transport-level failures for our purposes.
            raise OSError(f"HTTP {exc.code} from camera") from exc
        except urllib.error.URLError as exc:
            raise OSError(str(exc.reason)) from exc
        if body.startswith("ER"):
            raise AWError(f"{cmd.text} → {body}")
        return body


# ── backend ───────────────────────────────────────────────────────────────────


class PanasonicAWBackend(PTZBackend):
    """Panasonic AW-protocol PTZ backend over HTTP.

    Args:
        host:      Camera IP or hostname.
        port:      HTTP port (default 80).
        username:  Optional CGI user (only needed when the camera's user
                   authentication is enabled).
        password:  Matching password.
        timeout:   Per-request timeout in seconds.
        probe:     Verify the camera answers (``#O`` power query) before
                   returning; raises OSError if it doesn't.  The factory relies
                   on this to fall through ``auto`` probing.
        transport: Inject a transport (tests).
        min_gap_s: Inter-command gap; only lower this in tests.
        ramp_codes_per_s: Speed-change limit per axis (ease in/out); 0 disables.
    """

    def __init__(
        self,
        host: str,
        port: int = 80,
        username: str = "",
        password: str = "",
        timeout: float = 1.0,
        *,
        probe: bool = True,
        transport: AWHttpTransport | None = None,
        min_gap_s: float = MIN_GAP_S,
        ramp_codes_per_s: float = DEFAULT_RAMP_CODES_PER_S,
    ) -> None:
        super().__init__()
        self._host = host
        self._port = port
        self._io = transport or AWHttpTransport(host, port, username, password, timeout)
        self._min_gap_s = min_gap_s
        self._ramp_cps = max(0.0, float(ramp_codes_per_s))

        if probe:
            body = self._io.request(_Cmd("O"))
            if not body.startswith("p"):
                raise OSError(f"{host}:{port} did not answer like a Panasonic AW camera: {body!r}")
            if body in ("p0", "pf"):
                log.warning(
                    "PanasonicAW %s:%d is in standby; PTZ commands will be ignored.", host, port
                )

        self.caps = PTZCaps(
            continuous_pan_tilt=True,
            continuous_zoom=True,
            absolute_pan_tilt=True,
            absolute_zoom=True,
            native_presets=True,
            query_position=True,
        )

        self._cv = threading.Condition()
        # desired / last-sent velocity codes (None = unknown → must send)
        self._want_pt: tuple[int, int] = (_SPEED_STOP, _SPEED_STOP)
        self._want_z: int = _SPEED_STOP
        self._sent_pt: tuple[int, int] | None = None
        self._sent_z: int | None = None
        self._stop_pending = False
        self._prefer_zoom = False
        self._stop_retries = 0
        # when each velocity slot was last chosen (drives the ramp's dt)
        self._pt_t = 0.0
        self._z_t = 0.0
        self._oneshots: collections.deque[_Cmd] = collections.deque()
        self._closing = False

        # position cache
        self._pos: PTZState | None = None
        self._pos_pt: tuple[float, float] | None = None
        self._pos_zoom: float = 0.0
        self._pos_t: float = 0.0
        self._pos_interest_t: float = -1e9
        self._pos_next_zoom = False

        # OSD toggle state (None until first queried)
        self._osd_on: bool | None = None

        # link health
        self._policy = ReconnectPolicy(base_s=0.5, cap_s=10.0)
        self._link_ok = True
        self._last_err_log_t = 0.0
        # The probe above counts as a command for pacing purposes.
        self._last_done_t = time.monotonic() if probe else 0.0

        self._thread = threading.Thread(target=self._run, name=f"PanasonicAW-{host}", daemon=True)
        self._thread.start()
        log.info("PanasonicAW backend ready: %s:%d", host, port)

    # ── public PTZBackend API (all non-blocking) ──────────────────────────────

    @property
    def connected(self) -> bool:
        return self._link_ok

    def move_velocity(self, pan: float, tilt: float, zoom: float = 0.0) -> None:
        pt = (speed_code(pan), speed_code(tilt))
        z = speed_code(zoom)
        with self._cv:
            if pt == self._want_pt and z == self._want_z:
                return
            self._want_pt = pt
            self._want_z = z
            self._cv.notify()

    def stop(self) -> None:
        with self._cv:
            self._want_pt = (_SPEED_STOP, _SPEED_STOP)
            self._want_z = _SPEED_STOP
            self._stop_pending = True
            self._cv.notify()

    def move_absolute(self, pan: float, tilt: float, zoom: float) -> None:
        self._enqueue(
            _Cmd(f"APC{pan_to_raw(pan):04X}{tilt_to_raw(tilt):04X}"),
            _Cmd(f"AXZ{zoom_to_raw(zoom):03X}"),
        )

    def get_position(self) -> PTZState | None:
        now = time.monotonic()
        with self._cv:
            waking = (now - self._pos_interest_t) >= _POS_INTEREST_S
            self._pos_interest_t = now
            if waking:
                self._cv.notify()
            return self._pos

    def goto_preset(self, idx: int) -> None:
        if 0 <= idx < _NUM_PRESETS:
            self._enqueue(_Cmd(f"R{idx:02d}"))
        else:
            log.warning("PanasonicAW preset %d out of range 0–%d", idx, _NUM_PRESETS - 1)

    def save_preset(self, idx: int) -> None:
        if 0 <= idx < _NUM_PRESETS:
            self._enqueue(_Cmd(f"M{idx:02d}"))
        else:
            log.warning("PanasonicAW preset %d out of range 0–%d", idx, _NUM_PRESETS - 1)

    def home(self) -> None:
        self._enqueue(_Cmd(f"APC{_PT_CENTRE:04X}{_PT_CENTRE:04X}"))

    def osd_menu(self) -> None:
        with self._cv:
            if self._osd_on is None:
                self._oneshots.append(_Cmd("QUS", ptz=False, kind="osd_query"))
            else:
                # Flip the tracked state now (not on reply) so rapid presses toggle.
                self._osd_on = not self._osd_on
                self._oneshots.append(_Cmd(f"DUS:{int(self._osd_on)}", ptz=False))
            self._cv.notify()

    def close(self) -> None:
        with self._cv:
            if self._closing:
                return
            self._closing = True
            self._cv.notify()
        self._thread.join(timeout=2.0)
        # Synchronous final halt — the thread is gone, so we own the wire.
        for text in ("PTS5050", "Z50"):
            self._pace()
            try:
                self._io.request(_Cmd(text))
            except Exception:  # noqa: BLE001 - best-effort on shutdown
                pass
            self._last_done_t = time.monotonic()

    # ── sender thread ─────────────────────────────────────────────────────────

    def _enqueue(self, *cmds: _Cmd) -> None:
        with self._cv:
            self._oneshots.extend(cmds)
            self._cv.notify()

    def _pace(self) -> None:
        wait = self._last_done_t + self._min_gap_s - time.monotonic()
        if wait > 0:
            time.sleep(wait)

    def _next_cmd(self, now: float) -> _Cmd | None:
        """Pick the next command to send (caller holds ``_cv``)."""
        if self._stop_pending:
            self._stop_pending = False
            # Send both halves; mark sent state stale so whichever we don't send
            # first still goes out next slot.
            if self._sent_pt != (_SPEED_STOP, _SPEED_STOP):
                self._stop_pending = self._sent_z != _SPEED_STOP
                return _Cmd("PTS5050")
            if self._sent_z != _SPEED_STOP:
                return _Cmd("Z50")
        if self._oneshots:
            return self._oneshots.popleft()

        pt_dirty = self._want_pt != self._sent_pt
        z_dirty = self._want_z != self._sent_z
        if pt_dirty and z_dirty:
            send_zoom = self._prefer_zoom
            self._prefer_zoom = not self._prefer_zoom
        else:
            send_zoom = z_dirty
        if send_zoom:
            step = self._ramp_step(now, self._z_t)
            self._z_t = now
            return _Cmd(f"Z{ramp_code(self._sent_z, self._want_z, step):02d}")
        if pt_dirty:
            step = self._ramp_step(now, self._pt_t)
            self._pt_t = now
            sent = self._sent_pt
            pan = ramp_code(sent[0] if sent else None, self._want_pt[0], step)
            tilt = ramp_code(sent[1] if sent else None, self._want_pt[1], step)
            return _Cmd(f"PTS{pan:02d}{tilt:02d}")

        # Idle: refresh the position cache if anyone wants it.
        if (now - self._pos_interest_t) < _POS_INTEREST_S and (
            (now - self._pos_t) >= _POS_MAX_AGE_S or self._pos_next_zoom
        ):
            if self._pos_next_zoom:
                return _Cmd("GZ", kind="pos_zoom")
            return _Cmd("APC", kind="pos_pt")
        return None

    def _ramp_step(self, now: float, last_t: float) -> int:
        """Max speed-code change allowed for a slot last chosen at *last_t*."""
        if self._ramp_cps <= 0.0:
            return 0  # ramp disabled
        dt = min(_RAMP_DT_MAX_S, max(0.0, now - last_t))
        return max(1, round(self._ramp_cps * dt))

    def _idle_wait_s(self, now: float) -> float | None:
        """How long to sleep with nothing to send (None = until notified)."""
        if (now - self._pos_interest_t) < _POS_INTEREST_S:
            return max(0.01, self._pos_t + _POS_MAX_AGE_S - now)
        return None

    def _run(self) -> None:
        while True:
            with self._cv:
                while True:
                    if self._closing:
                        return
                    now = time.monotonic()
                    if not self._link_ok and not self._policy.should_attempt(now):
                        self._cv.wait(timeout=0.1)
                        continue
                    cmd = self._next_cmd(now)
                    if cmd is not None:
                        break
                    self._cv.wait(timeout=self._idle_wait_s(now))
                # Snapshot what this send will establish if it succeeds.
                restoring = not self._link_ok

            self._pace()
            if restoring:
                # Halt-on-reconnect: whatever the head was doing when the link
                # dropped, stop it before anything else goes out.
                if not self._send(_Cmd("PTS5050")):
                    if cmd.kind == "control" and not cmd.text.startswith(("PTS", "Z")):
                        with self._cv:
                            self._oneshots.appendleft(cmd)
                    continue
                self._pace()
                self._send(_Cmd("Z50"))
                with self._cv:
                    self._sent_pt = (_SPEED_STOP, _SPEED_STOP)
                    self._sent_z = _SPEED_STOP
                    if cmd.text in ("PTS5050", "Z50"):
                        continue
                self._pace()
            self._send(cmd)

    def _send(self, cmd: _Cmd) -> bool:
        """Perform one request on the sender thread; update state.  True on success."""
        try:
            body = self._io.request(cmd)
        except AWError as exc:
            # Camera is reachable but rejected the command (busy / unsupported /
            # out of range).  Treat the link as healthy; log at debug.
            self._last_done_t = time.monotonic()
            log.debug("PanasonicAW %s", exc)
            self._mark_link_ok()
            self._apply_sent(cmd, failed=True)
            return True
        except Exception as exc:  # noqa: BLE001 - never let the thread die
            self._last_done_t = time.monotonic()
            self._on_transport_error(cmd, exc)
            return False
        self._last_done_t = time.monotonic()
        self._mark_link_ok()
        self._apply_sent(cmd, failed=False, body=body)
        return True

    def _apply_sent(self, cmd: _Cmd, *, failed: bool, body: str = "") -> None:
        t = cmd.text
        with self._cv:
            if t.startswith("PTS") and len(t) == 7:
                self._sent_pt = (int(t[3:5]), int(t[5:7]))
            elif t.startswith("Z") and len(t) == 3:
                self._sent_z = int(t[1:3])
            if failed:
                # A rejected stop (e.g. ER2 busy) is retried a few times rather
                # than assumed — a missed stop is the one failure that matters.
                if t in ("PTS5050", "Z50") and self._stop_retries < 3:
                    self._stop_retries += 1
                    if t == "Z50":
                        self._sent_z = None
                    else:
                        self._sent_pt = None
                    return
                if cmd.kind == "pos_pt":
                    self._pos_t = time.monotonic()  # back off; don't hammer
                return
            if t in ("PTS5050", "Z50"):
                self._stop_retries = 0
            if cmd.kind == "pos_pt":
                self._parse_pos_pt(body)
            elif cmd.kind == "pos_zoom":
                self._parse_pos_zoom(body)
            elif cmd.kind == "osd_query":
                # "OUS:0" / "OUS:1" → toggle to the opposite state
                self._osd_on = not body.endswith("1")
                self._oneshots.appendleft(_Cmd(f"DUS:{int(self._osd_on)}", ptz=False))

    def _parse_pos_pt(self, body: str) -> None:
        # "aPC" + 4 hex pan + 4 hex tilt
        try:
            if not body.startswith("aPC") or len(body) < 11:
                raise ValueError(body)
            pan = raw_to_pan(int(body[3:7], 16))
            tilt = raw_to_tilt(int(body[7:11], 16))
        except ValueError:
            log.debug("PanasonicAW bad APC reply %r", body)
            self._pos_t = time.monotonic()
            return
        self._pos_pt = (pan, tilt)
        self._pos_next_zoom = True

    def _parse_pos_zoom(self, body: str) -> None:
        self._pos_next_zoom = False
        # "gz" + 3 hex, or "gz---" while in standby
        try:
            if body.startswith("gz") and body[2:5] != "---":
                self._pos_zoom = raw_to_zoom(int(body[2:5], 16))
        except ValueError:
            log.debug("PanasonicAW bad GZ reply %r", body)
        if self._pos_pt is not None:
            now = time.monotonic()
            self._pos = PTZState(
                pan=self._pos_pt[0], tilt=self._pos_pt[1], zoom=self._pos_zoom, timestamp=now
            )
            self._pos_t = now

    def _mark_link_ok(self) -> None:
        if not self._link_ok:
            log.info("PanasonicAW %s:%d link restored", self._host, self._port)
        self._link_ok = True
        self._policy.record_success()

    def _on_transport_error(self, cmd: _Cmd, exc: Exception) -> None:
        now = time.monotonic()
        with self._cv:
            self._link_ok = False
            # Unknown camera state: resend velocity once the link is back.
            self._sent_pt = None
            self._sent_z = None
            # A failed one-shot (preset / home) is retried on the restored link;
            # queries are simply re-issued by the idle poller.
            if cmd.kind == "control" and not cmd.text.startswith(("PTS", "Z")):
                self._oneshots.appendleft(cmd)
        self._policy.record_failure(now)
        if now - self._last_err_log_t >= _LOG_THROTTLE_S:
            self._last_err_log_t = now
            log.warning("PanasonicAW %s:%d %s failed: %s", self._host, self._port, cmd.text, exc)
