"""Panasonic AW (HTTP) PTZ backend tests.

A real HTTP server on localhost plays the camera (``FakeAWCamera``): it parses
``/cgi-bin/aw_ptz`` and ``/cgi-bin/aw_cam`` requests, records each command with
its arrival time, answers like the spec, and flags any command that arrives
sooner than the 130 ms minimum gap.  Covers:

* value mapping (speed codes, absolute pan/tilt/zoom raw ↔ normalised);
* the non-blocking contract (move_velocity never waits on the network);
* pacing, latest-wins coalescing and pan/tilt ↔ zoom fairness;
* stop priority, presets, home, absolute moves, OSD toggle;
* cached position polling;
* probe failure, halt-on-reconnect and basic auth;
* speed ramping (ease in / ease out; explicit stop bypasses it);
* factory dispatch + ``user:pass@host`` parsing.
"""

from __future__ import annotations

import base64
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from autoptz.config.models import PTZConfig
from autoptz.engine.ptz import panasonic_aw as aw
from autoptz.engine.ptz.factory import _split_userinfo, build_backend
from autoptz.engine.ptz.panasonic_aw import PanasonicAWBackend

GAP = 0.03  # shortened inter-command gap for most tests (real camera: 0.130)


# ─────────────────────────────────────────────────────────────────────────────
# Fake camera
# ─────────────────────────────────────────────────────────────────────────────


class FakeAWCamera:
    def __init__(self, *, min_gap: float = 0.0, auth: tuple[str, str] | None = None) -> None:
        self.min_gap = min_gap
        self.auth = auth
        self.log: list[tuple[float, str, str]] = []  # (t_start, cgi, cmd)
        self.gap_violations: list[str] = []
        self.pan, self.tilt, self.zoom = 0x8000, 0x8000, 0x555
        self.osd = 0
        self.power = "p1"
        self.busy_next: set[str] = set()  # commands to answer ER2 once
        self.down = False
        self._last_end = 0.0
        self._lock = threading.Lock()
        cam = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a: object) -> None:
                pass

            def do_GET(self) -> None:  # noqa: N802
                if cam.down:
                    self.close_connection = True
                    self.connection.close()
                    return
                if cam.auth is not None:
                    want = "Basic " + base64.b64encode(":".join(cam.auth).encode()).decode()
                    if self.headers.get("Authorization") != want:
                        self.send_response(401)
                        self.send_header("WWW-Authenticate", 'Basic realm="cam"')
                        self.send_header("Content-Length", "0")
                        self.end_headers()
                        return
                u = urllib.parse.urlparse(self.path)
                cgi = u.path.rsplit("/", 1)[-1]
                cmd = urllib.parse.parse_qs(u.query)["cmd"][0]
                body = cam.handle(cgi, cmd)
                data = body.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/plain")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Connection", "close")
                self.end_headers()
                self.wfile.write(data)
                with cam._lock:
                    cam._last_end = time.monotonic()

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.port = self.server.server_address[1]
        self._t = threading.Thread(target=self.server.serve_forever, daemon=True)
        self._t.start()

    def handle(self, cgi: str, cmd: str) -> str:
        now = time.monotonic()
        with self._lock:
            if self.min_gap and self._last_end and now - self._last_end < self.min_gap - 0.005:
                self.gap_violations.append(cmd)
            self.log.append((now, cgi, cmd))
        if cmd.lstrip("#") in self.busy_next:
            self.busy_next.discard(cmd.lstrip("#"))
            return "ER2:" + cmd.lstrip("#")
        if cgi == "aw_cam":
            if cmd == "QUS":
                return f"OUS:{self.osd}"
            if cmd.startswith("DUS:"):
                self.osd = int(cmd[4:])
                return cmd
            return "ER1:" + cmd
        c = cmd[1:]  # strip '#'
        if c == "O":
            return self.power
        if c.startswith("PTS"):
            return "pTS" + c[3:]
        if c.startswith("Z") and len(c) == 3:
            return "zS" + c[1:]
        if c == "APC":
            return f"aPC{self.pan:04X}{self.tilt:04X}"
        if c.startswith("APC"):
            self.pan, self.tilt = int(c[3:7], 16), int(c[7:11], 16)
            return "aPC" + c[3:]
        if c == "GZ":
            return f"gz{self.zoom:03X}"
        if c.startswith("AXZ"):
            self.zoom = int(c[3:], 16)
            return "axz" + c[3:]
        if c[0] in "MR" and len(c) == 3:
            return "s" + c[1:]
        return "ER1:" + c

    def cmds(self) -> list[str]:
        with self._lock:
            return [c for _, _, c in self.log]

    def close(self) -> None:
        self.server.shutdown()
        self.server.server_close()


@pytest.fixture
def cam():
    c = FakeAWCamera()
    yield c
    c.close()


def _backend(cam: FakeAWCamera, **kw: object) -> PanasonicAWBackend:
    kw.setdefault("min_gap_s", GAP)
    kw.setdefault("ramp_codes_per_s", 0.0)  # exact codes; ramping has its own tests
    return PanasonicAWBackend("127.0.0.1", cam.port, timeout=0.5, **kw)  # type: ignore[arg-type]


def _wait(pred, timeout: float = 2.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.005)
    return pred()


# ─────────────────────────────────────────────────────────────────────────────
# Pure mapping
# ─────────────────────────────────────────────────────────────────────────────


class TestMapping:
    @pytest.mark.parametrize(
        ("v", "code"),
        [
            (0.0, 50),
            (0.005, 50),
            (-0.005, 50),
            (1.0, 99),
            (-1.0, 1),
            (0.02, 51),
            (-0.02, 49),
            (0.5, 74),
            (-0.5, 26),
            (2.0, 99),
            (-2.0, 1),
        ],
    )
    def test_speed_code(self, v: float, code: int) -> None:
        assert aw.speed_code(v) == code

    def test_pan_tilt_spec_reference_points(self) -> None:
        # Spec: pan ±175° = 2D09…D2F5; tilt +90° = 5555, −30° = 8E38; centre 8000.
        assert aw.pan_to_raw(0.0) == 0x8000
        assert aw.pan_to_raw(1.0) == 0xD2F5
        assert aw.pan_to_raw(-1.0) == 0x2D0B  # symmetric about 8000 (spec rounds to 2D09)
        assert aw.tilt_to_raw(1.0) == 0x5555
        assert abs(aw.tilt_to_raw(-1 / 3) - 0x8E38) <= 1

    def test_round_trip(self) -> None:
        for v in (-0.9, -0.3, 0.0, 0.42, 1.0):
            assert aw.raw_to_pan(aw.pan_to_raw(v)) == pytest.approx(v, abs=1e-4)
            assert aw.raw_to_tilt(aw.tilt_to_raw(v)) == pytest.approx(v, abs=1e-4)
        for z in (0.0, 0.25, 1.0):
            assert aw.raw_to_zoom(aw.zoom_to_raw(z)) == pytest.approx(z, abs=1e-3)
        assert aw.zoom_to_raw(0.0) == 0x555 and aw.zoom_to_raw(1.0) == 0xFFF

    @pytest.mark.parametrize(
        ("sent", "want", "step", "out"),
        [
            (50, 99, 4, 54),
            (99, 50, 4, 95),
            (52, 47, 4, 48),  # reversal ramps through 50
            (50, 52, 4, 52),  # within one step → exact
            (None, 99, 4, 54),  # unknown state → ramp from stop
            (50, 99, 0, 99),  # disabled
        ],
    )
    def test_ramp_code(self, sent: int | None, want: int, step: int, out: int) -> None:
        assert aw.ramp_code(sent, want, step) == out

    def test_url_encodes_hash(self) -> None:
        t = aw.AWHttpTransport("10.0.0.5")
        assert t.url_for(aw._Cmd("PTS5050")) == (
            "http://10.0.0.5/cgi-bin/aw_ptz?cmd=%23PTS5050&res=1"
        )
        assert t.url_for(aw._Cmd("DUS:1", ptz=False)) == (
            "http://10.0.0.5/cgi-bin/aw_cam?cmd=DUS:1&res=1"
        )
        assert aw.AWHttpTransport("h", 8080).url_for(aw._Cmd("O")).startswith("http://h:8080/")


# ─────────────────────────────────────────────────────────────────────────────
# Backend behaviour against the fake camera
# ─────────────────────────────────────────────────────────────────────────────


class TestBackend:
    def test_probe_and_caps(self, cam: FakeAWCamera) -> None:
        b = _backend(cam)
        try:
            assert cam.cmds()[0] == "#O"
            assert b.caps.native_presets and b.caps.absolute_pan_tilt and b.caps.query_position
        finally:
            b.close()

    def test_probe_rejects_unreachable(self) -> None:
        with pytest.raises(OSError):
            PanasonicAWBackend("127.0.0.1", 1, timeout=0.2)

    def test_probe_rejects_non_panasonic(self, cam: FakeAWCamera) -> None:
        cam.power = "hello"
        with pytest.raises(OSError):
            _backend(cam)

    def test_move_velocity_does_not_block(self, cam: FakeAWCamera) -> None:
        b = _backend(cam, min_gap_s=0.2)
        try:
            t0 = time.perf_counter()
            for i in range(50):
                b.move_velocity((i % 10) / 10, -0.3, 0.1)
            assert time.perf_counter() - t0 < 0.05
        finally:
            b.close()

    def test_velocity_mapping_on_the_wire(self, cam: FakeAWCamera) -> None:
        b = _backend(cam)
        try:
            b.move_velocity(-1.0, 1.0, 0.0)
            assert _wait(lambda: "#PTS0199" in cam.cmds())
            b.move_velocity(-1.0, 1.0, -1.0)
            assert _wait(lambda: "#Z01" in cam.cmds())
        finally:
            b.close()

    def test_unchanged_codes_are_not_resent(self, cam: FakeAWCamera) -> None:
        b = _backend(cam)
        try:
            b.move_velocity(0.4, 0.0)
            assert _wait(lambda: "#PTS7050" in cam.cmds())
            n = len(cam.cmds())
            for _ in range(20):
                b.move_velocity(0.402, 0.001)  # quantises to the same codes
            time.sleep(GAP * 5)
            assert len(cam.cmds()) == n
        finally:
            b.close()

    def test_latest_wins_coalescing(self, cam: FakeAWCamera) -> None:
        b = _backend(cam, min_gap_s=0.15)
        try:
            b.move_velocity(0.1, 0.0)
            time.sleep(0.02)
            for v in (0.2, 0.3, 0.4, 0.5, 0.6):  # all inside one gap
                b.move_velocity(v, 0.0)
            assert _wait(lambda: f"#PTS{aw.speed_code(0.6)}50" in cam.cmds())
            pts = [c for c in cam.cmds() if c.startswith("#PTS")]
            assert len(pts) <= 2  # first command + the final value, nothing between
        finally:
            b.close()

    def test_zoom_not_starved_by_pan(self, cam: FakeAWCamera) -> None:
        b = _backend(cam)
        try:
            stop = threading.Event()

            def jitter() -> None:
                i = 0
                while not stop.is_set():
                    b.move_velocity(0.2 + (i % 2) * 0.3, 0.0, 0.5)
                    i += 1
                    time.sleep(0.002)

            th = threading.Thread(target=jitter)
            th.start()
            try:
                assert _wait(lambda: "#Z74" in cam.cmds(), timeout=1.0)
            finally:
                stop.set()
                th.join()
        finally:
            b.close()

    def test_real_gap_respected(self) -> None:
        cam = FakeAWCamera(min_gap=aw.MIN_GAP_S)
        b = PanasonicAWBackend("127.0.0.1", cam.port, timeout=0.5)  # real 130 ms gap
        try:
            for i in range(8):
                b.move_velocity(0.1 * i, -0.1 * i, 0.05 * i)
                b.goto_preset(i)
                time.sleep(0.03)
            assert _wait(lambda: "#R07" in cam.cmds(), timeout=5.0)
            assert cam.gap_violations == []
        finally:
            b.close()
            cam.close()

    def test_stop_jumps_queue(self, cam: FakeAWCamera) -> None:
        b = _backend(cam, min_gap_s=0.1)
        try:
            b.move_velocity(1.0, 0.0, 1.0)
            assert _wait(lambda: any(c.startswith("#PTS99") for c in cam.cmds()))
            for i in range(5):
                b.goto_preset(i)
            b.stop()
            assert _wait(lambda: "#R04" in cam.cmds())
            cmds = cam.cmds()
            # (the backend also sends one stop at startup; find the one after the move)
            cmds = cmds[cmds.index("#PTS9950") :]
            # stop went out before the queued presets drained
            assert cmds.index("#PTS5050") < cmds.index("#R04")
        finally:
            b.close()

    def test_busy_stop_is_retried(self, cam: FakeAWCamera) -> None:
        b = _backend(cam)
        try:
            b.move_velocity(1.0, 0.0)
            assert _wait(lambda: "#PTS9950" in cam.cmds())
            cam.busy_next.add("PTS5050")
            b.stop()
            assert _wait(lambda: cam.cmds().count("#PTS5050") >= 2)
        finally:
            b.close()

    @staticmethod
    def _pan_codes(cam: FakeAWCamera, start: int = 0) -> list[int]:
        return [int(c[4:6]) for c in cam.cmds()[start:] if c.startswith("#PTS")]

    def test_ramp_eases_in_and_out(self, cam: FakeAWCamera) -> None:
        rate = 100.0
        max_step = round(rate * aw._RAMP_DT_MAX_S)
        b = _backend(cam, ramp_codes_per_s=rate)
        try:
            n = len(cam.cmds())
            b.move_velocity(1.0, 0.0)
            assert _wait(lambda: "#PTS9950" in cam.cmds(), timeout=5.0)
            up = self._pan_codes(cam, n)
            assert len(up) >= 3  # took several steps, not one jump
            assert all(0 < b2 - a <= max_step for a, b2 in zip(up, up[1:], strict=False))
            n = len(cam.cmds())
            b.move_velocity(0.0, 0.0)  # controller "speed 0" (e.g. deadband) eases out
            assert _wait(lambda: "#PTS5050" in cam.cmds()[n:], timeout=5.0)
            down = self._pan_codes(cam, n)
            assert len(down) >= 3
            assert all(0 < a - b2 <= max_step for a, b2 in zip(down, down[1:], strict=False))
        finally:
            b.close()

    def test_ramp_applies_to_zoom(self, cam: FakeAWCamera) -> None:
        b = _backend(cam, ramp_codes_per_s=100.0)
        try:
            n = len(cam.cmds())
            b.move_velocity(0.0, 0.0, 1.0)
            assert _wait(lambda: "#Z99" in cam.cmds(), timeout=5.0)
            zs = [c for c in cam.cmds()[n:] if c.startswith("#Z")]
            assert len(zs) >= 3
        finally:
            b.close()

    def test_explicit_stop_bypasses_ramp(self, cam: FakeAWCamera) -> None:
        b = _backend(cam, ramp_codes_per_s=100.0)
        try:
            b.move_velocity(1.0, 0.0)
            assert _wait(lambda: "#PTS9950" in cam.cmds(), timeout=5.0)
            n = len(cam.cmds())
            b.stop()
            assert _wait(lambda: "#PTS5050" in cam.cmds()[n:])
            assert self._pan_codes(cam, n)[0] == 50  # straight to stop, no ramp-down
        finally:
            b.close()

    def test_presets_home_absolute(self, cam: FakeAWCamera) -> None:
        b = _backend(cam)
        try:
            b.save_preset(0)
            b.goto_preset(99)
            b.goto_preset(100)  # out of range → ignored
            b.home()
            b.move_absolute(0.5, -0.2, 1.0)
            assert _wait(lambda: any(c.startswith("#AXZ") for c in cam.cmds()))
            cmds = cam.cmds()
            assert "#M00" in cmds and "#R99" in cmds and "#R100" not in cmds
            assert "#APC80008000" in cmds
            assert f"#APC{aw.pan_to_raw(0.5):04X}{aw.tilt_to_raw(-0.2):04X}" in cmds
            assert "#AXZFFF" in cmds
        finally:
            b.close()

    def test_osd_toggle(self, cam: FakeAWCamera) -> None:
        b = _backend(cam)
        try:
            b.osd_menu()
            assert _wait(lambda: cam.osd == 1)
            assert "QUS" in cam.cmds()
            b.osd_menu()
            assert _wait(lambda: cam.osd == 0)
        finally:
            b.close()

    def test_position_cached_and_polled_only_on_demand(self, cam: FakeAWCamera) -> None:
        cam.pan, cam.tilt, cam.zoom = aw.pan_to_raw(0.5), aw.tilt_to_raw(0.25), 0xAAA
        b = _backend(cam)
        try:
            time.sleep(GAP * 5)
            assert "#APC" not in cam.cmds()  # nobody asked → no polling
            t0 = time.perf_counter()
            assert b.get_position() is None  # first call: nothing cached, but non-blocking
            assert time.perf_counter() - t0 < 0.01
            assert _wait(lambda: b.get_position() is not None)
            pos = b.get_position()
            assert pos is not None
            assert pos.pan == pytest.approx(0.5, abs=1e-3)
            assert pos.tilt == pytest.approx(0.25, abs=1e-3)
            assert pos.zoom == pytest.approx(0.5, abs=0.01)
        finally:
            b.close()

    def test_close_sends_final_stop(self, cam: FakeAWCamera) -> None:
        b = _backend(cam)
        b.move_velocity(0.8, 0.0, 0.3)
        assert _wait(lambda: any(c.startswith("#PTS") for c in cam.cmds()))
        b.close()
        assert cam.cmds()[-2:] == ["#PTS5050", "#Z50"]

    def test_halt_on_reconnect(self, cam: FakeAWCamera) -> None:
        b = _backend(cam)
        try:
            b.move_velocity(0.5, 0.0)
            assert _wait(lambda: "#PTS7450" in cam.cmds())
            cam.down = True
            b.move_velocity(0.6, 0.0)
            assert _wait(lambda: not b.connected)
            cam.down = False
            n = len(cam.cmds())
            b.move_velocity(0.7, 0.0)
            assert _wait(lambda: "#PTS8450" in cam.cmds()[n:], timeout=3.0)
            after = cam.cmds()[n:]
            assert after[0] == "#PTS5050"  # halted first
            assert b.connected
        finally:
            b.close()

    def test_basic_auth(self) -> None:
        cam = FakeAWCamera(auth=("admin", "12345"))
        try:
            with pytest.raises(OSError):
                PanasonicAWBackend("127.0.0.1", cam.port, timeout=0.5)
            b = PanasonicAWBackend(
                "127.0.0.1", cam.port, username="admin", password="12345", timeout=0.5
            )
            b.close()
        finally:
            cam.close()


# ─────────────────────────────────────────────────────────────────────────────
# Factory
# ─────────────────────────────────────────────────────────────────────────────


class TestFactory:
    @pytest.mark.parametrize(
        ("addr", "expected"),
        [
            ("10.0.0.5", ("", "10.0.0.5")),
            ("admin:pw@10.0.0.5:8080", ("admin:pw", "10.0.0.5:8080")),
            ("http://u:p:x@cam/foo", ("u:p:x", "http://cam/foo")),
            ("cam/a@b", ("", "cam/a@b")),
            (None, ("", "")),
        ],
    )
    def test_split_userinfo(self, addr: str | None, expected: tuple[str, str]) -> None:
        assert _split_userinfo(addr) == expected

    def test_build_panasonic(self, cam: FakeAWCamera) -> None:
        b = build_backend(PTZConfig(backend="panasonic", address=f"127.0.0.1:{cam.port}"))
        try:
            assert isinstance(b, PanasonicAWBackend)
        finally:
            if b is not None:
                b.close()

    def test_build_panasonic_unreachable_returns_none(self) -> None:
        assert build_backend(PTZConfig(backend="panasonic", address="127.0.0.1:1")) is None

    def test_build_panasonic_no_address(self) -> None:
        assert build_backend(PTZConfig(backend="panasonic")) is None

    def test_ramp_env_override(self, monkeypatch: pytest.MonkeyPatch) -> None:
        from autoptz.engine.ptz import factory

        monkeypatch.delenv("AUTOPTZ_PANASONIC_RAMP", raising=False)
        assert factory._panasonic_ramp() == aw.DEFAULT_RAMP_CODES_PER_S
        monkeypatch.setenv("AUTOPTZ_PANASONIC_RAMP", "15")
        assert factory._panasonic_ramp() == 15.0
        monkeypatch.setenv("AUTOPTZ_PANASONIC_RAMP", "0")
        assert factory._panasonic_ramp() == 0.0
        monkeypatch.setenv("AUTOPTZ_PANASONIC_RAMP", "fast")
        assert factory._panasonic_ramp() == aw.DEFAULT_RAMP_CODES_PER_S

    def test_auto_finds_panasonic_after_onvif(
        self, cam: FakeAWCamera, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        from autoptz.engine.ptz import factory

        monkeypatch.setattr(factory, "_build_onvif", lambda ptz: None)
        b = build_backend(PTZConfig(backend="auto", address=f"127.0.0.1:{cam.port}"))
        try:
            assert isinstance(b, PanasonicAWBackend)
        finally:
            if b is not None:
                b.close()
