#!/usr/bin/env python3
"""
Home-pose service for Wuji Hand / Hand 2.

Provides a small API other code (teleop, scripts, HTTP) can call to return the
hand to a saved home configuration (``home_pose.json`` from ``3.save_home.py``).

Python API (same process as teleop)::

    from home_pose_service import HomePoseService

    svc = HomePoseService.from_config("home_pose.json")
    svc.bind_sender(send)          # send: list[float] -> None
    svc.go_home(duration_s=1.5)    # blocking stream
    svc.request_go_home(1.5)       # non-blocking; teleop loop calls poll()

HTTP (optional, same process)::

    svc.serve_http(port=8765)
    # POST http://127.0.0.1:8765/go_home
    # GET  http://127.0.0.1:8765/home

CLI (standalone, takes the hand while teleop is stopped)::

    python home_pose_service.py --go-home
    python home_pose_service.py --serve --port 8765
"""

from __future__ import annotations

import argparse
import contextlib
import json
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable, Optional
from urllib.parse import parse_qs, urlparse

from wuji_sdk import (
    DeviceType,
    JointCommand,
    LowPass,
    SdkManager,
    WujiHand,
    WujiHand2,
)

TOTAL_JOINTS = 20
FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")
DEFAULT_CONFIG = Path(__file__).resolve().with_name("home_pose.json")
DEFAULT_FPS = 120
SendFn = Callable[[list[float]], None]


@dataclass
class _GoHomeRequest:
    duration_s: float
    event: threading.Event


class HomePoseService:
    """Load / hold / go-home for a saved 20-DoF pose.

    Thread-safe: ``request_go_home`` may be called from HTTP or another thread;
    the owner loop (teleop) should call ``poll()`` each frame, or use
    ``go_home()`` when it exclusively owns the command channel.
    """

    def __init__(
        self,
        qpos: list[float],
        *,
        config_path: Optional[Path] = None,
        meta: Optional[dict] = None,
        fps: float = DEFAULT_FPS,
    ) -> None:
        if len(qpos) != TOTAL_JOINTS:
            raise ValueError(f"home qpos must have {TOTAL_JOINTS} values, got {len(qpos)}")
        self._qpos = [float(x) for x in qpos]
        self.config_path = config_path
        self.meta = meta or {}
        self.fps = float(fps)
        self._send: Optional[SendFn] = None
        self._lock = threading.Lock()
        self._pending: Optional[_GoHomeRequest] = None
        self._http: Optional[ThreadingHTTPServer] = None
        self._http_thread: Optional[threading.Thread] = None

    # ── construction ──────────────────────────────────────────────────────

    @classmethod
    def from_config(cls, path: Path | str, *, fps: float = DEFAULT_FPS) -> "HomePoseService":
        path = Path(path)
        data = json.loads(path.read_text(encoding="utf-8"))
        qpos = data.get("home_qpos")
        if not isinstance(qpos, list) or len(qpos) != TOTAL_JOINTS:
            raise ValueError(f"{path} must contain home_qpos with {TOTAL_JOINTS} floats")
        return cls(qpos, config_path=path, meta=data, fps=fps)

    @classmethod
    def try_from_config(
        cls, path: Path | str, *, fps: float = DEFAULT_FPS
    ) -> Optional["HomePoseService"]:
        path = Path(path)
        if not path.is_file():
            return None
        return cls.from_config(path, fps=fps)

    # ── properties ────────────────────────────────────────────────────────

    @property
    def ready(self) -> bool:
        return len(self._qpos) == TOTAL_JOINTS

    @property
    def qpos(self) -> list[float]:
        return list(self._qpos)

    def bind_sender(self, send: SendFn) -> None:
        """Attach the hand command callback used by go_home / hold."""
        self._send = send

    # ── core API ──────────────────────────────────────────────────────────

    def hold(self) -> None:
        """Send one home frame (for idle / footkey-off loops)."""
        send = self._require_send()
        send(self.qpos)

    def go_home(self, duration_s: float = 1.5, *, rate_hz: Optional[float] = None) -> None:
        """Block while streaming home for ``duration_s`` seconds."""
        send = self._require_send()
        hz = rate_hz or self.fps
        dt = 1.0 / max(hz, 1.0)
        end = time.monotonic() + max(duration_s, 0.0)
        q = self.qpos
        while time.monotonic() < end:
            send(q)
            time.sleep(dt)

    def request_go_home(self, duration_s: float = 1.5) -> threading.Event:
        """Non-blocking: queue a go-home for the owner loop's ``poll()``.

        Returns an Event that is set when the move finishes (or immediately if
        the owner never polls — caller should not wait forever without a loop).
        """
        event = threading.Event()
        with self._lock:
            self._pending = _GoHomeRequest(duration_s=float(duration_s), event=event)
        return event

    @property
    def has_pending(self) -> bool:
        with self._lock:
            return self._pending is not None

    def poll(self) -> bool:
        """Run from the teleop/command loop. Returns True if a go-home ran."""
        with self._lock:
            req = self._pending
            self._pending = None
        if req is None:
            return False
        try:
            self.go_home(req.duration_s)
        finally:
            req.event.set()
        return True

    def reload(self, path: Optional[Path] = None) -> None:
        """Reload home_qpos from JSON (default: original config_path)."""
        path = Path(path) if path is not None else self.config_path
        if path is None:
            raise ValueError("no config path to reload")
        other = HomePoseService.from_config(path, fps=self.fps)
        self._qpos = other._qpos
        self.config_path = other.config_path
        self.meta = other.meta

    # ── HTTP service ──────────────────────────────────────────────────────

    def serve_http(self, host: str = "127.0.0.1", port: int = 8765) -> None:
        """Start a background HTTP server in this process.

        Endpoints:
          GET  /home              → JSON home pose
          POST /go_home[?duration=1.5] → request_go_home (or go_home if no loop)
          POST /reload            → reload config file
        """
        if self._http is not None:
            return
        service = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt: str, *args) -> None:  # quieter
                print(f"[home-http] {self.address_string()} {fmt % args}")

            def _json(self, code: int, payload: dict) -> None:
                body = json.dumps(payload).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:  # noqa: N802
                path = urlparse(self.path).path
                if path in ("/home", "/"):
                    self._json(
                        200,
                        {
                            "ok": True,
                            "home_qpos": service.qpos,
                            "config_path": str(service.config_path)
                            if service.config_path
                            else None,
                            "meta": {
                                k: service.meta.get(k)
                                for k in ("device_sn", "device_type", "saved_at")
                            },
                        },
                    )
                    return
                self._json(404, {"ok": False, "error": "not found"})

            def do_POST(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                path = parsed.path
                qs = parse_qs(parsed.query)
                if path == "/go_home":
                    duration = float(qs.get("duration", ["1.5"])[0])
                    service.request_go_home(duration)
                    self._json(
                        202,
                        {
                            "ok": True,
                            "accepted": True,
                            "duration_s": duration,
                            "mode": "queued",
                        },
                    )
                    return
                if path == "/reload":
                    try:
                        service.reload()
                        self._json(200, {"ok": True, "home_qpos": service.qpos})
                    except Exception as exc:
                        self._json(500, {"ok": False, "error": str(exc)})
                    return
                self._json(404, {"ok": False, "error": "not found"})

        self._http = ThreadingHTTPServer((host, port), Handler)
        self._http_thread = threading.Thread(
            target=self._http.serve_forever, name="home-http", daemon=True
        )
        self._http_thread.start()
        print(f"HomePoseService HTTP on http://{host}:{port}  (POST /go_home)")

    def stop_http(self) -> None:
        if self._http is not None:
            with contextlib.suppress(Exception):
                self._http.shutdown()
            self._http = None
            self._http_thread = None

    # ── internals ─────────────────────────────────────────────────────────

    def _require_send(self) -> SendFn:
        if self._send is None:
            raise RuntimeError("HomePoseService has no sender; call bind_sender() first")
        return self._send


# ── save / read helpers (shared with 3.save_home.py) ──────────────────────


def save_home_config(
    path: Path,
    qpos: list[float],
    *,
    sn: str,
    device_type: str,
) -> None:
    payload = {
        "version": 1,
        "joint_order": "firmware finger-major: thumb,index,middle,ring,pinky × joint1..4",
        "unit": "rad",
        "device_sn": sn,
        "device_type": device_type,
        "saved_at": datetime.now(timezone.utc).isoformat(),
        "home_qpos": qpos,
        "fingers": {
            name: qpos[i * 4 : (i + 1) * 4] for i, name in enumerate(FINGER_NAMES)
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def flat_from_nid_frame(joints, total: int = TOTAL_JOINTS) -> list[float]:
    q = [0.0] * total
    filled = 0
    for j in joints:
        nid = int(j.nid)
        if 0 <= nid < total:
            q[nid] = float(j.position)
            filled += 1
        elif 1 <= nid <= total:
            q[nid - 1] = float(j.position)
            filled += 1
    if filled == 0:
        raise RuntimeError("joint_states frame had no usable nid/position entries")
    return q


def read_qpos_hand2(hand: WujiHand2, timeout_s: float = 3.0) -> list[float]:
    sub = hand.joint_states().subscribe()
    try:
        deadline = time.monotonic() + timeout_s
        best: Optional[list[float]] = None
        best_n = 0
        while time.monotonic() < deadline:
            frame = sub.recv()
            if frame is None:
                time.sleep(0.01)
                continue
            q = flat_from_nid_frame(frame.joints)
            n = len(frame.joints)
            if n >= best_n:
                best, best_n = q, n
            if n >= TOTAL_JOINTS:
                return q
        if best is None:
            raise TimeoutError(f"No joint_states within {timeout_s:.1f}s")
        print(f"Warning: only {best_n} joints in last frame; missing entries left as 0")
        return best
    finally:
        sub.close()


def read_qpos_hand1(hand: WujiHand) -> list[float]:
    state = hand.read_joint_state()
    pos = list(state.position)
    if len(pos) != TOTAL_JOINTS:
        raise RuntimeError(f"Expected {TOTAL_JOINTS} positions, got {len(pos)}")
    return [float(x) for x in pos]


def connect_hand(manager: SdkManager):
    hand_dev = None
    for d in manager.scan():
        print(f"  SN={d.sn}, Type={d.device_type}, Address={d.address}")
        if d.device_type in (DeviceType.WujiHand2, DeviceType.WujiHand):
            hand_dev = d
    if hand_dev is None:
        raise SystemExit("No Wuji Hand / Wuji Hand 2 found")
    return manager.connect(sn=hand_dev.sn, device_name=hand_dev.sn)


def _make_sender(hand):
    """Enable hand and return (send_fn, cleanup_cm_or_None)."""
    if isinstance(hand, WujiHand2):
        hand.effort_limit().set(1.5)
        hand.mit_params().set((3.0, 0.05))
        hand.enable()
        publisher = hand.joint_command().publish()

        def send(q: list[float]) -> None:
            publisher.send([JointCommand(p, 0.0, 0.0) for p in q])

        return send, None

    hand.set_all_effort_limit(1.5)
    hand.enable()
    ctrl_cm = hand.realtime_controller(LowPass(cutoff_hz=5.0))
    ctrl = ctrl_cm.__enter__()
    return ctrl.set_target_position, ctrl_cm


def main() -> int:
    parser = argparse.ArgumentParser(description="Home pose service / one-shot go-home")
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"home_pose.json path (default: {DEFAULT_CONFIG})",
    )
    parser.add_argument(
        "--go-home",
        action="store_true",
        help="Connect to hand and stream home once, then exit.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.5,
        help="go-home stream duration seconds (default 1.5).",
    )
    parser.add_argument(
        "--serve",
        action="store_true",
        help="Connect to hand and serve HTTP go_home until Ctrl+C.",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    if not args.go_home and not args.serve:
        parser.error("specify --go-home and/or --serve")

    if not args.config.is_file():
        raise SystemExit(f"Missing home config: {args.config} (run 3.save_home.py first)")

    svc = HomePoseService.from_config(args.config)
    manager = SdkManager.instance()
    hand = connect_hand(manager)
    send, ctrl_cm = _make_sender(hand)
    svc.bind_sender(send)

    try:
        if args.serve:
            svc.serve_http(host=args.host, port=args.port)
            print("Serving; POST /go_home or Ctrl+C to stop. Holding home while idle.")
            try:
                while True:
                    if not svc.poll():
                        svc.hold()
                    time.sleep(1.0 / DEFAULT_FPS)
            except KeyboardInterrupt:
                pass
        else:
            print(f"Going home for {args.duration:.2f}s ...")
            svc.go_home(args.duration)
            print("Done.")
    finally:
        svc.stop_http()
        if ctrl_cm is not None:
            with contextlib.suppress(Exception):
                ctrl_cm.__exit__(None, None, None)
        with contextlib.suppress(Exception):
            hand.disable()
        manager.disconnect_all()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
