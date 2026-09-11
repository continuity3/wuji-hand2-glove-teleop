#!/usr/bin/env python3
"""
Retargeting example - live teleoperation from a Wuji Glove.

The SDK exposes only the pure retarget interface (``RetargetSession``). Teleop —
read keypoints → retarget → drive the hand — is plain example code you can adapt,
shown end to end below: read the glove's hand keypoints, retarget each frame, and
drive a connected Wuji Hand or Wuji Hand 2.

To drive from a different keypoint source (camera / MediaPipe / VR), replace the
glove read with your own: ``RetargetSession.step`` accepts any ``(21, 3)`` float32
array of MediaPipe-format landmarks and returns a 20-value joint command in
firmware order, so the result is sent to the hand as-is.

The session is built with ``RetargetSession.for_hand(model, side=SIDE)`` — the hand
model selects the builtin tuning config internally, so there is no config path to
manage. Configuring and enabling the hand's motors is the caller's responsibility,
shown here in configure_wuji_hand_2 / configure_wuji_hand.

This example pairs each glove with the matching-handedness Wuji Hand and
teleoperates every pair at once (left glove → left hand, right glove → right
hand). A single pair still works if only one side is connected.

This example runs the **glove** on the SDK built-in default hand URDF: it
switches to the default SDK user before connecting — the default user runs the
glove on the built-in default hand URDF — and restores the previously selected
user on exit. See ``use_builtin_urdf_user``. Use a named SDK user to run with a
calibrated hand model: create one, switch to it, then calibrate the glove under
it — calibrating under the default user has no effect, since the default user
always runs on the built-in URDF.

Install the retargeting runtime dependencies first (numpy for keypoint/qpos arrays):

    pip install wuji-sdk numpy

Usage:
    python 1.teleop_real.py
"""

import contextlib
import threading
import time

import numpy as np

from wuji_sdk import (
    DeviceType,
    HandModel,
    Handedness,
    JointCommand,
    LowPass,
    RetargetSession,
    SdkManager,
    WujiHand2,
)

FPS = 120  # target loop rate (ceiling); a slow frame lowers the rate, never bursts.
# Hand 2 has no realtime LowPass like gen-1; EMA on qpos reduces command chatter.
HAND2_QPOS_EMA = 0.35
HAND2_KP = 5.0
HAND2_KD = 0.15
HAND2_EFFORT_LIMIT = 1.5  # Amps


def wait_hand2_enabled(hand, timeout_s: float = 5.0) -> bool:
    """Block until all online joints report Enabled (ext_state==2)."""
    deadline = time.monotonic() + timeout_s
    diag_sub = hand.joint_diagnostics().subscribe()
    try:
        while time.monotonic() < deadline:
            time.sleep(0.05)
            frame = diag_sub.recv()
            if frame is None or not frame.joints:
                continue
            if all(e.status_word.ext_state == 2 for e in frame.joints):
                return True
    finally:
        diag_sub.close()
    return False


def configure_wuji_hand_2(hand):
    """Configure + enable a Wuji Hand 2; return the retarget HandModel."""
    # MIT impedance: hold each joint at its commanded position. The
    # firmware defaults to MIT control mode (control mode is not set from Python).
    with contextlib.suppress(Exception):
        hand.clear_fault()
    hand.effort_limit().set(HAND2_EFFORT_LIMIT)
    hand.mit_params().set((HAND2_KP, HAND2_KD))  # (kp, kd), broadcast to all joints
    hand.enable()
    if not wait_hand2_enabled(hand):
        print(
            f"[{hand.serial_number}] enable timeout — "
            "motors not all Enabled; motion may stutter"
        )
    return HandModel.WujiHand2


def configure_wuji_hand(hand):
    """Configure + enable a first-gen Wuji Hand; return the retarget HandModel."""
    hand.set_all_effort_limit(1.5)  # Amps
    hand.enable()
    return HandModel.WujiHand


def use_builtin_urdf_user(manager):
    """Switch to the default SDK user so the glove uses the built-in URDF.

    The default SDK user runs the glove on the built-in default hand URDF rather
    than a per-user IK calibration. Returns the previously selected user so
    main() can restore it on exit.
    """
    previous = manager.current_user()
    manager.switch_to_default_user()
    return previous


def read_keypoints(skeleton_sub):
    """Read the glove's latest hand_skeleton frame as a (21, 3) float32 array.

    The glove publishes hand_skeleton faster than this loop consumes it, and
    recv() returns the OLDEST unread frame — taking one per loop falls behind.
    Drain every queued frame, keep only the latest. Returns None if no frame is
    available this tick.
    """
    latest = None
    while True:
        frame = skeleton_sub.recv()
        if frame is None:
            break
        latest = frame
    if latest is None:
        return None
    return np.array([j.pose.position for j in latest.joints], dtype=np.float32)


def parse_side(value) -> str | None:
    """Map glove/hand side labels to 'left' or 'right'."""
    text = str(value).strip().lower()
    if "left" in text:
        return "left"
    if "right" in text:
        return "right"
    return None


def hand_side(hand) -> str | None:
    """Wuji Hand 2: handedness().get(); gen-1 Hand: handedness_name()."""
    if isinstance(hand, WujiHand2):
        return parse_side(hand.handedness().get())
    return parse_side(hand.handedness_name())


def side_enum(side: str) -> Handedness:
    return Handedness.Left if side == "left" else Handedness.Right


def teleop(glove, model, send, side, stop, smooth_qpos: bool = False):
    """Build the retargeting session + glove source, then loop read → retarget → send.

    Called *after* the hand's command channel is already open (see run_pair):
    open the command channel before session initialization so device
    communication remains active during setup.
    """
    session = RetargetSession.for_hand(model, side=side)
    skeleton_sub = glove.hand_skeleton().subscribe()

    budget = 1.0 / FPS
    last_kp = None
    q_filt = None
    n = 0
    t0 = time.monotonic()
    while not stop.is_set():
        frame_start = time.monotonic()
        kp = read_keypoints(skeleton_sub)
        if kp is None:
            kp = last_kp  # no fresh frame: hold the last pose
            if kp is None:
                time.sleep(budget)  # nothing yet — wait out the budget
                continue
        else:
            last_kp = kp
        qpos = session.step(kp)  # (20,) firmware order
        if smooth_qpos:
            if q_filt is None:
                q_filt = qpos.copy()
            else:
                q_filt = (1.0 - HAND2_QPOS_EMA) * q_filt + HAND2_QPOS_EMA * qpos
            send(q_filt.tolist())
        else:
            send(qpos.tolist())
        n += 1
        if n % 240 == 0:
            hz = n / max(time.monotonic() - t0, 1e-6)
            print(f"[{side}] loop ~{hz:.0f} Hz", flush=True)
        dt = time.monotonic() - frame_start
        if dt < budget:
            time.sleep(budget - dt)


def run_pair(glove, hand, side, stop):
    """Enable one hand and stream glove keypoints into it until ``stop`` is set."""
    is_hand2 = isinstance(hand, WujiHand2)
    model = configure_wuji_hand_2(hand) if is_hand2 else configure_wuji_hand(hand)
    print(f"[{side}] glove={glove.serial_number} hand={hand.serial_number}")

    # Open the hand's command channel first — before teleop() builds the
    # retargeting session — so device communication remains active during setup.
    try:
        if is_hand2:
            publisher = hand.joint_command().publish()
            cmds = [JointCommand(0.0, 0.0, 0.0) for _ in range(20)]

            def send_hand2(q):
                for i, p in enumerate(q):
                    cmds[i] = JointCommand(p, 0.0, 0.0)
                publisher.send(cmds)

            try:
                teleop(
                    glove,
                    model,
                    send_hand2,
                    side_enum(side),
                    stop,
                    smooth_qpos=True,
                )
            finally:
                publisher.close()
        else:
            with hand.realtime_controller(LowPass(cutoff_hz=5.0)) as controller:
                teleop(
                    glove,
                    model,
                    controller.set_target_position,
                    side_enum(side),
                    stop,
                )
    finally:
        with contextlib.suppress(Exception):
            hand.disable()


def run_teleop(manager) -> int:
    """Connect matching glove+hand pairs and teleoperate until interrupted."""

    glove_devs = []
    hand_devs = []
    for d in manager.scan():
        print(f"  SN={d.sn}, Type={d.device_type}, Address={d.address}")
        if d.device_type in (DeviceType.WujiHand2, DeviceType.WujiHand):
            hand_devs.append(d)
        elif d.device_type == DeviceType.WujiGlove:
            glove_devs.append(d)

    if not glove_devs:
        print("No Wuji Glove found")
        return 1
    if not hand_devs:
        print("No Wuji Hand / Wuji Hand 2 found")
        return 1

    gloves_by_side: dict[str, object] = {}
    for d in glove_devs:
        glove = manager.connect(sn=d.sn, device_name=f"glove_{d.sn}")
        side = parse_side(glove.hand_side().get())
        if side is None:
            print(f"Skip glove {d.sn}: unknown hand_side")
            continue
        gloves_by_side[side] = glove

    hands_by_side: dict[str, object] = {}
    for d in hand_devs:
        hand = manager.connect(sn=d.sn, device_name=f"hand_{d.sn}")
        side = hand_side(hand)
        if side is None:
            print(f"Skip hand {d.sn}: unknown handedness")
            continue
        hands_by_side[side] = hand

    pairs = []
    for side in ("left", "right"):
        glove = gloves_by_side.get(side)
        hand = hands_by_side.get(side)
        if glove is not None and hand is not None:
            pairs.append((glove, hand, side))
        elif glove is not None or hand is not None:
            missing = "hand" if glove is not None else "glove"
            print(f"Skip {side}: no matching {missing}")

    if not pairs:
        print("No matching glove+hand pairs (left/right)")
        manager.disconnect_all()
        return 1

    stop = threading.Event()
    errors: list[BaseException] = []

    def worker(glove, hand, side):
        try:
            run_pair(glove, hand, side, stop)
        except Exception as exc:
            errors.append(exc)
            stop.set()

    print(f"Teleoperating {len(pairs)} pair(s) (Ctrl+C to stop)...")
    threads = [
        threading.Thread(target=worker, args=pair, daemon=True) for pair in pairs
    ]
    try:
        for t in threads:
            t.start()
        while any(t.is_alive() for t in threads):
            for t in threads:
                t.join(timeout=0.2)
    except KeyboardInterrupt:
        stop.set()
        for t in threads:
            t.join(timeout=2.0)
    finally:
        stop.set()
        manager.disconnect_all()

    if errors:
        raise errors[0]
    return 0


def main() -> int:
    manager = SdkManager.instance()

    # Run the glove on the SDK built-in default hand URDF: the default SDK user
    # uses the built-in URDF, so switch to it before connecting and restore the
    # previous user on exit. Use a named SDK user to run with a calibrated hand
    # model.
    previous_user = use_builtin_urdf_user(manager)
    exit_code = 0
    try:
        exit_code = run_teleop(manager)
    finally:
        try:
            manager.switch_user(previous_user["user_id"])
        except Exception as exc:
            print(f"Failed to restore previous SDK user: {exc}")
            exit_code = 1

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
