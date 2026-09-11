#!/usr/bin/env python3
"""ROS2 driver bridge for Wuji Hand 2 (Ethernet / wuji_sdk).

Mirrors the gen-1 ``wujihandros2`` topic shape, with a trailing ``2`` on the
hand namespace so both generations can coexist:

  Gen-1:  /hand_left/joint_commands   /hand_right/joint_commands
  Hand2:  /hand_left2/joint_commands  /hand_right2/joint_commands

  /control/footkey2   std_msgs/Bool   (True = accept commands; like Apex footkey)

Also publishes:
  /{hand_name}/joint_states          sensor_msgs/JointState
  /{hand_name}/tactile               std_msgs/Float32MultiArray
      5 fingers × [fx, fy, fz, temperature, contacts, max_force]
      finger order: thumb, index, middle, ring, pinky
  /{hand_name}/tactile/<finger>      std_msgs/Float32MultiArray
      per-point forces flat [fx,fy,fz] × N  (thumb ~40, others ~34)

Usage::

    source /opt/ros/humble/setup.bash
    python wujihand2_ros_driver.py --side both --no-footkey
    python wujihand2_ros_driver.py --side both --no-footkey --tactile-calibrate

    ros2 topic echo /hand_left2/tactile --once
    ros2 topic echo /hand_left2/tactile/index --once
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import struct
import threading
import time
from typing import Any, Callable, Optional

import numpy as np

from wuji_sdk import DeviceType, JointCommand, SdkManager, WujiHand2

TOTAL_JOINTS = 20
DEFAULT_HAND_NAME = {"left": "hand_left2", "right": "hand_right2"}
HAND2_KP = 5.0
HAND2_KD = 0.15
HAND2_EFFORT_LIMIT = 1.5
HAND2_QPOS_EMA = 0.35
FOOTKEY_TOPIC = "/control/footkey2"
STATE_HZ = 50.0
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
CONTACT_N = 0.2  # newtons; |F| above this counts as contact
FIELD_FMT = {
    "i8": "<b",
    "u8": "<B",
    "i16": "<h",
    "u16": "<H",
    "i32": "<i",
    "u32": "<I",
    "f32": "<f",
}
SUMMARY_FIELDS = 6  # fx fy fz temp contacts max_force per finger


def nid_to_flat(nid: int) -> Optional[int]:
    """Map Hand2 bus nid → flat firmware index 0..19."""
    bus, node_index = divmod(int(nid) - 1, 5)
    if 0 <= bus < 5 and 0 <= node_index < 4:
        return bus * 4 + node_index
    return None


def parse_side_label(value: object) -> Optional[str]:
    text = str(value).strip().lower()
    if "left" in text:
        return "left"
    if "right" in text:
        return "right"
    return None


def wait_enabled(hand: WujiHand2, timeout_s: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_s
    sub = hand.joint_diagnostics().subscribe()
    try:
        while time.monotonic() < deadline:
            time.sleep(0.05)
            frame = sub.recv()
            if frame is None or not frame.joints:
                continue
            if all(e.status_word.ext_state == 2 for e in frame.joints):
                return True
    finally:
        sub.close()
    return False


def _make_fingertip_decoder(fmt: dict[str, Any]) -> Callable[[bytes], tuple[list[dict], dict]]:
    pc, stride = fmt["point_count"], fmt["point_stride"]
    expect = pc * stride + fmt["aggregate_stride"]

    def read(defs: list[dict], data: bytes, base: int) -> dict[str, float]:
        out: dict[str, float] = {}
        for d in defs:
            raw = struct.unpack_from(FIELD_FMT[d["type"]], data, base + d["offset"])[0]
            out[d["name"]] = float(raw) * float(d.get("scale", 1.0))
        return out

    def decode(data: bytes) -> tuple[list[dict], dict]:
        if len(data) != expect:
            raise ValueError(f"data length {len(data)} != expected {expect}")
        points = [read(fmt["point_fields"], data, k * stride) for k in range(pc)]
        agg = read(fmt["aggregate_fields"], data, pc * stride)
        return points, agg

    return decode


def _point_force(p: dict[str, float]) -> float:
    return math.sqrt(p.get("fx", 0.0) ** 2 + p.get("fy", 0.0) ** 2 + p.get("fz", 0.0) ** 2)


class Hand2Slot:
    """One Hand 2 + ROS pubs/subs under /{hand_name}/…"""

    def __init__(
        self,
        node: Any,
        hand: WujiHand2,
        hand_name: str,
        JointState: Any,
        Float32MultiArray: Any,
        MultiArrayDimension: Any,
        qos: Any,
        accept_fn: Any,
        *,
        enable_tactile: bool = True,
        tactile_calibrate: bool = False,
    ) -> None:
        self.hand = hand
        self.hand_name = hand_name
        self._JointState = JointState
        self._Float32MultiArray = Float32MultiArray
        self._MultiArrayDimension = MultiArrayDimension
        self._accept_fn = accept_fn
        self._pub = hand.joint_command().publish()
        self._cmds = [JointCommand(0.0, 0.0, 0.0) for _ in range(TOTAL_JOINTS)]
        self._q_filt: Optional[np.ndarray] = None
        self._last_cmd: Optional[list[float]] = None
        self._lock = threading.Lock()
        self._motors_on = False
        self._tactile_ok = False
        self._decoders: dict[str, Callable] = {}
        self._tactile_subs: dict[str, Any] = {}
        self._tactile_finger_pubs: dict[str, Any] = {}
        self._tactile_summary_pub = None

        with contextlib.suppress(Exception):
            hand.clear_fault()
        hand.effort_limit().set(HAND2_EFFORT_LIMIT)
        hand.mit_params().set((HAND2_KP, HAND2_KD))
        hand.enable()
        if not wait_enabled(hand):
            node.get_logger().warn(f"[{hand_name}] enable timeout — may stutter")
        self._motors_on = True

        self.state_pub = node.create_publisher(
            JointState, f"/{hand_name}/joint_states", qos
        )
        self.cmd_sub = node.create_subscription(
            JointState,
            f"/{hand_name}/joint_commands",
            self._on_cmd,
            qos,
        )
        self._state_sub = hand.joint_states().subscribe()
        node.get_logger().info(
            f"Hand2 ROS: SN={hand.serial_number} "
            f"cmd=/{hand_name}/joint_commands "
            f"state=/{hand_name}/joint_states"
        )

        if enable_tactile:
            self._setup_tactile(node, qos, tactile_calibrate)

    def _setup_tactile(self, node: Any, qos: Any, calibrate: bool) -> None:
        if calibrate:
            try:
                self.hand.tactile_calibrate()
                node.get_logger().info(
                    f"[{self.hand_name}] tactile_calibrate() issued (keep fingertips unloaded)"
                )
                time.sleep(0.5)
            except Exception as exc:
                node.get_logger().warn(f"[{self.hand_name}] tactile_calibrate failed: {exc}")

        accessors = {
            "thumb": self.hand.fingertip_thumb_data,
            "index": self.hand.fingertip_index_data,
            "middle": self.hand.fingertip_middle_data,
            "ring": self.hand.fingertip_ring_data,
            "pinky": self.hand.fingertip_pinky_data,
        }
        for i, name in enumerate(FINGERS):
            try:
                info = self.hand.get_fingertip_info(i)
                fmt = json.loads(info.format)
                if fmt.get("v") != 1 or fmt.get("encoding") != "point_array":
                    raise ValueError(f"unsupported format {fmt}")
                self._decoders[name] = _make_fingertip_decoder(fmt)
                self._tactile_subs[name] = accessors[name]().subscribe()
                self._tactile_finger_pubs[name] = node.create_publisher(
                    self._Float32MultiArray,
                    f"/{self.hand_name}/tactile/{name}",
                    qos,
                )
                rate = getattr(info, "rate_hz", "?")
                node.get_logger().info(
                    f"[{self.hand_name}] tactile/{name}: "
                    f"{fmt['point_count']} pts @ ~{rate} Hz"
                )
            except Exception as exc:
                node.get_logger().warn(
                    f"[{self.hand_name}] no tactile on {name}: {exc}"
                )

        if self._tactile_subs:
            self._tactile_summary_pub = node.create_publisher(
                self._Float32MultiArray, f"/{self.hand_name}/tactile", qos
            )
            self._tactile_ok = True
            node.get_logger().info(
                f"[{self.hand_name}] tactile summary → /{self.hand_name}/tactile "
                f"(5×[fx,fy,fz,temp,contacts,max_force])"
            )
        else:
            node.get_logger().warn(f"[{self.hand_name}] tactile disabled (no sensors)")

    def _on_cmd(self, msg: Any) -> None:
        if not self._motors_on:
            return
        if not self._accept_fn():
            return
        if len(msg.position) < TOTAL_JOINTS:
            return
        q = [float(x) for x in msg.position[:TOTAL_JOINTS]]
        with self._lock:
            self._last_cmd = q
            self._send_locked(q)

    def _send_locked(self, q: list[float]) -> None:
        arr = np.asarray(q, dtype=np.float32)
        if self._q_filt is None:
            self._q_filt = arr.copy()
        else:
            self._q_filt = (1.0 - HAND2_QPOS_EMA) * self._q_filt + HAND2_QPOS_EMA * arr
        for i, p in enumerate(self._q_filt.tolist()):
            self._cmds[i] = JointCommand(float(p), 0.0, 0.0)
        self._pub.send(self._cmds)

    def hold_last(self) -> None:
        with self._lock:
            if self._last_cmd is not None:
                self._send_locked(self._last_cmd)

    def publish_state(self, stamp: Any) -> None:
        latest = None
        while True:
            frame = self._state_sub.recv()
            if frame is None:
                break
            latest = frame
        if latest is None or not latest.joints:
            return
        pos = [0.0] * TOTAL_JOINTS
        vel = [0.0] * TOTAL_JOINTS
        eff = [0.0] * TOTAL_JOINTS
        for j in latest.joints:
            idx = nid_to_flat(j.nid)
            if idx is None:
                continue
            pos[idx] = float(j.position)
            vel[idx] = float(j.velocity)
            eff[idx] = float(j.effort)
        msg = self._JointState()
        msg.header.stamp = stamp
        msg.name = [f"j{i}" for i in range(TOTAL_JOINTS)]
        msg.position = pos
        msg.velocity = vel
        msg.effort = eff
        self.state_pub.publish(msg)

    def publish_tactile(self) -> None:
        if not self._tactile_ok:
            return
        summary = [0.0] * (len(FINGERS) * SUMMARY_FIELDS)
        for fi, name in enumerate(FINGERS):
            sub = self._tactile_subs.get(name)
            decode = self._decoders.get(name)
            pub = self._tactile_finger_pubs.get(name)
            if sub is None or decode is None or pub is None:
                continue
            latest = None
            while True:
                frame = sub.recv()
                if frame is None:
                    break
                latest = frame
            if latest is None:
                continue
            try:
                points, agg = decode(bytes(latest.data))
            except Exception:
                continue
            forces = [_point_force(p) for p in points]
            contacts = float(sum(1 for f in forces if f > CONTACT_N))
            max_f = float(max(forces) if forces else 0.0)
            base = fi * SUMMARY_FIELDS
            summary[base : base + SUMMARY_FIELDS] = [
                float(agg.get("fx", 0.0)),
                float(agg.get("fy", 0.0)),
                float(agg.get("fz", 0.0)),
                float(agg.get("temperature", 0.0)),
                contacts,
                max_f,
            ]
            flat: list[float] = []
            for p in points:
                flat.extend(
                    [
                        float(p.get("fx", 0.0)),
                        float(p.get("fy", 0.0)),
                        float(p.get("fz", 0.0)),
                    ]
                )
            pt_msg = self._Float32MultiArray()
            pt_msg.layout.dim = [
                self._MultiArrayDimension(
                    label=f"{name}_points", size=len(points), stride=3 * len(points)
                ),
                self._MultiArrayDimension(label="xyz", size=3, stride=3),
            ]
            pt_msg.data = flat
            pub.publish(pt_msg)

        if self._tactile_summary_pub is not None:
            s_msg = self._Float32MultiArray()
            s_msg.layout.dim = [
                self._MultiArrayDimension(
                    label="fingers",
                    size=len(FINGERS),
                    stride=SUMMARY_FIELDS * len(FINGERS),
                ),
                self._MultiArrayDimension(
                    label="fx_fy_fz_temp_contacts_maxforce",
                    size=SUMMARY_FIELDS,
                    stride=SUMMARY_FIELDS,
                ),
            ]
            s_msg.data = summary
            self._tactile_summary_pub.publish(s_msg)

    def close(self) -> None:
        for sub in self._tactile_subs.values():
            with contextlib.suppress(Exception):
                sub.close()
        with contextlib.suppress(Exception):
            self._state_sub.close()
        with contextlib.suppress(Exception):
            self._pub.close()
        with contextlib.suppress(Exception):
            self.hand.disable()


class WujiHand2RosDriver:
    def __init__(
        self,
        sides: list[str],
        hand_names: dict[str, str],
        *,
        require_footkey: bool = True,
        sn_filter: Optional[dict[str, str]] = None,
        enable_tactile: bool = True,
        tactile_calibrate: bool = False,
    ) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension

        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self._node = Node("wujihand2_ros_driver")
        self._require_footkey = require_footkey
        self._footkey = not require_footkey
        self._slots: dict[str, Hand2Slot] = {}
        self._manager = SdkManager.instance()

        by_side: dict[str, WujiHand2] = {}
        for d in self._manager.scan():
            if d.device_type != DeviceType.WujiHand2:
                continue
            if sn_filter:
                wanted = set(sn_filter.values())
                if d.sn not in wanted:
                    continue
            hand = self._manager.connect(sn=d.sn, device_name=f"ros2_{d.sn}")
            side = parse_side_label(hand.handedness().get())
            if side is None:
                self._node.get_logger().warn(f"Skip {d.sn}: unknown handedness")
                continue
            if sn_filter and sn_filter.get(side) and sn_filter[side] != d.sn:
                continue
            by_side[side] = hand

        for side in sides:
            hand = by_side.get(side)
            if hand is None:
                self._node.get_logger().error(f"No Wuji Hand 2 for side={side}")
                continue
            name = hand_names[side]
            self._slots[name] = Hand2Slot(
                self._node,
                hand,
                name,
                JointState,
                Float32MultiArray,
                MultiArrayDimension,
                qos_profile_sensor_data,
                accept_fn=lambda: self._footkey if self._require_footkey else True,
                enable_tactile=enable_tactile,
                tactile_calibrate=tactile_calibrate,
            )

        if not self._slots:
            raise SystemExit("No Hand 2 connected — check Ethernet / IP / power")

        self._node.create_subscription(Bool, FOOTKEY_TOPIC, self._on_footkey, 10)
        self._node.get_logger().info(
            f"Footkey: {FOOTKEY_TOPIC} (require={require_footkey}, "
            f"initial_accept={self._footkey})"
        )

        period = 1.0 / STATE_HZ
        self._node.create_timer(period, self._on_timer)
        self._last_gate = self._footkey

    def _on_footkey(self, msg: Any) -> None:
        self._footkey = bool(msg.data)
        if self._footkey != self._last_gate:
            self._last_gate = self._footkey
            self._node.get_logger().info(f"footkey2 = {self._footkey}")

    def _on_timer(self) -> None:
        stamp = self._node.get_clock().now().to_msg()
        accept = self._footkey if self._require_footkey else True
        for slot in self._slots.values():
            if not accept:
                slot.hold_last()
            slot.publish_state(stamp)
            slot.publish_tactile()

    def spin(self) -> None:
        try:
            self._rclpy.spin(self._node)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def close(self) -> None:
        for slot in self._slots.values():
            slot.close()
        with contextlib.suppress(Exception):
            self._manager.disconnect_all()
        with contextlib.suppress(Exception):
            self._node.destroy_node()
        with contextlib.suppress(Exception):
            if self._rclpy.ok():
                self._rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Wuji Hand 2 ROS2 driver (topics *2 + tactile)")
    p.add_argument(
        "--side",
        choices=("left", "right", "both"),
        default="both",
        help="Which Hand 2 to drive (default both).",
    )
    p.add_argument(
        "--hand-name",
        default=None,
        help="Override namespace when --side is left or right (default hand_*2).",
    )
    p.add_argument(
        "--no-footkey",
        action="store_true",
        help="Accept joint_commands without /control/footkey2 == true.",
    )
    p.add_argument(
        "--no-tactile",
        action="store_true",
        help="Do not publish fingertip tactile topics.",
    )
    p.add_argument(
        "--tactile-calibrate",
        action="store_true",
        help="Call tactile_calibrate() on connect (fingertips must be unloaded).",
    )
    p.add_argument("--left-sn", default=None, help="Optional left Hand 2 SN filter.")
    p.add_argument("--right-sn", default=None, help="Optional right Hand 2 SN filter.")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    sides = ["left", "right"] if args.side == "both" else [args.side]
    hand_names = dict(DEFAULT_HAND_NAME)
    if args.hand_name:
        if args.side == "both":
            print("Ignoring --hand-name with --side both")
        else:
            hand_names[args.side] = args.hand_name

    sn_filter: Optional[dict[str, str]] = None
    if args.left_sn or args.right_sn:
        sn_filter = {}
        if args.left_sn:
            sn_filter["left"] = args.left_sn
        if args.right_sn:
            sn_filter["right"] = args.right_sn

    driver = WujiHand2RosDriver(
        sides,
        hand_names,
        require_footkey=not args.no_footkey,
        sn_filter=sn_filter,
        enable_tactile=not args.no_tactile,
        tactile_calibrate=args.tactile_calibrate,
    )
    driver.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
