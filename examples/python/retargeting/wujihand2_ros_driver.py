#!/usr/bin/env python3
"""ROS2 driver bridge for Wuji Hand 2 (Ethernet / wuji_sdk).

Mirrors the gen-1 ``wujihandros2`` topic shape, with a trailing ``2`` on the
hand namespace so both generations can coexist:

  Gen-1:  /hand_left/joint_commands   /hand_right/joint_commands
  Hand2:  /hand_left2/joint_commands  /hand_right2/joint_commands

  /control/footkey2   std_msgs/Bool   (True = accept commands; like Apex footkey)

Also publishes:
  /{hand_name}/joint_states   sensor_msgs/JointState

Usage::

    source /opt/ros/humble/setup.bash
    # both hands (default namespaces hand_left2 / hand_right2)
    python wujihand2_ros_driver.py --side both

    # one side
    python wujihand2_ros_driver.py --side left
    python wujihand2_ros_driver.py --side right --hand-name hand_right2

Then run teleop with ROS sink::

    python 2.teleop_tuned.py --drive ros --hand-model wujihand2 --no-footkey
"""

from __future__ import annotations

import argparse
import contextlib
import threading
import time
from typing import Any, Optional

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


def nid_to_flat(nid: int) -> Optional[int]:
    """Map Hand2 bus nid → flat firmware index 0..19.

    Official layout: nid uses groups of five slots per finger bus; only the
    first four are joints (see Wuji Hand 2 Quick Start)::

        bus, node_index = divmod(nid - 1, 5)
        flat = bus * 4 + node_index   # if node_index < 4
    """
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


class Hand2Slot:
    """One Hand 2 + ROS pubs/subs under /{hand_name}/…"""

    def __init__(
        self,
        node: Any,
        hand: WujiHand2,
        hand_name: str,
        JointState: Any,
        qos: Any,
        accept_fn: Any,
    ) -> None:
        self.hand = hand
        self.hand_name = hand_name
        self._JointState = JointState
        self._accept_fn = accept_fn
        self._pub = hand.joint_command().publish()
        self._cmds = [JointCommand(0.0, 0.0, 0.0) for _ in range(TOTAL_JOINTS)]
        self._q_filt: Optional[np.ndarray] = None
        self._last_cmd: Optional[list[float]] = None
        self._lock = threading.Lock()
        self._motors_on = False

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
        # Map bus nid → flat 0..19 (NOT pos[nid] — nids are 1..4,6..9,...,21..24)
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

    def close(self) -> None:
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
    ) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self._node = Node("wujihand2_ros_driver")
        self._require_footkey = require_footkey
        self._footkey = not require_footkey
        self._slots: dict[str, Hand2Slot] = {}
        self._manager = SdkManager.instance()

        # Discover / connect Hand2
        by_side: dict[str, WujiHand2] = {}
        for d in self._manager.scan():
            if d.device_type != DeviceType.WujiHand2:
                continue
            if sn_filter:
                # optional: only connect listed SNs
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
                qos_profile_sensor_data,
                accept_fn=lambda: self._footkey if self._require_footkey else True,
            )

        if not self._slots:
            raise SystemExit("No Hand 2 connected — check Ethernet / IP / power")

        self._node.create_subscription(
            Bool, FOOTKEY_TOPIC, self._on_footkey, 10
        )
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
    p = argparse.ArgumentParser(description="Wuji Hand 2 ROS2 driver (topics *2)")
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
    )
    driver.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
