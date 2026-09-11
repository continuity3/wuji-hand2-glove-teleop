#!/usr/bin/env python3
"""
Capture the connected Wuji Hand / Hand 2 actual joint positions and save them
as a home pose config (JSON) for ``HomePoseService`` / ``2.teleop_tuned.py``.

Pose the hand (or hold a teleop pose), then:

    python 3.save_home.py
    python 3.save_home.py --out home_pose.json

Then return to that pose via the service:

    python home_pose_service.py --go-home
    # or from teleop / HTTP: see home_pose_service.py docstring
"""

from __future__ import annotations

import argparse
from pathlib import Path

from wuji_sdk import SdkManager, WujiHand2

from home_pose_service import (
    DEFAULT_CONFIG,
    FINGER_NAMES,
    connect_hand,
    read_qpos_hand1,
    read_qpos_hand2,
    save_home_config,
)


def print_qpos(qpos: list[float]) -> None:
    for i, name in enumerate(FINGER_NAMES):
        vals = qpos[i * 4 : (i + 1) * 4]
        print(f"  {name:6s}: {[f'{v:+.4f}' for v in vals]}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Save current hand pose as home_pose.json")
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"Output JSON path (default: {DEFAULT_CONFIG})",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="Seconds to wait for Hand 2 joint_states (default 3).",
    )
    args = parser.parse_args()

    manager = SdkManager.instance()
    hand = connect_hand(manager)
    try:
        if isinstance(hand, WujiHand2):
            print(f"Connected Hand 2 SN={hand.serial_number}; reading joint_states...")
            qpos = read_qpos_hand2(hand, timeout_s=args.timeout)
            dtype = "WujiHand2"
        else:
            print(f"Connected Hand SN={hand.serial_number}; reading joint state...")
            qpos = read_qpos_hand1(hand)
            dtype = "WujiHand"

        print("Current qpos (rad):")
        print_qpos(qpos)
        save_home_config(args.out, qpos, sn=str(hand.serial_number), device_type=dtype)
        print(f"Saved home → {args.out.resolve()}")
    finally:
        manager.disconnect_all()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
