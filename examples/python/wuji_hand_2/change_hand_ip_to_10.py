#!/usr/bin/env python3
"""Temporarily require PC on 192.168.1.x; rewrite Hand 2 static IPs to 192.168.10.x.

Factory defaults:
  left  (SN ...J...)  192.168.1.110
  right (SN ...K...)  192.168.1.111

After this script + reboot:
  left  192.168.10.110
  right 192.168.10.111

Usage (conda env wuji):
  # 1) put PC NIC on 192.168.1.100 first, then:
  conda run -n wuji python change_hand_ip_to_10.py
  # 2) put PC NIC back to 192.168.10.149 and ping the new IPs
"""

from __future__ import annotations

import time

from wuji_sdk import SdkManager

# Prefer SN (from scan). Factory IPs are fallback only.
TARGETS = [
    ("WH2JA01260812008", "192.168.10.110", "left"),
    ("WH2KA01260813040", "192.168.10.111", "right"),
]


def main() -> None:
    manager = SdkManager.instance()
    changed: list[tuple[str, str, str]] = []

    print("scan:")
    for d in manager.scan():
        print(f"  {d.sn}  {d.address}  {d.device_type}")

    try:
        for sn, new_ip, side in TARGETS:
            print(f"\n=== {side}: connect sn={sn} ===")
            hand = manager.connect(sn=sn, device_name=f"hand_{side}")
            cur = hand.ip().get()
            print(f"SN={hand.serial_number} current_ip={cur}")
            if cur == new_ip:
                print(f"already {new_ip}, skip set")
            else:
                hand.ip().set(new_ip)
                print(f"ip().set({new_ip!r}) written (takes effect after reboot)")
            changed.append((side, hand.serial_number, new_ip))
            hand.reboot()
            print(f"reboot issued for {side}")

        print("\nWaiting ~12s for reboots...")
        time.sleep(12)
    finally:
        manager.disconnect_all()

    print("\nDone. Switch PC NIC back to 192.168.10.149/24, then:")
    for side, sn, new_ip in changed:
        print(f"  ping {new_ip}   # {side} {sn}")
    print("Studio / SDK should then use 192.168.10.110 and 192.168.10.111")


if __name__ == "__main__":
    main()
