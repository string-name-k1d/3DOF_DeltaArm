#!/usr/bin/env python3
"""3-D joint-response check for a given assembly.

    python3 gz_check.py <band_hi_deg> <label> <reachable x y z> <refused x y z>

Kept parameterised because the two assemblies resolve different bands and the
interesting poses differ (default reaches ~99.7 deg of arm, gen0 ~101.1 deg but
only from ~0 deg up, and the gen0 linkage is the mirrored horn/rod pair).
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from arm.action import SetPosition
from arm.msg import ArmPosition
from sensor_msgs.msg import JointState

JOINTS = ["shoulder_0", "shoulder_1", "shoulder_2"]


def dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


class C(Node):
    def __init__(self):
        super().__init__("gz_check2")
        self.js = None
        self.pos = None
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.create_subscription(ArmPosition, "/arm/pos", self._pos, 10)
        self.ac = ActionClient(self, SetPosition, "/arm/set_pos")

    def _js(self, m):
        self.js = m

    def _pos(self, m):
        self.pos = m

    def deg(self):
        if self.js is None:
            return None
        d = {}
        for nm in JOINTS:
            if nm in self.js.name:
                d[nm] = math.degrees(self.js.position[self.js.name.index(nm)])
        return d if len(d) == 3 else None

    def send(self, xyz):
        g = SetPosition.Goal()
        g.target.linear.x, g.target.linear.y, g.target.linear.z = xyz
        f = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        gh = f.result()
        if gh is None or not gh.accepted:
            return "NO_ADMIT", ""
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=10.0)
        r = rf.result()
        if r is None:
            return "NO_RESULT", ""
        return ("OK" if r.result.succeeded else "REFUSED"), r.result.message

    def settle(self, secs=12.0):
        end = time.time() + secs
        last = None
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            s = self.deg()
            if s:
                last = s
        return last


def main():
    band_hi = float(sys.argv[1])
    label = sys.argv[2]
    ok_xyz = tuple(float(v) for v in sys.argv[3:6])
    no_xyz = tuple(float(v) for v in sys.argv[6:9])

    rclpy.init()
    n = C()
    if not n.ac.wait_for_server(timeout_sec=20.0):
        print("FAIL: no action server")
        return 1
    end = time.time() + 20.0
    while n.deg() is None and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.3)
    if n.deg() is None:
        print("FAIL: no /joint_states from the gz bridge")
        return 1
    print(f"[{label}] shoulders at rest (deg): " +
          ", ".join(f"{k}={v:.2f}" for k, v in sorted(n.deg().items())))

    fails = []

    st, msg = n.send(ok_xyz)
    j1 = n.settle()
    print(f"\n[{label}] reachable {ok_xyz} -> {st}")
    if st != "OK":
        fails.append(f"expected success, got {st}: {msg}")
    if j1:
        print("  shoulders (deg): " + ", ".join(f"{k}={v:.2f}" for k, v in sorted(j1.items())))
        if max(j1.values()) <= 1.0:
            fails.append("joints did not move for a reachable goal")
        if max(j1.values()) > band_hi + 0.5:
            fails.append(f"joint {max(j1.values()):.1f} deg exceeds the {band_hi} deg band")
    else:
        fails.append("lost /joint_states")

    st2, msg2 = n.send(no_xyz)
    j2 = n.settle()
    print(f"\n[{label}] unreachable {no_xyz} -> {st2}")
    print(f"  reason: {msg2[:76]}")
    if st2 != "REFUSED":
        fails.append(f"expected a refusal, got {st2}")
    if j1 and j2:
        drift = max(abs(j2[k] - j1[k]) for k in j1)
        print(f"  max joint drift after the refusal: {drift:.2f} deg")
        if drift > 2.0:
            fails.append(f"a refused goal still moved the joints by {drift:.1f} deg")

    print()
    if fails:
        print(f"FAIL ({len(fails)}):")
        for f in fails:
            print("  - " + f)
        return 1
    print(f"PASS [{label}]: 3-D joints respond within the resolved band; refusals never reach the sim.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
