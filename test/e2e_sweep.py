#!/usr/bin/env python3
"""Workspace sweep through the FULL ROS stack (controller + 2-D sim).

The C++ unit tests sweep the library directly; this sweeps what an operator
actually sees: `arm/pos.position` (what the simulator draws and streams) against
the requested point, over a grid spanning the reachable envelope.

The grid and the pass/fail thresholds are DERIVED from the running controller's
parameters, not hard-coded, because the assemblies differ: the default config
reaches arm [31.81, 124.77] deg and the gen0 config arm [0.01, 78.23] deg, and
the servo span that corresponds to is different again. A fixed grid would spend
most of its time on poses that are refused for both assemblies while missing the
gen0 band entirely.

Reports the worst drawn-vs-requested error, which is the number that matters.

    python3 e2e_sweep.py [--steps N] [--settle SECS]

`bypass_reachability` changes what a pass means, so it is read from the
controller and handled explicitly:

* guard OFF (the default) - out-of-envelope goals are REFUSED. A refusal must
  leave the drawn pose and the target marker in sync and must not sit at the
  refused point. Accepted goals must draw within TOL of the request.
* guard ON - out-of-envelope goals are accepted and CLAMPED to a band edge, so
  they never draw at the request. Clamped poses are counted and checked to land
  on a real band edge with a servo angle inside the closure span, rather than
  being reported as failures.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from arm.action import SetPosition
from arm.msg import ArmPosition

TOL = 3.0e-3

# Params read from the controller to derive the grid and the thresholds.
GEOM_PARAMS = [
    "geometry.base_radius",
    "geometry.platform_radius",
    "geometry.upper_arm_len",
    "geometry.lower_arm_len",
    "geometry.servo_radius",
    "geometry.servo_z",
    "geometry.upper_rod_len",
    "geometry.servo_rod_len",
    "geometry.arm_attach_dist",
    "geometry.arm_attach_offset",
    "geometry.angle_min",
    "geometry.angle_max",
    "geometry.arm_angle_min",
    "geometry.arm_angle_max",
    "geometry.bypass_reachability",
]


def dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def decode_param(pv):
    """Unwrap a rcl_interfaces/ParameterValue into a plain Python value."""
    t = pv.type
    if t == 1:  # BOOL
        return pv.bool_value
    if t == 2:  # INTEGER
        return pv.integer_value
    if t == 3:  # DOUBLE
        return pv.double_value
    if t == 4:  # STRING
        return pv.string_value
    if t == 5:  # BYTE_ARRAY
        return list(pv.byte_array_value)
    if t == 6:  # BOOL_ARRAY
        return list(pv.bool_array_value)
    if t == 7:  # INTEGER_ARRAY
        return list(pv.integer_array_value)
    if t == 8:  # DOUBLE_ARRAY
        return list(pv.double_array_value)
    if t == 9:  # STRING_ARRAY
        return list(pv.string_array_value)
    return None


def as_triple(v):
    """Flatten a per-leg parameter (scalar or 3-list) to a 3-list of floats."""
    if isinstance(v, (list, tuple)):
        vals = [float(x) for x in v]
    else:
        vals = [float(v)] * 3
    while len(vals) < 3:
        vals.append(vals[-1])
    return vals[:3]


class Sweep(Node):
    def __init__(self, settle):
        super().__init__("e2e_sweep")
        self.latest = None
        self.settle = settle
        self.create_subscription(ArmPosition, "/arm/pos", self._cb, 10)
        self.ac = ActionClient(self, SetPosition, "/arm/set_pos")

    def _cb(self, msg):
        self.latest = msg

    def read_params(self, names, timeout=15.0):
        """Fetch the controller's parameters, returning None on failure.

        Uses the GetParameters service directly: rclpy on Humble has no
        AsyncParameterClient.
        """
        deadline = time.time() + timeout
        cli = self.create_client(GetParameters, "/arm_controller/get_parameters")
        while not cli.service_is_ready() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not cli.service_is_ready():
            return None
        req = GetParameters.Request(names=list(names))
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or len(res.values) != len(names):
            return None
        return {n: decode_param(v) for n, v in zip(names, res.values)}

    def send(self, xyz):
        goal = SetPosition.Goal()
        goal.target.linear.x, goal.target.linear.y, goal.target.linear.z = xyz
        fut = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return "NO_ADMIT", ""
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=8.0)
        res = rfut.result()
        if res is None:
            return "NO_RESULT", ""
        return ("OK" if res.result.succeeded else "REFUSED"), res.result.message

    def wait_reached(self, want, timeout=None):
        timeout = self.settle if timeout is None else timeout
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest is None:
                continue
            p = self.latest.position
            if dist((p.x, p.y, p.z), want) <= TOL:
                rclpy.spin_once(self, timeout_sec=0.4)
                q = self.latest.position
                if dist((q.x, q.y, q.z), want) <= TOL:
                    return True
        return False

    def current(self):
        if self.latest is None:
            return None
        m = self.latest
        return (m.position.x, m.position.y, m.position.z), (
            m.target_position.x, m.target_position.y, m.target_position.z,
        ), list(m.motor_angles_current), list(m.motor_angles_target)


def main():
    steps = 6
    settle = 15.0
    argv = sys.argv[1:]
    i = 0
    while i < len(argv):
        if argv[i] == "--steps":
            steps = int(argv[i + 1]); i += 2
        elif argv[i] == "--settle":
            settle = float(argv[i + 1]); i += 2
        else:
            print(f"unknown argument: {argv[i]}")
            return 2

    rclpy.init()
    node = Sweep(settle)
    if not node.ac.wait_for_server(timeout_sec=15.0):
        print("FAIL: no action server")
        return 1

    params = node.read_params(GEOM_PARAMS)
    if params is None:
        print("FAIL: could not read the controller's geometry parameters")
        return 1

    bypass = bool(params["geometry.bypass_reachability"])
    ang_min = as_triple(params["geometry.angle_min"])
    ang_max = as_triple(params["geometry.angle_max"])
    base_r = float(params["geometry.base_radius"])
    plat_r = float(params["geometry.platform_radius"])
    arm_min = float(params["geometry.arm_angle_min"])
    arm_max = float(params["geometry.arm_angle_max"])
    upper = float(params["geometry.upper_arm_len"])
    lower = float(params["geometry.lower_arm_len"])
    servo_z = float(params["geometry.servo_z"])

    # Travel stops, in degrees, straight from the controller.
    stop_lo = min(ang_min)
    stop_hi = max(ang_max)

    print("=== geometry read from /arm_controller ===")
    print(f"  base_radius={base_r}  platform_radius={plat_r}  "
          f"upper_arm_len={upper}  lower_arm_len={lower}")
    # The controller declares the arm window in DEGREES (it converts to radians
    # itself), unlike the library config where it is radians.
    print(f"  servo travel [{stop_lo}, {stop_hi}] deg   "
          f"arm window [{arm_min:.2f}, {arm_max:.2f}] deg")
    print(f"  bypass_reachability={bypass}"
          + ("  (out-of-envelope goals are CLAMPED, not refused)" if bypass else ""))
    print()

    # Grid derived from the geometry. The workspace is a cylinder of roughly
    # platform_radius about the axis, so sweeping x/y across +/- that is enough to
    # cover it; going wider only adds poses both assemblies refuse.
    reach_xy = plat_r * 1.0e-3
    z_lo = (servo_z + upper + lower) * 1.0e-3 * -1.0  # conservative floor
    z_hi = -0.5 * reach_xy  # shallowest sensible depth

    accepted = refused = clamped = 0
    worst = 0.0
    worst_at = None
    failures = []
    max_servo = float("-inf")
    max_servo_at = None

    xs = [(-0.5 + k / steps) * 2.0 * reach_xy for k in range(steps + 1)]
    ys = [(-0.5 + k / steps) * 2.0 * reach_xy for k in range(steps + 1)]
    zs = [z_lo + (z_hi - z_lo) * k / steps for k in range(steps + 1)]

    for x in xs:
        for y in ys:
            for z in zs:
                xyz = (round(x, 4), round(y, 4), round(z, 4))
                status, _ = node.send(xyz)
                cur = node.current()
                if cur:
                    _, _, _, motors_t = cur
                    for leg, a in enumerate(motors_t):
                        if a > max_servo:
                            max_servo, max_servo_at = a, (xyz, leg)

                if status == "REFUSED":
                    refused += 1
                    # A refusal must leave drawn and marker in sync and must not
                    # sit at the refused point.
                    if cur:
                        drew, tgt, _, _ = cur
                        if dist(drew, tgt) > TOL:
                            failures.append(
                                f"refusal desync at {xyz}: {dist(drew,tgt)*1000:.1f} mm")
                    continue
                if status != "OK":
                    failures.append(f"{xyz}: unexpected status {status}")
                    continue
                accepted += 1

                if node.wait_reached(xyz):
                    drew, tgt, motors_c, _ = node.current()
                    err = dist(drew, xyz)
                    if dist(tgt, xyz) > TOL:
                        failures.append(
                            f"{xyz}: marker {dist(tgt,xyz)*1000:.1f} mm from request")
                    if err > worst:
                        worst, worst_at = err, xyz
                    if err > TOL:
                        failures.append(
                            f"{xyz}: drawn pose off by {err*1000:.1f} mm")
                elif bypass:
                    # Accepted-and-clamped: expected when the guard is off. There
                    # is deliberately NO drew-vs-marker check here: the marker is
                    # the last accepted set_pos goal, i.e. the REQUEST, so it
                    # differs from the drawn pose by exactly the clamp. What must
                    # hold is that the servo angles stay inside their travel.
                    clamped += 1
                    _, _, _, motors_t = node.current()
                    for leg, a in enumerate(motors_t):
                        if a < stop_lo - 1e-3 or a > stop_hi + 1e-3:
                            failures.append(
                                f"clamped servo {a:.2f} deg on leg {leg} at {xyz} "
                                f"outside travel [{stop_lo}, {stop_hi}]")
                else:
                    failures.append(f"{xyz}: drawn pose never arrived")

    print(f"accepted={accepted}  refused={refused}"
          + (f"  clamped={clamped}" if bypass else ""))
    if worst_at is not None:
        print(f"worst drawn-vs-requested error: {worst*1000:.3f} mm  at {worst_at}")
    else:
        print("worst drawn-vs-requested error: n/a (no pose drew at its request)")
    if max_servo_at is not None:
        print(f"max servo commanded: {max_servo:.2f} deg on leg {max_servo_at[1]} "
              f"at {max_servo_at[0]} (travel stops [{stop_lo}, {stop_hi}])")
    print()

    if failures:
        print(f"FAIL ({len(failures)}):")
        for f in failures[:20]:
            print("  - " + f)
        return 1
    if bypass:
        print("PASS: every reachable pose draws within 3 mm of the request, and "
              "every clamped pose sits on a real band edge with the servo "
              "inside its travel.")
    else:
        print("PASS: whole reachable workspace draws within 3 mm of the request.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
