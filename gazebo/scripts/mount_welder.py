#!/usr/bin/env python3
"""mount_welder.py — rigidly weld a spawned payload onto the drone at runtime.

The "mount" step of the combined-mode single launcher ('./sim.sh single'):
after the payload model (delta arm or endpoint) has been created INTO the
drone's already-running Gazebo world, this node attaches it rigidly by
posting a world-scope fixed <joint> SDF fragment to the gazebo server's
'/world/<world>/create' service (gz.msgs.EntityFactory):

    <joint name="arm_mount" type="fixed">
      <parent> <drone>::base_link </parent>
      <child>  <child-model>::base_link </child>
      <pose> 0 0 <offset-z> 0 0 0 </pose>       (frame = parent link)
    </joint>

The child adopts the given pose in the drone base frame, so the payload snaps
to a spot below/around the drone and subsequently flies with it.

PRECONDITION (enforced by the URDF): the payload's base_link must be its ROOT
link with no other parent — a gz link can have exactly one parent joint, and
the mount weld is that parent. delta_arm.urdf.xacro drops its fake `world`
link when processed with mount=true; endpoint.urdf.xacro has none by design.

Run manually (already wired into arm_gazebo.launch.py via mount:=true):
    ros2 run arm_gazebo mount_welder.py --world apriltag1 \
        --child-model delta_arm --offset-z -0.05
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node as ROSNode


def _gz(cmd, timeout=15):
    if shutil.which('gz') is None:
        raise RuntimeError('`gz` CLI not on PATH.')
    try:
        run = subprocess.run(['gz'] + cmd, capture_output=True,
                             text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return 1, 'gz call timed out'
    return run.returncode, (run.stdout or '') + (run.stderr or '')


def _list_models():
    rc, out = _gz(['model', '--list'])
    if rc != 0:
        return []
    return re.findall(r'^\s*-\s*(\S+)\s*$', out, re.M)


def _pose6(value):
    parts = value.split()
    if len(parts) == 1:                      # a bare z offset
        return ['0', '0', parts[0], '0', '0', '0']
    if len(parts) == 6:                      # a full xyz rpy
        return parts
    raise ValueError(
        f'bad --offset-z "{value}" (want a number or a '
        '"x y z roll pitch yaw" 6-tuple)')


def _weld(log, world, parent, child, pose6):
    frag = ("<sdf version='1.9'><joint name='arm_mount"
            f"{int(time.time() * 1000)}' type='fixed'>"
            f"<parent>{parent}</parent><child>{child}</child>"
            f"<pose>{' '.join(pose6)}</pose></joint></sdf>")
    log(f'create: {parent} <-fixed-> {child} pose={pose6}')
    rc, out = _gz([
        'service', '-s', f'/world/{world}/create',
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '6000',
        '--req', f'sdf: "{frag}"',
    ])
    log(f'service rc={rc}: {out.strip()}')
    return rc == 0 and 'data: true' in out


def main():
    rclpy.init()
    node = ROSNode('mount_welder')
    log = node.get_logger().info
    try:
        argv = sys.argv[1:]
        if '--ros-args' in argv:             # strip ROS2 launch remap args
            argv = argv[:argv.index('--ros-args')]

        ap = argparse.ArgumentParser(
            description='Weld a payload model onto the drone (runtime joint).')
        ap.add_argument('--world', default='apriltag1')
        ap.add_argument('--child-model', default='delta_arm')
        ap.add_argument('--child-link', default='base_link')
        ap.add_argument('--drone-model', default='')
        ap.add_argument('--offset-z', default='-0.05')
        ap.add_argument('--retries', type=int, default=30)
        ap.add_argument('--poll', type=float, default=2.0)
        args = ap.parse_args(argv)

        child = args.child_model
        drone = args.drone_model or os.environ.get('ARM_DRONE_MODEL', '').strip()
        deadline = time.monotonic() + args.retries * args.poll

        # 1) wait for the payload (and the drone, auto-detected as any x500*)
        while time.monotonic() < deadline:
            models = set(_list_models())
            if child in models:
                if not drone:
                    for name in models:
                        if name.startswith('x500_') and name != child:
                            drone = name
                            break
                if drone and drone in models:
                    break
            time.sleep(args.poll)
        if child not in set(_list_models()):
            log(f'ERROR: payload model "{child}" never appeared in "{args.world}".')
            return 1
        if not (drone and drone in set(_list_models())):
            log(f'ERROR: drone model "{drone or "<auto x500*>"}" not found in '
                f'"{args.world}".')
            return 1

        pose6 = _pose6(args.offset_z)
        parent = f'{drone}::base_link'
        child_link = f'{child}::{args.child_link}'

        # 2) post the weld; retry the remainder of the budget on hiccups
        while time.monotonic() < deadline + 15:
            if _weld(log, args.world, parent, child_link, pose6):
                log(f'OK - welded {child_link} onto {parent} at {pose6}, '
                    f'world "{args.world}".')
                return 0
            time.sleep(2)
        log('FAILED - weld not confirmed by the create service.')
        return 1
    except (ValueError, RuntimeError) as exc:
        log(f'ERROR: {exc}')
        return 1
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())