#!/usr/bin/env python3
"""Probe /compute_ik across each axis to find the reachable envelope.
Queries only -- never sends a trajectory, so the arm does not move."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration

GROUP_NAME = 'ninni_arm'
BASE_FRAME = 'base_servo_board'
EE_LINK = 'left_cover_2'
TARGET_QUAT = [0.5, 0.5, 0.5, 0.5]

ORIGIN = [-0.0304, 0.0214, 0.2836]   # your measured FK pose
STEP = 0.01                          # 1 cm
MAX_STEPS = 60                       # up to +/- 60 cm


class Sweeper(Node):
    def __init__(self):
        super().__init__('ik_workspace_sweep')
        self.current_js = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.ik = self.create_client(GetPositionIK, '/compute_ik')

    def _js_cb(self, msg):
        self.current_js = msg

    def wait_ready(self):
        self.ik.wait_for_service()
        while rclpy.ok() and self.current_js is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def reachable(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = TARGET_QUAT[0]
        pose.pose.orientation.y = TARGET_QUAT[1]
        pose.pose.orientation.z = TARGET_QUAT[2]
        pose.pose.orientation.w = TARGET_QUAT[3]

        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.ik_link_name = EE_LINK
        req.ik_request.pose_stamped = pose
        req.ik_request.robot_state.joint_state = self.current_js
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout = Duration(sec=0, nanosec=50_000_000)

        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        return res is not None and res.error_code.val == 1

    def sweep_axis(self, axis_idx, axis_name):
        lo = hi = ORIGIN[axis_idx]

        for direction, label in ((1, 'max'), (-1, 'min')):
            pt = list(ORIGIN)
            last_ok = ORIGIN[axis_idx]
            for i in range(1, MAX_STEPS + 1):
                pt[axis_idx] = ORIGIN[axis_idx] + direction * i * STEP
                if self.reachable(*pt):
                    last_ok = pt[axis_idx]
                else:
                    break
            if direction == 1:
                hi = last_ok
            else:
                lo = last_ok

        print(f"  {axis_name}:  min = {lo:+.4f}   max = {hi:+.4f}   "
              f"(span {hi - lo:.4f} m)")
        return lo, hi

    def run(self):
        self.wait_ready()
        print(f"\nOrigin: x={ORIGIN[0]:.4f} y={ORIGIN[1]:.4f} z={ORIGIN[2]:.4f}")
        print(f"Sweeping each axis independently (others held at origin)...\n")

        bounds = {}
        for idx, name in ((0, 'x'), (1, 'y'), (2, 'z')):
            bounds[name] = self.sweep_axis(idx, name)

        print("\n--- suggested test targets (midpoints of each axis range) ---")
        for name, (lo, hi) in bounds.items():
            mid_lo = ORIGIN['xyz'.index(name)] + (lo - ORIGIN['xyz'.index(name)]) * 0.5
            mid_hi = ORIGIN['xyz'.index(name)] + (hi - ORIGIN['xyz'.index(name)]) * 0.5
            for val in (mid_lo, mid_hi):
                pt = list(ORIGIN)
                pt['xyz'.index(name)] = val
                print(f"  {pt[0]:+.4f}  {pt[1]:+.4f}  {pt[2]:+.4f}")
        print()


def main():
    rclpy.init()
    node = Sweeper()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
