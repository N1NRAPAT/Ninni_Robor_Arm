#!/usr/bin/env python3
"""Closed-form IK for the Ninni arm.

Geometry derived directly from URDF joint origins:
  waist axis is global -Z; joints 2,3 are pitch joints in a plane that is
  offset laterally from the waist axis by a constant.
  left_3_arm (theta4) does NOT affect EE position -- only orientation.

Takes a target x y z, solves analytically, checks joint limits,
verifies by forward kinematics, then executes via FollowJointTrajectory.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINT_NAMES = ['Servor_1_to_waist', 'left_1_arm', 'left_2_arm', 'left_3_arm']
CONTROLLER_ACTION = '/ninni_arm_controller/follow_joint_trajectory'

# ---- geometry constants, straight from the URDF ----
P1 = (-0.0133535, 0.021776, 0.0426284)   # waist joint origin in base frame
A_OFF = 0.000373                          # in-plane offset at shoulder
O_OFF = -0.0169731 + 2 * (-1.91259e-5)    # constant out-of-plane offset
Z_OFF = 0.049                             # shoulder height above waist origin
L2 = 0.096                                # shoulder -> elbow
L3 = 0.096                                # elbow -> EE

# joint limits (rad), from URDF
LIMITS = {
    'Servor_1_to_waist': (-3.14159, 3.14159),
    'left_1_arm':        (-1.5708,  1.5708),
    'left_2_arm':        (-2.0944,  2.0944),
    'left_3_arm':        (-2.0944,  2.0944),
}

SECONDS_PER_MOVE = 6.0
PREFER_ELBOW = +1        # flip to -1 for the other elbow branch
EE_PITCH = 0.0           # desired theta2+theta3+theta4; theta4 absorbs the rest


def wrap(a):
    """normalize angle to [-pi, pi]"""
    return math.atan2(math.sin(a), math.cos(a))


def fk(t1, t2, t3):
    """Forward kinematics -- EE position only (theta4 does not matter)."""
    s1, c1 = math.sin(t1), math.cos(t1)
    s2, c2 = math.sin(t2), math.cos(t2)
    s23, c23 = math.sin(t2 + t3), math.cos(t2 + t3)

    r = -A_OFF - L2 * s2 - L3 * s23
    h = Z_OFF + L2 * c2 + L3 * c23

    x = P1[0] + r * s1 + O_OFF * c1
    y = P1[1] + r * c1 - O_OFF * s1
    z = P1[2] + h
    return x, y, z


def solve_candidates(x, y, z):
    """Return every geometrically valid (t1,t2,t3,t4) for this target."""
    dx = x - P1[0]
    dy = y - P1[1]
    dz = z - P1[2]

    rho = math.hypot(dx, dy)
    if rho < abs(O_OFF) - 1e-9:
        return []          # inside the lateral-offset cylinder: unreachable

    ratio = max(-1.0, min(1.0, O_OFF / rho))
    base = math.acos(ratio)
    phi = math.atan2(dy, dx)

    out = []
    # two waist branches: arm leaning to +r or -r
    for sign in (+1, -1):
        t1 = wrap(sign * base - phi)
        r = rho * math.sin(phi + t1)

        P = -(A_OFF + r)          # = L2*sin(t2) + L3*sin(t2+t3)
        Q = dz - Z_OFF            # = L2*cos(t2) + L3*cos(t2+t3)

        cos_t3 = (P * P + Q * Q - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
        if abs(cos_t3) > 1.0:
            continue              # out of the planar arm's reach

        for elbow in (+1, -1):
            t3 = elbow * math.acos(max(-1.0, min(1.0, cos_t3)))
            k1 = L2 + L3 * math.cos(t3)
            k2 = L3 * math.sin(t3)
            t2 = math.atan2(P * k1 - Q * k2, P * k2 + Q * k1)
            t4 = wrap(EE_PITCH - t2 - t3)
            out.append((wrap(t1), wrap(t2), wrap(t3), t4, elbow))
    return out


def within_limits(angles):
    for name, a in zip(JOINT_NAMES, angles):
        lo, hi = LIMITS[name]
        if a < lo - 1e-6 or a > hi + 1e-6:
            return False
    return True


def solve_ik(x, y, z):
    """Pick exactly one solution: in-limits, preferred elbow branch, best FK error."""
    cands = solve_candidates(x, y, z)
    valid = []
    for t1, t2, t3, t4, elbow in cands:
        angles = (t1, t2, t3, t4)
        if not within_limits(angles):
            continue
        fx, fy, fz = fk(t1, t2, t3)
        err = math.dist((fx, fy, fz), (x, y, z))
        valid.append((err, elbow, angles))

    if not valid:
        return None, None

    preferred = [v for v in valid if v[1] == PREFER_ELBOW]
    pool = preferred if preferred else valid
    pool.sort(key=lambda v: v[0])
    err, elbow, angles = pool[0]
    return angles, (err, elbow)


class AnalyticIK(Node):
    def __init__(self):
        super().__init__('ik_analytic_node')
        self.client = ActionClient(self, FollowJointTrajectory, CONTROLLER_ACTION)

    def send(self, positions):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=int(SECONDS_PER_MOVE))
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('goal rejected')
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        return True

    def run(self):
        self.get_logger().info('waiting for controller action server...')
        self.client.wait_for_server()

        hx, hy, hz = fk(0.0, 0.0, 0.0)
        print(f"\nHome pose (all joints 0): x={hx:.4f} y={hy:.4f} z={hz:.4f}")
        print("Ctrl+C to quit.\n")

        while rclpy.ok():
            raw = input("target x y z (m), or 'home': ").strip()
            if raw.lower() in ('home', 'h'):
                print("  returning to home (all joints 0)...")
                if self.send([0.0, 0.0, 0.0, 0.0]):
                    print("  at home.\n")
                continue
            parts = raw.split()
            if len(parts) != 3:
                print("  need 3 numbers\n")
                continue
            try:
                x, y, z = [float(p) for p in parts]
            except ValueError:
                print("  couldn't parse those\n")
                continue

            angles, info = solve_ik(x, y, z)
            if angles is None:
                print("  unreachable (no in-limit solution)\n")
                continue

            err, elbow = info
            deg = [round(math.degrees(a), 2) for a in angles]
            fx, fy, fz = fk(angles[0], angles[1], angles[2])
            print(f"  solution (deg): {deg}   elbow={elbow:+d}")
            print(f"  FK check: x={fx:.4f} y={fy:.4f} z={fz:.4f}  err={err*1000:.3f} mm")
            print("  executing...")
            if self.send(angles):
                print("  reached target. stopped.\n")
            else:
                print("  execution failed.\n")


def main():
    rclpy.init()
    node = AnalyticIK()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
