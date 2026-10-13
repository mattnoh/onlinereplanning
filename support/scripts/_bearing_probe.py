#!/usr/bin/env python3
"""Print `beta rate` -- the staging bearing and its rate of change, in rad.

beta is the angle from the target's velocity direction to the target->drone
direction; it is the quantity tools_motion_seed_screen is parameterised on.

Why this exists: the campaign originally sampled beta with two separate
`rostopic echo -n1` calls, ~3-4 s per sample. The acceptance window is
+-0.35 rad, which for a circle at omega=0.55 rad/s is open for only 1.3 s --
the poll was slower than its own window and missed it, so 8/25 runs fired with
an uncontrolled bearing and both TIMEOUTs came from that group. One rospy
process reading both topics takes ~0.2 s and also yields the rate, which lets
the caller SLEEP the computed time instead of busy-polling.
"""
import math
import sys

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def beta_now():
    t = rospy.wait_for_message("/target/odom", Odometry, timeout=5.0)
    d = rospy.wait_for_message("/mavros/local_position/pose", PoseStamped, timeout=5.0)
    tx, ty = t.pose.pose.position.x, t.pose.pose.position.y
    vx, vy = t.twist.twist.linear.x, t.twist.twist.linear.y
    dx, dy = d.pose.position.x, d.pose.position.y
    if math.hypot(vx, vy) < 1e-6:
        return None
    return wrap(math.atan2(dy - ty, dx - tx) - math.atan2(vy, vx))


def main():
    rospy.init_node("bearing_probe", anonymous=True, disable_signals=True)
    b0 = beta_now()
    if b0 is None:
        print("nan nan")
        return
    rospy.sleep(1.0)
    b1 = beta_now()
    if b1 is None:
        print(f"{b0:.5f} nan")
        return
    print(f"{b1:.5f} {wrap(b1 - b0) / 1.0:.5f}")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        print("nan nan")
