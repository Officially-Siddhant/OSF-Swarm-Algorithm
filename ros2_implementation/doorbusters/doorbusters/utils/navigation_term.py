import numpy as np
from geometry_msgs.msg import Vector3
from custom_msgs.msg import *
from nav_msgs.msg import Odometry
import rclpy
#import the linear algebra module if not imported by defaul from numpy.

# Constants from the paper
K1 = 0.9   # Distance threshold factor (relative) + look at page
K2 = 1.5   # Position attraction when far
K3 = 1.0   # Velocity attraction when far

# The weights for pos_term and vel_term are designed in
# such a way that the attractive force is small enough at
# the initial time and increases as we get closer (pretty much like attraction)

c1 = 0.8   # Position gain when close
c2 = 0.5   # Velocity gain when close

def compute_navigation_force(pose_i, twist_i,
                             target_pose, target_twist,
                             initial_pose, initial_target_pose):

    # Convert to numpy arrays
    qi = np.array([pose_i.pose.position.x,
                   pose_i.pose.position.y,
                   pose_i.pose.position.z])

    q0 = np.array([initial_pose.pose.position.x,
                   initial_pose.pose.position.y,
                   initial_pose.pose.position.z])

    qt0 = np.array([initial_target_pose.pose.position.x,
                    initial_target_pose.pose.position.y,
                    initial_target_pose.pose.position.z])

    pi = np.array([twist_i.twist.linear.x,
                   twist_i.twist.linear.y,
                   twist_i.twist.linear.z])

    qt = np.array([target_pose.pose.position.x,
                   target_pose.pose.position.y,
                   target_pose.pose.position.z])

    pt = np.array([target_twist.twist.linear.x,
                   target_twist.twist.linear.y,
                   target_twist.twist.linear.z])

    #     initial_positions[robot_id] = qi.copy()
    #     initial_target = qt.copy() #i.e., qt(0) at time = 0.

    dist_now = np.linalg.norm(qi - qt)
    dist_initial = np.linalg.norm(q0 - qt0)

    # Decide control law based on proximity
    if dist_now > K1 * dist_initial:
        # Far: use normalized attraction
        direction = (qi - qt)
        norm = np.linalg.norm(direction) + 1e-6  # Avoid divide-by-zero
        pos_term = -K2 * direction / norm
        vel_term = -K3 * (pi - pt) / norm
        f_t = pos_term + vel_term
    else:
        # Close: use standard linear gains
        f_t = -c1 * (qi - qt) - c2 * (pi - pt)
        # print("Target Tracking: ")
        rclpy.logging.get_logger("doorbusters").debug("Target Tracking: ")
    result = Vector3()
    result.x = f_t[0]
    result.y = f_t[1]
    result.z = f_t[2]
    return result
