import numpy as np
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from custom_msgs.msg import *
from nav_msgs.msg import Odometry
# Damping only depends on the informed agent's pose and twist and the target POSITION
# the idea is to dampen our velocity!
# Constants from the paper
K_DAMP = 0.8  # Damping gain
K4 = 1.2      # Threshold multiplier
R = 1.2       # Same as in flocking_control (neighborhood radius)


def compute_damping_force(pose_i, twist_i, target_pose):
    # Positions
    qi = np.array([pose_i.pose.position.x,
                   pose_i.pose.position.y,
                   pose_i.pose.position.z])

    qt = np.array([target_pose.pose.position.x,
                   target_pose.pose.position.y,
                   target_pose.pose.position.z])

    # Velocity
    pi = np.array([twist_i.twist.linear.x,
                   twist_i.twist.linear.y,
                   twist_i.twist.linear.z])

    dist = np.linalg.norm(qi - qt)

    if dist < K4 * R:
        f_damp = -K_DAMP * pi
    else:
        f_damp = np.zeros(3)

    result = Vector3()
    result.x = f_damp[0]
    result.y = f_damp[1]
    result.z = f_damp[2]
    return result
