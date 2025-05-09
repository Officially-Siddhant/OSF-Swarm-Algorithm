import numpy as np
from geometry_msgs.msg import Vector3
from custom_msgs.msg import *
from nav_msgs.msg import Odometry
import rclpy


# The Olfati Saber flocking control law is u=fα+ft (control term = (gradient component + consensus component) + target tracking component)
# Gradient component = ∑ φ_α(∥qj −qi∥σ)nij +
# Consensus component = ∑ aij(q)(pj − pi) ; so I gotta populate the adjacency matrix
# φ_α() is the action term
#
# Constants from the paper
EPSILON = 0.1
D = 1.0         # Desired inter-agent distance (normalized)
KC = 1.2        # Scaling factor
R = KC * D      # Neighborhood radius
R_SIGMA = (1 / EPSILON) * (np.sqrt(1 + EPSILON * R ** 2) - 1)  # Sigma norm of r
H = 0.2         # Bump function transition width
a = 5.0         # Sigmoid parameter
b = 5.0         # Sigmoid parameter

c = abs(a - b) / np.sqrt(4 * a * b)  # Ensures phi(0) = 0
d_alpha = (1 / EPSILON) * (np.sqrt(1 + EPSILON * D ** 2) - 1)


def sigma_norm(z):
    return (1.0 / EPSILON) * (np.sqrt(1 + EPSILON * np.dot(z, z)) - 1)


def bump_function(z): # this is ρ(z) with h belonging to (0,1). We can tune h. Check paper.
    if 0 <= z < H:
        return 1.0
    elif H <= z < 1:
        return 0.5 * (1 + np.cos(np.pi * (z - H) / (1 - H)))
    else:
        return 0.0


def phi(z):
    return 0.5 * ((a + b) * (z + c) / np.sqrt(1 + (z + c) ** 2) + (a - b))


def phi_alpha(z): #φ(z) = 0.5[(a+b)σ1(z+c)+(a−b)], here σ1(z) = z/ √(1+z^2) near equation (5)
    if z >= R_SIGMA:
        return 0.0
    return bump_function(z / R_SIGMA) * phi(z - d_alpha) #φ_α(z)=ρ(z/rα)φ(z−dalpha)


def compute_flocking_force(pose_i, twist_i, neighbors):
    qi = np.array([pose_i.pose.position.x,
                   pose_i.pose.position.y,
                   pose_i.pose.position.z])

    pi = np.array([twist_i.twist.linear.x,
                   twist_i.twist.linear.y,
                   twist_i.twist.linear.z])

    f_alpha = np.zeros(3)

    for neighbor in neighbors.neighbors_odom: #check for the way in which the message has been declared.
        qj = np.array([neighbor.pose.pose.position.x,
                       neighbor.pose.pose.position.y,
                       neighbor.pose.pose.position.z])

        pj = np.array([neighbor.twist.twist.linear.x,
                       neighbor.twist.twist.linear.y,
                       neighbor.twist.twist.linear.z])

        q_diff = qj - qi
        norm_sigma = sigma_norm(q_diff)

        if norm_sigma >= R_SIGMA: # Skip neighbors outside active range
            continue #we don't need to execute the rest, so we skip the iteration lol.

        n_ij = q_diff / np.sqrt(1 + EPSILON * np.dot(q_diff, q_diff))

        a_ij = bump_function(norm_sigma / R_SIGMA)

        # Gradient-based term - phi_alpha φα(z)=ρ(z/r_alpha)φ(z−d)
        grad_term = phi_alpha(norm_sigma) * n_ij

        # Consensus-based term
        consensus_term = a_ij * (pj - pi)
        #print('F alpha = {0} + /t {1}'.format(grad_term, consensus_term))
        rclpy.logging.get_logger("doorbusters").debug("F alpha = {0} + /t {1}".format(grad_term, consensus_term))
        f_alpha += grad_term + consensus_term

    result = Vector3()
    result.x = f_alpha[0]
    result.y = f_alpha[1]
    result.z = f_alpha[2]
    return result
