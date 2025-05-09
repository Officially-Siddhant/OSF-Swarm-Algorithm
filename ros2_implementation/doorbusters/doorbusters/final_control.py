#!/usr/bin/env python3
import rclpy
from networkx.classes import neighbors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3, Point, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, ColorRGBA
from custom_msgs.msg import *

from tf2_ros import TransformException, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from doorbusters.utils.flocking_control import compute_flocking_force
from doorbusters.utils.navigation_term import compute_navigation_force
from doorbusters.utils.damping_term import compute_damping_force


class FinalControlNode(Node):
    def __init__(self):
        super().__init__('final_control_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.declare_parameter('is_informed', True)
        self.is_informed = self.get_parameter('is_informed').get_parameter_value().bool_value

        self.declare_parameter('neighborhood_id', '0')
        self.neighborhood_id = self.get_parameter('neighborhood_id').get_parameter_value().string_value

        self.declare_parameter('num_agents', 10)
        self.num_agents = self.get_parameter('num_agents').get_parameter_value().integer_value

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value

        self.tf2_buffer = Buffer()
        self.tf2_listener = TransformListener(self.tf2_buffer, self)

        self.initialize = False
        self.initial_odom = Odometry()
        self.initial_target = Odometry()

        # Internal state
        self.odom = Odometry()
        self.neighbors = WeightedTopologyNeighbors()
        self.neighbors.neighbors_odom = [Odometry() for _ in range(10)]
        self.target_odom = Odometry()

        # Marker for trail
        self.trail_marker = Marker()
        self.trail_marker.header.frame_id = self.world_frame
        self.trail_marker.ns = 'trajectory'
        self.trail_marker.id = 0
        self.trail_marker.type = Marker.LINE_STRIP
        self.trail_marker.action = Marker.ADD
        self.trail_marker.scale.x = 0.02  # thin line
        self.trail_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        self.trail_marker.points = []

        # Subscribers
        self.create_subscription(Odometry,
                                 'odom',
                                 self.odom_callback, 10)
        self.neighbors_sub = self.create_subscription(
            WeightedTopologyNeighbors,
            '/neighbors_odom',
            self.neighbor_callback,
            qos_profile
        )
        self.create_subscription(Odometry,
                                 '/target/odom',
                                 self.target_odom_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.neighbor_pub = self.create_publisher(WeightedTopologyNeighbors,
                                                  '/neighbors_odom', 10)
        self.marker_publisher = self.create_publisher(Marker, 'markers_path', 10)

        # Main control loop
        self.create_timer(0.05, self.control_loop)  # 20 Hz

    def odom_callback(self, msg):
        self.odom = msg

        try:
            odom_pose = PoseStamped()
            odom_pose.header = msg.header
            odom_pose.pose = msg.pose.pose

            transform = self.tf2_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )

            pose_in_world = do_transform_pose(odom_pose, transform).pose

            # Add new point to trail
            self.trail_marker.header.stamp = self.get_clock().now().to_msg()
            point = Point()
            point.x = pose_in_world.position.x
            point.y = pose_in_world.position.y
            point.z = pose_in_world.position.z
            self.trail_marker.points.append(point)

            # Optional: limit trail length
            if len(self.trail_marker.points) > 200:
                self.trail_marker.points.pop(0)

            self.marker_publisher.publish(self.trail_marker)

        except TransformException as ex:
            self.get_logger().warn(f'Transform lookup failed: {ex}')

    def neighbor_callback(self, msg):
        self.neighbors = msg
        self.neighbors.neighbors_odom[int(self.neighborhood_id)] = self.odom

    def target_odom_callback(self, msg):
        self.target_odom = msg

    def control_loop(self):

        if self.odom is None:
            self.get_logger().warn("No odometry data received yet.")
            return
        if not self.initialize:
            self.initialize = True
            self.get_logger().info("Storing initial odometry data.")
            self.initial_odom = self.odom
            self.initial_target = self.target_odom

        f_alpha = compute_flocking_force(self.odom.pose, self.odom.twist, self.neighbors)

        if self.is_informed and self.target_odom:
            f_t = compute_navigation_force(self.odom.pose,
                                           self.odom.twist,
                                           self.target_odom.pose,
                                           self.target_odom.twist,
                                           self.initial_odom.pose,
                                           self.initial_target.pose)
            f_damp = compute_damping_force(self.odom.pose, self.odom.twist, self.target_odom.pose)
        else:
            f_t = Vector3()
            f_damp = Vector3()
        control_force = Vector3()
        control_force.x = f_alpha.x + f_t.x + f_damp.x
        control_force.y = f_alpha.y + f_t.y + f_damp.y
        control_force.z = f_alpha.z + f_t.z + f_damp.z

        twist_msg = Twist()
        twist_msg.linear = control_force
        self.cmd_pub.publish(twist_msg)
        self.neighbor_pub.publish(self.neighbors)

def main(args=None):
    rclpy.init(args=args)
    node = FinalControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

