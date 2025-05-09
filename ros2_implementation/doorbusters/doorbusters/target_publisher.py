import rclpy
from rclpy.node import Node
from custom_msgs.msg import *
from nav_msgs.msg import Odometry

class GlobalTargetPublisher(Node):
    def __init__(self):
        super().__init__('global_target_publisher')

        # Target pose (fixed)
        self.odom_pub = self.create_publisher(Odometry, '/target/odom', 10)
        self.timer = self.create_timer(0.1, self.publish_target)  # 10 Hz

    def publish_target(self):
        odom_msg = Odometry()

        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.pose.pose.position.x = 5.0
        odom_msg.pose.pose.position.y = 5.0
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalTargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
