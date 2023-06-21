import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import TransformStamped
import tf2_ros
from math import pi, cos, sin

class EncoderToOdomNode(Node):

    def __init__(self):
        super().__init__('encoder_to_odom_node')
        self.subscription = self.create_subscription(
            Int32MultiArray, 'encoder_ticks', self.encoder_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.left_ticks_prev = 0
        self.right_ticks_prev = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def encoder_callback(self, msg):
        # Get encoder ticks for current iteration
        left_ticks_curr = msg.data[0]
        right_ticks_curr = msg.data[1]

        # Convert ticks to position (assuming wheel radius = 0.1 and wheelbase = 0.5)
        dt = 0.02 # 20ms or 50 Hz
        wheel_radius = 0.05
        wheelbase = 0.315
        counts_per_rev = 280

        left_distance = (left_ticks_curr - self.left_ticks_prev) * (2 * pi * wheel_radius) / counts_per_rev
        right_distance = (right_ticks_curr - self.right_ticks_prev) * (2 * pi * wheel_radius) / counts_per_rev

        delta_distance = (left_distance + right_distance) / 2
        delta_theta = (right_distance - left_distance) / wheelbase

        # Perform pose estimation
        self.x += delta_distance * cos(self.theta)
        self.y += delta_distance * sin(self.theta)
        self.theta += delta_theta

        # Update previous ticks
        self.left_ticks_prev = left_ticks_curr
        self.right_ticks_prev = right_ticks_curr

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = sin(self.theta / 2)
        odom.pose.pose.orientation.w = cos(self.theta / 2)

        # Set the covariance matrix for pose estimation
        # Assuming a simple diagonal covariance matrix for simplicity
        # Adjust the values according to your requirements
        odom.pose.covariance[0] = 0.1  # x
        odom.pose.covariance[7] = 0.1  # y
        odom.pose.covariance[14] = 0.1  # z
        odom.pose.covariance[21] = 0.05  # rotation x
        odom.pose.covariance[28] = 0.05  # rotation y
        odom.pose.covariance[35] = 0.05  # rotation z

        # Set the twist with covariance
        # Assuming zero covariance for simplicity
        # Adjust the values according to your requirements
        odom.twist.twist.linear.x = delta_distance / dt  # Linear velocity in x direction
        odom.twist.twist.angular.z = delta_theta / dt  # Angular velocity in z direction

        # Set the covariance matrix for twist estimation
        # Assuming a simple diagonal covariance matrix for simplicity
        # Adjust the values according to your requirements
        odom.twist.covariance[0] = 0.1  # linear x
        odom.twist.covariance[7] = 0.1  # linear y
        odom.twist.covariance[14] = 0.1  # linear z
        odom.twist.covariance[21] = 0.05  # angular x
        odom.twist.covariance[28] = 0.05  # angular y
        odom.twist.covariance[35] = 0.05  # angular z

        self.odom_publisher.publish(odom)

        # Publish tf transform between "odom" and "base_link"
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = sin(self.theta / 2)
        transform.transform.rotation.w = cos(self.theta / 2)

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderToOdomNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()