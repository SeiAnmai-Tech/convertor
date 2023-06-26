import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
import numpy as np

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.publish_initial_pose()

    def publish_initial_pose(self):
        pose_cov_stamped = PoseWithCovarianceStamped()

        # Set the header information
        pose_cov_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_cov_stamped.header.frame_id = 'map'  # Replace with your desired frame ID
        
        # Set the position of the origin point
        pose_cov_stamped.pose.pose.position.x = 0.0  # Change the x-coordinate to your desired value
        pose_cov_stamped.pose.pose.position.y = 0.0  # Change the y-coordinate to your desired value
        pose_cov_stamped.pose.pose.position.z = 0.0  # Change the z-coordinate to your desired value
        
        # Set the orientation of the origin point
        pose_cov_stamped.pose.pose.orientation.x = 0.0
        pose_cov_stamped.pose.pose.orientation.y = 0.0
        pose_cov_stamped.pose.pose.orientation.z = 0.0
        pose_cov_stamped.pose.pose.orientation.w = 1.0

        # Set the covariance matrix
        covariance_matrix = np.zeros((6, 6))
        # covariance_matrix[0] = 0.1  # x
        # covariance_matrix[7] = 0.1  # y
        # covariance_matrix[14] = 0.1  # z
        # covariance_matrix[21] = 0.05  # rotation x
        # covariance_matrix[28] = 0.05  # rotation y
        # covariance_matrix[35] = 0.05  # rotation z
        pose_cov_stamped.pose.covariance = covariance_matrix.flatten().tolist()

        self.publisher_.publish(pose_cov_stamped)
        self.get_logger().info('Published initial pose')

def main(args=None):
    rclpy.init(args=args)
    initial_pose_publisher = InitialPosePublisher()
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
