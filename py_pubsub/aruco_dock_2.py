import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist
import math


class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.pose1 = None
        self.pose2 = None

        self.subscription1 = self.create_subscription(
            Pose,
            '/aruco_double/pose',
            self.pose1_callback,
            10
        )

        self.subscription2 = self.create_subscription(
            Pose,
            '/aruco_double/pose2',
            self.pose2_callback,
            10
        )
        
        self.docked_pub = self.create_publisher(Bool, 'docked', 10)

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = 0.0
        self.cmd.angular.x = 0.0
        self.cmd.angular.y = 0.0
        self.cmd.angular.z = 0.0

        self.timer = self.create_timer(0.1, self.timer_callback)

    def pose1_callback(self, msg):
        self.pose1 = msg
        # self.get_logger().info('Received Pose1')

    def pose2_callback(self, msg):
        self.pose2 = msg
        # self.get_logger().info('Received Pose2')

    def timer_callback(self):
        trans = [0.0, 0.0, 0.0]
        trans[0] = (self.pose1.position.z + self.pose2.position.z) / 2.0
        trans[1] = (- self.pose1.position.x - self.pose2.position.x) / 2.0
        trans[2] = (- self.pose1.position.y - self.pose2.position.y) / 2.0

        dist = math.sqrt(trans[0] ** 2 + trans[1] ** 2 + trans[2] ** 2)
        angular = 2.0 * math.atan2(trans[1], trans[0])
        linear = 0.1 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        print(dist)

        if dist < 0.165:
            # self.cmd.linear.x = linear
            # self.cmd.angular.z = angular
            # self.publisher.publish(self.cmd)
            # time.sleep(0.2)
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            docked = Bool()
            docked.data = True
            self.publisher.publish(self.cmd)
            self.docked_pub.publish(docked)
            exit(0)
        elif dist >= 0.165:
            self.cmd.linear.x = linear
            self.cmd.angular.z = angular
            self.publisher.publish(self.cmd)



def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
