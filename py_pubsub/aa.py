import time
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from action_msgs.msg import GoalStatus

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.publisher_2 = self.create_publisher(Int32, 'goal_published', 10)
        self.subscription1 = self.create_subscription(
            Int32,
            '/goal_reached',
            self.goal_callback,
            10
        )
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
        self.subscription3 = self.create_subscription(
            Int32,
            '/hall_publisher',
            self.hall_callback,
            10
        )
        
        self.goal_publish_msg = Int32()
        self.is_goal_reached = 0
        
        self.pose1 = None
        self.pose2 = None
        self.hall_value = 0

        self.docked_pub = self.create_publisher(Bool, 'docked', 10)

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = 0.0
        self.cmd.angular.x = 0.0
        self.cmd.angular.y = 0.0
        self.cmd.angular.z = 0.0

        self.timer_ = self.create_timer(0.05, self.timer_callback)

    def pose1_callback(self, msg):
        self.pose1 = msg
        # self.get_logger().info('Received Pose1')

    def pose2_callback(self, msg):
        self.pose2 = msg
        # self.get_logger().info('Received Pose2')

    def hall_callback(self, msg):
        self.hall_value = msg.data

    def goal_callback(self, msg):
        self.is_goal_reached = msg.data

    def timer_callback(self):
        if self.is_goal_reached == 0 and self.goal_publish_msg.data == 0:
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            timestamp = self.get_clock().now().to_msg()
            goal_msg.header.stamp = timestamp
            goal_msg.pose.position.x = 0
            goal_msg.pose.position.y = 0
            goal_msg.pose.position.z = 0.0
            goal_msg.pose.orientation.z = 1.0
            goal_msg.pose.orientation.w = 0.0

            
            self.goal_publish_msg.data = 1

            self.publisher_.publish(goal_msg)
            self.publisher_2.publish(self.goal_publish_msg)
            self.get_logger().info('Published goal pose: x={}, y={}'.format(goal_msg.pose.position.x, goal_msg.pose.position.y))

        elif self.is_goal_reached == 1 and self.goal_publish_msg.data == 1:
            trans = [0.0, 0.0, 0.0]
            trans[0] = (self.pose1.position.z + self.pose2.position.z) / 2.0
            trans[1] = (- self.pose1.position.x - self.pose2.position.x) / 2.0
            trans[2] = (- self.pose1.position.y - self.pose2.position.y) / 2.0

            dist = math.sqrt(trans[0] ** 2 + trans[1] ** 2 + trans[2] ** 2)
            angular = 1.5 * math.atan2(trans[1], trans[0])
            linear = 0.1 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            print(dist)

            # if dist < 0.175:
            if self.hall_value > 2300:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                docked = Bool()
                docked.data = True
                self.publisher.publish(self.cmd)
                self.docked_pub.publish(docked)
                exit(0)
            # elif dist >= 0.175:
            elif self.hall_value < 2300:
                self.cmd.linear.x = linear
                self.cmd.angular.z = angular
                self.publisher.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()