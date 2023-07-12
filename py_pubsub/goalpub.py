import time
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32,Bool
from geometry_msgs.msg import PoseStamped, Pose, Twist
from action_msgs.msg import GoalStatus
from aruco_msgs.msg import MarkerArray
import math


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.goal_pose_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.publisher_2 = self.create_publisher(Int32, 'goal_published', 10)
        self.subscription1 = self.create_subscription(
            Int32,
            '/goal_reached',
            self.goal_callback,
            10
        )
        self.subscription1 = self.create_subscription(
            MarkerArray,
            '/marker_publisher/markers',
            self.aruco_pose_callback,
            10
        )
        self.subscription3 = self.create_subscription(
            Int32,
            '/hall_publisher',
            self.hall_callback,
            10
        )
        
        self.goal_publish_msg = Int32()
        self.goal_publish_msg.data = 0
        self.is_goal_reached = 0
        
        self.aruco_1_pose_x = 0.0
        self.aruco_1_pose_y = 0.0
        self.aruco_1_pose_z = 0.0
        self.aruco_2_pose_x = 0.0
        self.aruco_2_pose_y = 0.0
        self.aruco_2_pose_z = 0.0
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
        time.sleep(1)
        self.give_goal()
        self.timer_ = self.create_timer(0.05, self.timer_callback)

    def give_goal(self):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        timestamp = self.get_clock().now().to_msg()
        goal_msg.header.stamp = timestamp
        goal_msg.pose.position.x = 0.0
        goal_msg.pose.position.y = 0.0
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        print('goal published')
        self.goal_publish_msg.data = 1
        self.goal_pose_publisher.publish(goal_msg)

    def aruco_pose_callback(self, msg):    
        for marker in msg.markers:
            if marker.id == 582:
                self.aruco_1_pose_x = marker.pose.pose.position.x
                self.aruco_1_pose_y = marker.pose.pose.position.y
                self.aruco_1_pose_z = marker.pose.pose.position.z
            if marker.id == 236:
                self.aruco_2_pose_x = marker.pose.pose.position.x
                self.aruco_2_pose_y = marker.pose.pose.position.y
                self.aruco_2_pose_z = marker.pose.pose.position.z

    def hall_callback(self, msg):
        self.hall_value = msg.data

    def goal_callback(self, msg):
        self.is_goal_reached = msg.data

    def timer_callback(self):
        if self.is_goal_reached == 1 and self.goal_publish_msg.data == 1:
            trans = [0.0, 0.0, 0.0]
            trans[0] = (self.aruco_1_pose_z + self.aruco_2_pose_z) / 2.0
            trans[1] = (- self.aruco_1_pose_x - self.aruco_2_pose_x) / 2.0
            trans[2] = (- self.aruco_1_pose_y - self.aruco_2_pose_y) / 2.0

            dist = math.sqrt(trans[0] * 2 + trans[1] * 2 + trans[2] ** 2)
            angular = 1.5 * math.atan2(trans[1], trans[0])
            linear = 0.1 * math.sqrt(trans[0] * 2 + trans[1] * 2)
            print(dist)
            # print(linear)
            # print(angular)

            # if dist < 0.175:
            if self.hall_value > 2300 or dist < 0.63:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                docked = Bool()
                docked.data = True
                self.publisher.publish(self.cmd)
                self.docked_pub.publish(docked)
                print("Aruco Done!")
                exit(0)
            # elif dist >= 0.175:
            elif self.hall_value < 2300 or dist > 0.63:
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