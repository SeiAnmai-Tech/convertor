#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
from tf2_ros import TransformListener
from tf2_geometry_msgs import do_transform_pose
from move_base_msgs.action import MoveBase

import math
import time

class MoveBaseClient(Node):

    def __init__(self):
        super().__init__('movebase_client_py')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sharp_dis_pub = self.create_publisher(Int32, 'sharp_disable', 10)
        self.docked_pub = self.create_publisher(Bool, 'docked', 10)
        self.subscription = self.create_subscription(Int32, 'hall_publisher', self.mcbk, 10)
        self.tf_listener = TransformListener(self)

        self.tf_timer = self.create_timer(0.05, self.movebase_client)

        self.magnet = 0

        self.xpos = [0.0, -0.5]
        self.ypos = [0.0, 0.0]
        self.i = 0

    def mcbk(self, msg):
        self.magnet = msg.data

    def movebase_client(self):
        time.sleep(5)
        rate = self.create_rate(20)

        try:
            cmd = Twist()
            time.sleep(1)
            client = ActionClient(self, MoveBase, 'move_base')
            client.wait_for_server()

            goal_msg = MoveBase.Goal()
            goal_msg.target_pose.header.frame_id = 'map'
            goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.target_pose.pose.position.x = 0.0
            goal_msg.target_pose.pose.position.y = 0.0
            goal_msg.target_pose.pose.orientation.z = 0.0
            goal_msg.target_pose.pose.orientation.w = 1.0

            goal_handle = client.send_goal_async(goal_msg)

            rclpy.spin_until_future_complete(self, goal_handle)

            if goal_handle.result() is not None:
                goal_state = goal_handle.result().status
                print(goal_state)
                if goal_state == goal_handle.result().status.SUCCEEDED:
                    while rclpy.ok():
                        try:
                            (trans0, rot0) = self.tf_listener.lookup_transform('base_link', 'dock_visual_0', rclpy.time.Time())
                            (trans1, rot1) = self.tf_listener.lookup_transform('base_link', 'dock_visual_1', rclpy.time.Time())
                            trans = [0.0, 0.0, 0.0]
                            trans[0] = (trans1[0] + trans0[0]) / 2.0
                            trans[1] = (trans1[1] + trans0[1]) / 2.0
                            trans[2] = (trans1[2] + trans0[2]) / 2.0
                            dist = math.sqrt(trans[0] ** 2 + trans[1] ** 2 + trans[2] ** 2)
                            angular = 2.0 * math.atan2(trans[1], trans[0])
                            linear = 0.07 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
                            sharp_dis_msg = Int32()
                            sharp_dis_msg.data = 1
                            print(dist)
                            if dist < 0.555:
                                cmd.linear.x = linear
                                cmd.angular.z = angular
                                self.publisher.publish(cmd)
                                time.sleep(0.2)
                                cmd.linear.x = 0
                                cmd.angular.z = 0
                                docked = Bool()
                                docked.data = True
                                self.publisher.publish(cmd)
                                self.docked_pub.publish(docked)
                                return
                            elif dist >= 0.56:
                                cmd.linear.x = linear
                                cmd.angular.z = angular
                            self.publisher.publish(cmd)
                            self.sharp_dis_pub.publish(sharp_dis_msg)
                        except (tf2.TransformException):
                            cmd.linear.x = 0
                            cmd.angular.z = 0
                            continue
                        rate.sleep()
            else:
                self.get_logger().error('Action server not available!')
                self.get_logger().info('Navigation test finished.')
                return
        except (tf2.TransformException):
            print('mam')

def main(args=None):
    rclpy.init(args=args)
    move_base_client = MoveBaseClient()
    rclpy.spin(move_base_client)
    move_base_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
