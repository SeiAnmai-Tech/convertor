#! /usr/bin/env python3
import time  # Time library
from enum import Enum
import rclpy
from rclpy.duration import Duration  # Handles time for ROS 2
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped  # Pose with ref frame and timestamp
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient  # Add this line to import ActionClient
from std_msgs.msg import Int32

class NavigationResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

class BasicNavigator(Node):
    def __init__(self):
        super().__init__('basic_navigator')
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        self.initial_pose_received = False
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseStamped, 'initialpose', 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal_pose', self.goalCallback, 10)

        self.goal_given = 0

        self.publisher_ = self.create_publisher(Int32, 'goal_reached', 10)
        self.is_goal_reached = Int32()

        #self.nav_to_pose_client.wait_for_service()
        #self.wait_for_service('initialpose')

    def goToPose(self, pose):
        # Sends a `NavToPose` action request
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                      str(pose.pose.position.y) + '...')
        print("A")
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   feedback_callback = self._feedbackCallback)
        print("B")
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        # rclpy.spin_until_future_complete(self, self.send_goal_future)
        # print("C")
        # self.goal_handle = self.send_goal_future.result()
        # print("D")

        # if not self.goal_handle.accepted:
        #     self.get_logger().error('Goal to ' + str(pose.pose.position.x) + ' ' +
        #                    str(pose.pose.position.y) + ' was rejected!')
        #     return False
        # print("E")
        # self.result_future = self.goal_handle.get_result_async()
        # print("F")
        # return True
    
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded! ')
            self.is_goal_reached.data = 1
            self.publisher_.publish(self.is_goal_reached)
            
        else:
            self.get_logger().info(
                'Navigation failed with status: {0}'.format(status))

    def _feedbackCallback(self, msg):
        # print("feedback is  coming")
        self.feedback = msg.feedback
        return

    def cancelNav(self):
        self.get_logger().info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was canceled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().debug('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.get_logger().debug('Goal succeeded!')
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return NavigationResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return NavigationResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return NavigationResult.CANCELED
        else:
            return NavigationResult.UNKNOWN

    def goalCallback(self, msg):
        self.get_logger().info('Received goal pose: ' + str(msg.pose.position.x) + ' ' +
                          str(msg.pose.position.y) + ' ' + str(msg.pose.position.z))
        self.goal_given = 1
        self.goToPose(msg)
        

    def _setInitialPose(self):
        msg = PoseStamped()
        msg.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.get_logger().info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)

def main():
    rclpy.init()
    navigator = BasicNavigator()
    executor = MultiThreadedExecutor()
    executor.add_node(navigator)

    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)

    executor.shutdown()
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
