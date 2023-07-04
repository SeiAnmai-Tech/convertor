#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
from math import radians

class ServoJointState(Node):
    def __init__(self):
        super().__init__('servo_joints')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'servo_positions',
            self.servo_positions_callback,
            10
        )
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 1000)
        self.tf_timer = self.create_timer(0.05, self.joint_state_publisher)
        self.pan_servo_angle = 0
        self.tilt_servo_angle = 0

    def servo_positions_callback(self, msg):
        servo_angles = msg.data
        if len(servo_angles) >= 2:
            self.pan_servo_angle = servo_angles[0]
            self.tilt_servo_angle = servo_angles[1]
            # print(pan_servo_angle)

    def joint_state_publisher(self):
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = ["camera_joint_1", "camera_joint_2"]
        state.position = [0.0, 0.0]

        pan_servo_angle_rad = radians(self.pan_servo_angle)
        tilt_servo_angle_rad = radians(self.tilt_servo_angle)

        state.position[0] = -pan_servo_angle_rad
        state.position[1] = -tilt_servo_angle_rad

        self.joint_pub.publish(state)

def main(args=None):
    rclpy.init(args=args)
    node = ServoJointState()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
