#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class MotorRpsPublisher(Node):
    def __init__(self):
        super().__init__('motor_rps_publisher')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_rps', 10)

    def cmd_vel_callback(self, msg):
        motor_rps = Float32MultiArray()
        left_rps_r = ( ( 2 * msg.linear.x ) + ( 0.315 * (-msg.angular.z) ) ) / ( 2 * 2 * 3.14 * 0.05 )
        right_rps_r = - ( ( 2 * msg.linear.x ) - ( 0.315 * (-msg.angular.z) ) ) / ( 2 * 2 * 3.14 * 0.05 )
        motor_rps.data = [left_rps_r, right_rps_r]
        self.publisher.publish(motor_rps)

def main(args=None):
    rclpy.init(args=args)
    motor_rps_publisher = MotorRpsPublisher()
    rclpy.spin(motor_rps_publisher)
    motor_rps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
