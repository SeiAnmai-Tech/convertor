import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Undock(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = 0.0
        self.cmd.angular.x = 0.0
        self.cmd.angular.y = 0.0
        self.cmd.angular.z = 0.0
        self.count = 0
        self.timer_ = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        if self.count <= 100:
            self.cmd.linear.x = -0.1
            self.publisher.publish(self.cmd)
            self.count += 1
        else:
            exit(0)

def main(args=None):
    rclpy.init(args=args)
    undock = Undock()
    rclpy.spin(undock)
    undock.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()