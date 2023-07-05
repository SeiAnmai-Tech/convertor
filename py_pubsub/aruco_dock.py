from rclpy.duration import Duration # Handles time for ROS 2
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
import math
import time

class ArucoDock(Node):
    def __init__(self):
        super().__init__(node_name='aruco_dock')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.docked_pub = self.create_publisher(Bool, 'docked', 10)
        self.tf_buffer = Buffer(Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = 0.0
        self.cmd.angular.x = 0.0
        self.cmd.angular.y = 0.0
        self.cmd.angular.z = 0.0

    def timer_callback(self):
        try:
            transform0 = self.tf_buffer.lookup_transform('base_link', 'dock_visual_0', rclpy.time.Time())
            transform1 = self.tf_buffer.lookup_transform('base_link', 'dock_visual_1', rclpy.time.Time())

            trans0 = transform0.transform.translation
            trans1 = transform1.transform.translation

            trans = [0.0, 0.0, 0.0]
            trans[0] = (trans1.x + trans0.x) / 2.0
            trans[1] = (trans1.y + trans0.y) / 2.0
            trans[2] = (trans1.z + trans0.z) / 2.0

            dist = math.sqrt(trans[0] ** 2 + trans[1] ** 2 + trans[2] ** 2)
            angular = 2.0 * math.atan2(trans[1], trans[0])
            linear = 0.07 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            print(dist)

            if dist < 0.53:
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
                return
            elif dist >= 0.53:
                self.cmd.linear.x = linear
                self.cmd.angular.z = angular
                self.publisher.publish(self.cmd)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDock()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()