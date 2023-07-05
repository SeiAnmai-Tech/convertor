import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
import tf2_ros
import math
import tf2_geometry_msgs


class MapToBaseLinkPublisher(Node):
    def __init__(self):
        super().__init__('map_to_base_link_publisher')
        self.publisher_x = self.create_publisher(Float64, 'map_to_base_link/x', 10)
        self.publisher_y = self.create_publisher(Float64, 'map_to_base_link/y', 10)
        self.publisher_yaw = self.create_publisher(Float64, 'map_to_base_link/yaw', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.02, self.publish_transform)

    def publish_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(str(e))
            return

        # Extract x, y, and yaw from the transform
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        yaw = self.quaternion_to_yaw(transform.transform.rotation)

        # Publish x value
        msg_x = Float64()
        msg_x.data = x
        self.publisher_x.publish(msg_x)

        # Publish y value
        msg_y = Float64()
        msg_y.data = y
        self.publisher_y.publish(msg_y)

        # Publish yaw value
        msg_yaw = Float64()
        msg_yaw.data = yaw
        self.publisher_yaw.publish(msg_yaw)
        print(x,y,yaw)

    @staticmethod
    def quaternion_to_yaw(quaternion):
        """
        Convert quaternion to yaw angle in radians. 
        """
        #print(quaternion)
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return yaw

def main(args=None):
    rclpy.init(args=args)
    map_to_base_link_publisher = MapToBaseLinkPublisher()
    rclpy.spin(map_to_base_link_publisher)
    map_to_base_link_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()