import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class SharpSensorPublisher(Node):
    def __init__(self):
        super().__init__('sharp_sensor_publisher')
        self.left_publisher_ = self.create_publisher(Range, 'left_sharp', 10)
        self.left_subscription_ = self.create_subscription(
            Float32,
            'left_sharp_publisher',
            self.left_sharp_sensor_callback,
            10
        )

        self.right_publisher_ = self.create_publisher(Range, 'right_sharp', 10)
        self.right_subscription_ = self.create_subscription(
            Float32,
            'right_sharp_publisher',
            self.right_sharp_sensor_callback,
            10
        )

        self.floor_val = 0.11 # (in meters) distance from sharp sensor to ground
        self.floor_thresh = 0.03

        self.left_sharp_dist = 0.0
        self.left_prev_ir = 0.0
        self.left_prev_prev_ir = 0.0
        self.left_sharp_flag = 0
        self.left_range_msg = Range()
        self.left_range_msg.header.frame_id = "left_sharp"
        self.left_range_msg.radiation_type = Range.INFRARED
        self.left_range_msg.field_of_view = 1.57/2
        self.left_range_msg.min_range = 0.1
        self.left_range_msg.max_range = 0.8
        self.left_range_msg.range = float('inf')

        self.right_sharp_dist = 0.0
        self.right_prev_ir = 0.0
        self.right_prev_prev_ir = 0.0
        self.right_sharp_flag = 0
        self.right_range_msg = Range()
        self.right_range_msg.header.frame_id = "right_sharp"
        self.right_range_msg.radiation_type = Range.INFRARED
        self.right_range_msg.field_of_view = 1.57/2
        self.right_range_msg.min_range = 0.1
        self.right_range_msg.max_range = 0.8
        self.right_range_msg.range = float('inf')

    def left_sharp_sensor_callback(self, msg):

        self.left_sharp_dist = msg.data/100

        if (abs(self.floor_val - self.left_sharp_dist) > self.floor_thresh and abs(self.floor_val - self.left_prev_ir) > self.floor_thresh and abs(self.floor_val - self.left_prev_prev_ir) > self.floor_thresh):
            # Update the costmap with the IR range data
            self.left_range_msg.header.stamp = self.get_clock().now().to_msg()
            self.left_range_msg.range = self.sharp_dist #add the distance in meters to give as obstacle
            self.left_sharp_flag = 1

        else:
            # IR range is within fixed distance, do not update the costmap
            self.left_sharp_flag = 0
            self.left_range_msg.range = float('inf')

        self.left_prev_prev_ir = self.left_prev_ir
        self.left_prev_ir = self.left_sharp_dist
        self.left_publisher_.publish(left_range_msg)

    def right_sharp_sensor_callback(self, msg):

        self.right_sharp_dist = msg.data/100

        if (abs(self.floor_val - self.right_sharp_dist) > self.floor_thresh and abs(self.floor_val - self.right_prev_ir) > self.floor_thresh and abs(self.floor_val - self.right_prev_prev_ir) > self.floor_thresh):
            # Update the costmap with the IR range data
            self.right_range_msg.header.stamp = self.get_clock().now().to_msg()
            self.right_range_msg.range = self.sharp_dist #add the distance in meters to give as obstacle
            self.right_sharp_flag = 1

        else:
            # IR range is within fixed distance, do not update the costmap
            self.right_sharp_flag = 0
            self.right_range_msg.range = float('inf')

        self.right_prev_prev_ir = self.right_prev_ir
        self.right_prev_ir = self.right_sharp_dist
        self.right_publisher_.publish(right_range_msg)

def main(args=None):
    rclpy.init(args=args)
    sharp_sensor_publisher = SharpSensorPublisher()
    rclpy.spin(sharp_sensor_publisher)
    sharp_sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
