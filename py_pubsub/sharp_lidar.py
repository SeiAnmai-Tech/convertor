import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from builtin_interfaces.msg import Time

class LaserScanPublisher(Node):
    def __init__(self):
        super().__init__('laser_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'test_scan', 10)
        timer_period = 0.2  # 1 second
        self.timer = self.create_timer(timer_period, self.publish_laser_scan)
        self.left_subscription_ = self.create_subscription(
            Int32,
            'left_sharp_publisher',
            self.left_sharp_sensor_callback,
            10
        )
        self.right_subscription_ = self.create_subscription(
            Int32,
            'right_sharp_publisher',
            self.right_sharp_sensor_callback,
            10
        )

        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = 'base_link'  # Change the frame ID if needed
        self.scan_msg.angle_min = -0.5  # Start angle of the scan [rad]
        self.scan_msg.angle_max = 0.5  # End angle of the scan [rad]
        self.scan_msg.angle_increment = 0.05  # Angular distance between measurements [rad]
        self.scan_msg.time_increment = 0.0  # Time between measurements [s]
        self.scan_msg.scan_time = 0.0  # Time taken to complete one scan [s]
        self.scan_msg.range_min = 0.1  # Minimum range value [m]
        self.scan_msg.range_max = 10.0  # Maximum range value [m]

        self.floor_val = 0.28 # (in meters) distance from sharp sensor to ground
        self.floor_thresh = 0.04

        self.left_sharp_dist = 0.0
        self.left_prev_ir = 0.0
        self.left_prev_prev_ir = 0.0
        self.left_sharp_flag = 0
        self.left_val = float('inf')

        self.right_sharp_dist = 0.0
        self.right_prev_ir = 0.0
        self.right_prev_prev_ir = 0.0
        self.right_sharp_flag = 0
        self.right_val = float('inf')

    def left_sharp_sensor_callback(self, msg):

        self.left_sharp_dist = msg.data/100

        if (abs(self.floor_val - self.left_sharp_dist) > self.floor_thresh and abs(self.floor_val - self.left_prev_ir) > self.floor_thresh and abs(self.floor_val - self.left_prev_prev_ir) > self.floor_thresh):
            # Update the costmap with the IR range data
            
            self.left_sharp_flag = 1
            self.left_val = 0.35

        else:
            # IR range is within fixed distance, do not update the costmap
            self.left_sharp_flag = 0
            self.left_val = float('inf')

        self.left_prev_prev_ir = self.left_prev_ir
        self.left_prev_ir = self.left_sharp_dist

    def right_sharp_sensor_callback(self, msg):

        self.right_sharp_dist = msg.data/100

        if (abs(self.floor_val - self.right_sharp_dist) > self.floor_thresh and abs(self.floor_val - self.right_prev_ir) > self.floor_thresh and abs(self.floor_val - self.right_prev_prev_ir) > self.floor_thresh):
            # Update the costmap with the IR range data
            self.right_val = 0.35
            self.right_sharp_flag = 1

        else:
            # IR range is within fixed distance, do not update the costmap
            self.right_val = float('inf')
            self.right_sharp_flag = 0

        self.right_prev_prev_ir = self.right_prev_ir
        self.right_prev_ir = self.right_sharp_dist

    def publish_laser_scan(self):

        # Set the timestamp in the header
        self.scan_msg.header.stamp = self.get_clock().now().to_msg()

        self.scan_msg.ranges =  ([self.right_val] * 10) + ([self.left_val] * 10) # Generate an array of 20 values, all set to 1.0

        self.publisher_.publish(self.scan_msg)
        # self.get_logger().info('Published LaserScan data')

def main(args=None):
    rclpy.init(args=args)
    laser_scan_publisher = LaserScanPublisher()
    rclpy.spin(laser_scan_publisher)
    laser_scan_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()