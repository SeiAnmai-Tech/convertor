import rclpy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
from math import radians

pan_servo_angle = 0
tilt_servo_angle = 0

def servo_positions_callback(msg):
    global pan_servo_angle, tilt_servo_angle
    servo_angles = msg.data
    if len(servo_angles) >= 2:
        pan_servo_angle = servo_angles[0]
        tilt_servo_angle = servo_angles[1]

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('joint_updater')
    joint_pub = node.create_publisher(JointState, 'joint_states', 1000)
    servo_sub = node.create_subscription(Int32MultiArray, 'servo_positions', servo_positions_callback, 1000)
    loop_rate = node.create_rate(50)
    count = 0

    while rclpy.ok():
        state = JointState()
        state.header.stamp = node.get_clock().now().to_msg()
        state.header.seq = count
        state.name = ["camera_joint_1", "camera_joint_2"]
        state.position = [0.0, 0.0]

        pan_servo_angle_rad = radians(pan_servo_angle)
        tilt_servo_angle_rad = radians(tilt_servo_angle)

        state.position[0] = -pan_servo_angle_rad
        state.position[1] = -tilt_servo_angle_rad

        joint_pub.publish(state)
        rclpy.spin_once(node)
        loop_rate.sleep()
        count += 1

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
