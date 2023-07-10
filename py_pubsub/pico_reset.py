import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class PicoResetSubscriber(Node):
    def __init__(self):
        super().__init__('pico_reset_subscriber')
        self.GPIO_PIN = 16  # Replace with the GPIO pin number you are using
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.GPIO_PIN, GPIO.OUT)  # Replace with the GPIO pin number you are using
        GPIO.output(self.GPIO_PIN, GPIO.HIGH)
        self.subscription = self.create_subscription(
            Int32,
            'reset_docker',
            self.callback,
            10  # QoS profile depth
        )
        self.subscription

    def callback(self, msg):
        if msg.data == 1:
            GPIO.output(self.GPIO_PIN, GPIO.LOW)  # Set the pin to low (GND)
            self.get_logger().info('Resetting Pi Pico')
            time.sleep(1)  # Keep the pin low for 1 second
            GPIO.output(self.GPIO_PIN, GPIO.HIGH)  # Set the pin to high (3.3V)
            self.get_logger().info('Reset complete')

def main(args=None):

    rclpy.init(args=args)
    node = PicoResetSubscriber()
    rclpy.spin(node)

    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
