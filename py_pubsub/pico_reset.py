import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class PicoResetSubscriber(Node):
    def __init__(self):
        super().__init__('pico_reset_subscriber')
        self.GPIO_PIN_TOP = 16  # Replace with the GPIO pin number you are using
        self.GPIO_PIN_BOTTOM = 26

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.GPIO_PIN_TOP, GPIO.OUT)  # Replace with the GPIO pin number you are using
        GPIO.output(self.GPIO_PIN_TOP, GPIO.HIGH)

        GPIO.setup(self.GPIO_PIN_BOTTOM, GPIO.OUT)  # Replace with the GPIO pin number you are using
        GPIO.output(self.GPIO_PIN_BOTTOM, GPIO.HIGH)

        self.subscription = self.create_subscription(
            Int32,
            'reset_docker',
            self.callback,
            10  # QoS profile depth
        )
        self.subscription

    def callback(self, msg):
        if msg.data == 1:
            GPIO.output(self.GPIO_PIN_TOP, GPIO.LOW)  # Set the pin to low (GND)
            self.get_logger().info('Resetting Pi Pico top')
            GPIO.output(self.GPIO_PIN_BOTTOM, GPIO.LOW)  # Set the pin to low (GND)
            self.get_logger().info('Resetting Pi Pico bottom')
            time.sleep(0.5)  # Keep the pin low for 0.5 seconds
            GPIO.output(self.GPIO_PIN_TOP, GPIO.HIGH)  # Set the pin to high (3.3V)
            self.get_logger().info('Reset top pico complete')
            GPIO.output(self.GPIO_PIN_BOTTOM, GPIO.HIGH)  # Set the pin to high (3.3V)
            self.get_logger().info('Reset bottom pico complete')

def main(args=None):

    rclpy.init(args=args)
    node = PicoResetSubscriber()
    rclpy.spin(node)

    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
