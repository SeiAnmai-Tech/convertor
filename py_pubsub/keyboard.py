import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Int32MultiArray


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_publisher_ = self.create_publisher(Int32MultiArray, '/servo_positions', 10)
        self.reset_publisher_ = self.create_publisher(Int32, '/reset_ticks', 10)
        self.reset_docker_ = self.create_publisher(Int32, '/reset_docker', 10)
        self.timer_ = self.create_timer(0.067, self.publish_twist)  # Publishing rate of 20Hz
        self.servo_timer_ = self.create_timer(0.01, self.publish_servo)   # Publishing rate of 100Hz
        self.twist_msg_ = Twist()
        self.servo_msg = Int32MultiArray()
        self.servo_msg.data = [0, 0]
        self.battery_subscriber_ = self.create_subscription(Int32, 'battery_publisher', self.battery_callback, 10)
        self.battery_percentage = 0
        self.is_w_pressed = False
        self.is_a_pressed = False
        self.is_s_pressed = False
        self.is_d_pressed = False
        self.is_r_pressed = False
        self.is_g_pressed = False
        self.is_r_released = True
        self.is_g_released = True
        self.is_up_pressed = False
        self.is_down_pressed = False
        self.is_left_pressed = False
        self.is_right_pressed = False
        self.is_space_pressed = False

        rclpy.parameter.Parameter()

    def publish_twist(self):
        if self.is_w_pressed or self.is_a_pressed or self.is_s_pressed or self.is_d_pressed:
            self.twist_msg_.linear.x = 0.15 if self.is_w_pressed else (-0.15 if self.is_s_pressed else 0.0)
            self.twist_msg_.angular.z = 0.4 if self.is_a_pressed else (-0.4 if self.is_d_pressed else 0.0)
            self.publisher_.publish(self.twist_msg_)

    def publish_servo(self):
        if self.is_up_pressed or self.is_down_pressed or self.is_left_pressed or self.is_right_pressed or self.is_space_pressed:
            self.servo_msg.data[0] += -1 if self.is_up_pressed else (1 if self.is_down_pressed else 0)
            self.servo_msg.data[1] += 1 if self.is_left_pressed else (-1 if self.is_right_pressed else 0)
            if self.is_space_pressed:
                self.servo_msg.data = [0, 0] 
            if self.servo_msg.data[0]>60:
                self.servo_msg.data[0]=60
            elif self.servo_msg.data[0]<-60:
                self.servo_msg.data[0]=-60
            elif self.servo_msg.data[1]>60:
                self.servo_msg.data[1]=60
            elif self.servo_msg.data[1]<-60:
                self.servo_msg.data[1]=-60
            self.servo_publisher_.publish(self.servo_msg)

    def publish_reset_ticks(self):
        reset_msg = Int32()
        if self.is_r_pressed and self.is_r_released:
            reset_msg.data = 1
            self.reset_publisher_.publish(reset_msg)
            self.is_r_released = False
        elif not self.is_r_pressed:
            reset_msg.data = 0
            self.is_r_released = True

    def publish_reset_docker(self):
        reset_docker_msg = Int32()
        if self.is_g_pressed and self.is_g_released:
            reset_docker_msg.data = 1
            self.reset_docker_.publish(reset_docker_msg)
            self.is_g_released = False
        elif not self.is_g_pressed:
            reset_docker_msg.data = 0
            self.is_g_released = True

    def battery_callback(self, battery_msg):
        self.battery_percentage = battery_msg.data

    def display_screen(self, screen):
        battery_width = 100
        battery_height = 20
        battery_level = int(battery_width * self.battery_percentage / 100)

        screen.fill((255, 255, 255))

        # Draw the empty battery bar
        pygame.draw.rect(screen, (255, 0, 0), (50, 50, battery_width, battery_height), 1)

        # Draw the filled battery level
        pygame.draw.rect(screen, (0, 255, 0), (50, 50, battery_level, battery_height))

        # Display the battery percentage text
        font = pygame.font.Font(None, 24)
        text = font.render(f'Battery: {self.battery_percentage}%', True, (0, 0, 0))
        screen.blit(text, (50, 30))

        # Display the control message
        control_font = pygame.font.Font(None, 20)
        control_text = control_font.render("Control TOTO!", True, (0, 0, 0))
        screen.blit(control_text, (50, 100))

        message = """
        -----------------------------
        Moving robot:
        w: Move Forward
        s: Move Backward
        a: Turn Left
        d: Turn Right
    
        Moving camera:
        Up Arrow:    Up 
        Down Arrow:  Down 
        Left Arrow:  Left 
        Right Arrow: Right 
        
        Space: Reset Camera Posi
        r: Reset Encoder Ticks
        g: Reset Docker for Pico
        -----------------------------
        """

        control_y = 100
        control_lines = message.split('\n')
        for line in control_lines:
            control_text = control_font.render(line.strip(), True, (0, 0, 0))
            screen.blit(control_text, (50, control_y))
            control_y += 20

    def run(self):
        pygame.init()
        screen_width = 250
        screen_height = 480
        screen = pygame.display.set_mode((screen_width, screen_height))
        pygame.display.set_caption('Keyboard Publisher')
        clock = pygame.time.Clock()

        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_w:
                        self.is_w_pressed = True
                    elif event.key == pygame.K_a:
                        self.is_a_pressed = True
                    elif event.key == pygame.K_s:
                        self.is_s_pressed = True
                    elif event.key == pygame.K_d:
                        self.is_d_pressed = True
                    elif event.key == pygame.K_r:
                        self.is_r_pressed = True
                    elif event.key == pygame.K_g:
                        self.is_g_pressed = True
                    elif event.key == pygame.K_UP:
                        self.is_up_pressed = True
                    elif event.key == pygame.K_DOWN:
                        self.is_down_pressed = True
                    elif event.key == pygame.K_LEFT:
                        self.is_left_pressed = True
                    elif event.key == pygame.K_RIGHT:
                        self.is_right_pressed = True
                    elif event.key == pygame.K_SPACE:
                        self.is_space_pressed = True
                    
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_w:
                        self.is_w_pressed = False
                    elif event.key == pygame.K_a:
                        self.is_a_pressed = False
                    elif event.key == pygame.K_s:
                        self.is_s_pressed = False
                    elif event.key == pygame.K_d:
                        self.is_d_pressed = False
                    elif event.key == pygame.K_r:
                        self.is_r_pressed = False
                    elif event.key == pygame.K_g:
                        self.is_g_pressed = False
                    elif event.key == pygame.K_UP:
                        self.is_up_pressed = False
                    elif event.key == pygame.K_DOWN:
                        self.is_down_pressed = False
                    elif event.key == pygame.K_LEFT:
                        self.is_left_pressed = False
                    elif event.key == pygame.K_RIGHT:
                        self.is_right_pressed = False
                    elif event.key == pygame.K_SPACE:
                        self.is_space_pressed = False

            rclpy.spin_once(self)

            self.publish_reset_ticks()
            self.publish_reset_docker()

            self.display_screen(screen)
            pygame.display.update()

            clock.tick(60)  # Increase frame rate


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
