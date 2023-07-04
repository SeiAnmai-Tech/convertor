import sys
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QSlider, QVBoxLayout, QWidget
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher1_ = self.create_publisher(Float32MultiArray, 'motor_rps', 10)
        self.publisher2_ = self.create_publisher(Float32MultiArray, 'pid', 10)
        self.slider1 = None
        self.slider2 = None
        self.slider3 = None
        self.slider4 = None
        self.slider5 = None

    def publish_messages(self):
        if (
            self.slider1 is not None
            and self.slider2 is not None
            and self.slider3 is not None
            and self.slider4 is not None
            and self.slider5 is not None
        ):
            # Publish array of 2 float numbers on motor_rps
            msg1 = Float32MultiArray()
            msg1.data = [self.slider1.value() / 100.0, -self.slider2.value() / 100.0]
            self.publisher1_.publish(msg1)
            self.get_logger().info('Publishing on motor_rps: "%s"' % msg1.data)

            # Publish array of 3 float numbers on pid
            msg2 = Float32MultiArray()
            msg2.data = [
                self.slider3.value() / 100.0,
                self.slider4.value() / 100.0,
                self.slider5.value() / 100.0
            ]
            self.publisher2_.publish(msg2)
            self.get_logger().info('Publishing on pid: "%s"' % msg2.data)

    def close(self):
        self.destroy_node()
        rclpy.shutdown()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Float Value Control")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout(central_widget)

        self.slider1 = self.create_slider(layout, "left_motor_rps", -100, 100)
        self.slider2 = self.create_slider(layout, "right_motor_rps", -100, 100)
        self.slider3 = self.create_slider(layout, "Kp", 0, 50000)
        self.slider4 = self.create_slider(layout, "Ki", 0, 2000)
        self.slider5 = self.create_slider(layout, "Kd", 0, 2000)

        # Initialize ROS node and publisher
        rclpy.init()
        self.node = MinimalPublisher()
        self.node.slider1 = self.slider1
        self.node.slider2 = self.slider2
        self.node.slider3 = self.slider3
        self.node.slider4 = self.slider4
        self.node.slider5 = self.slider5

        self.node.get_logger().info("Node initialized.")

        self.timer = QTimer()
        self.timer.timeout.connect(self.node.publish_messages)
        self.timer.start(20)  # Spin the node every 20 milliseconds / 50 Hz

    def create_slider(self, layout, label_text, min_value, max_value):
        label = QLabel(label_text)
        slider = QSlider()
        slider.setOrientation(1)
        slider.setMinimum(min_value)
        slider.setMaximum(max_value)
        slider.setTickInterval(1)
        slider.setTickPosition(QSlider.TicksRight)

        layout.addWidget(label)
        layout.addWidget(slider)

        return slider

    def closeEvent(self, event):
        self.timer.stop()
        self.node.close()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()



# import rclpy
# import random
# from rclpy.node import Node

# from std_msgs.msg import Float32MultiArray

# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('minimal_publisher')
#         self.publisher1_ = self.create_publisher(Float32MultiArray, 'motor_rps', 10)
#         self.publisher2_ = self.create_publisher(Float32MultiArray, 'pid', 10)
#         timer_period = 5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)

#     def timer_callback(self):
#         # Publish array of 2 float numbers on motor_rps
#         msg1 = Float32MultiArray()
#         msg1.data = [-0.32, -0.32]  # Modify the array elements as per your requirements
#         self.publisher1_.publish(msg1)
#         self.get_logger().info('Publishing on motor_rps: "%s"' % msg1.data)

#         # Publish array of 3 float numbers on pid
#         msg2 = Float32MultiArray()
#         msg2.data = [10, 0.0, 0.5]  # Modify the array elements as per your requirements
#         self.publisher2_.publish(msg2)
#         self.get_logger().info('Publishing on pid: "%s"' % msg2.data)


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_publisher = MinimalPublisher()

#     rclpy.spin(minimal_publisher)

#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()




# import tkinter as tk
# from std_msgs.msg import Float32MultiArray
# import rclpy
# from rclpy.node import Node


# class MinimalPublisher(Node):
#     def __init__(self):
#         super().__init__('minimal_publisher')
#         self.publisher1_ = self.create_publisher(Float32MultiArray, 'motor_rps', 10)
#         self.publisher2_ = self.create_publisher(Float32MultiArray, 'pid', 10)
#         timer_period = 1  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.slider1 = None
#         self.slider2 = None
#         self.slider3 = None
#         self.slider4 = None
#         self.slider5 = None

#     def timer_callback(self):
#         if (
#             self.slider1 is not None
#             and self.slider2 is not None
#             and self.slider3 is not None
#             and self.slider4 is not None
#             and self.slider5 is not None
#         ):
#             # Publish array of 2 float numbers on motor_rps
#             msg1 = Float32MultiArray()
#             msg1.data = [self.slider1.get(), self.slider2.get()]
#             self.publisher1_.publish(msg1)
#             self.get_logger().info('Publishing on motor_rps: "%s"' % msg1.data)

#             # Publish array of 3 float numbers on pid
#             msg2 = Float32MultiArray()
#             msg2.data = [self.slider3.get(), self.slider4.get(), self.slider5.get()]
#             self.publisher2_.publish(msg2)
#             self.get_logger().info('Publishing on pid: "%s"' % msg2.data)


# class App:
#     def __init__(self):
#         self.root = tk.Tk()
#         self.root.title("Float Value Control")

#         # Create sliders
#         self.slider1 = self.create_slider("left_motor_rps", -2.0, 2.0)
#         self.slider2 = self.create_slider("right_motor_rps", -2.0, 2.0)
#         self.slider3 = self.create_slider("Kp", 0.0, 200.0)
#         self.slider4 = self.create_slider("Ki", -1.0, 1.0)
#         self.slider5 = self.create_slider("Kd", -30.0, 30.0)

#         # Initialize ROS node and publisher
#         rclpy.init()
#         self.node = MinimalPublisher()
#         self.node.slider1 = self.slider1
#         self.node.slider2 = self.slider2
#         self.node.slider3 = self.slider3
#         self.node.slider4 = self.slider4
#         self.node.slider5 = self.slider5

#     def create_slider(self, label_text, min_value, max_value):
#         slider_frame = tk.Frame(self.root)
#         slider_frame.pack()

#         label = tk.Label(slider_frame, text=label_text)
#         label.pack(side=tk.LEFT)

#         slider = tk.Scale(
#             slider_frame,
#             from_=min_value,
#             to=max_value,
#             resolution=0.01,
#             orient=tk.HORIZONTAL,
#             length=200
#         )
#         slider.pack(side=tk.LEFT)

#         return slider

#     def run(self):
#         self.root.mainloop()

#     def destroy(self):
#         self.node.destroy_node()
#         rclpy.shutdown()


# def main():
#     app = App()
#     app.run()
#     app.destroy()


# if __name__ == '__main__':
#     main()


# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# import rclpy
# import random
# from rclpy.node import Node

# from std_msgs.msg import Float32


# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('minimal_publisher')
#         self.publisher_ = self.create_publisher(Float32, 'talker', 10)
#         timer_period = 5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0

#     def timer_callback(self):
#         msg = Float32()
#         #msg.data = random.uniform(-5, 5)
#         msg.data = -0.32
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%f"' % msg.data)


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_publisher = MinimalPublisher()

#     rclpy.spin(minimal_publisher)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
