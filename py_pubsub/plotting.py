import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

class RealtimePlotter(Node):
    def __init__(self):
        super().__init__('realtime_plotter')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'measured_rps',  # replace 'topic_name' with your topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.fig, self.ax = plt.subplots()
        self.x_data = []
        self.y_data = []
        self.z_data = []  # for the 1st element
        print('getting here')

        plt.ion()
        plt.show()

    def listener_callback(self, msg):
        self.y_data.append(msg.data[0])
        self.z_data.append(msg.data[1])  # append the 1st element
        self.x_data.append(len(self.y_data))

        # Keep only the last 100 elements of the arrays
        self.x_data_s = self.x_data[-100:]
        self.y_data_s = self.y_data[-100:]
        self.z_data_s = self.z_data[-100:]

        self.ax.clear()
        self.ax.plot(self.x_data_s, self.y_data_s)
        self.ax.plot(self.x_data_s, self.z_data_s)  # plot the 1st element
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)

    realtime_plotter = RealtimePlotter()

    rclpy.spin(realtime_plotter)

    realtime_plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()