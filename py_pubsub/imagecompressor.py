import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge

class ImageToCompressedImageNode(Node):
    def __init__(self):
        super().__init__('image_to_compressed_image_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            CompressedImage,
            'compressed_image_topic',
            10
        )

    def image_callback(self, image_msg):
        # Convert Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        # Compress the image using a suitable compression algorithm (e.g., JPEG)
        _, compressed_data = cv2.imencode('.jpg', cv_image)

        # Create a CompressedImage message
        compressed_image_msg = CompressedImage()
        compressed_image_msg.format = 'jpeg'
        compressed_image_msg.data = compressed_data.tobytes()

        # Publish the compressed image
        self.publisher.publish(compressed_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageToCompressedImageNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()