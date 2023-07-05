import rclpy
from nav_msgs.srv import GetMap
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import MapMetaData  # Replace `your_package` with the actual package name
import cv2
import numpy as np

def display_map(map_data, publisher_compressed, publisher_metadata):
    # Convert occupancy grid to numpy array
    width = map_data.info.width
    height = map_data.info.height
    print(width,height)
    data = map_data.data
    map_np = np.array(data).reshape((height, width))

    # Convert to binary image
    map_binary = np.where(map_np == 0, 0, 255).astype(np.uint8)
    map_binary = np.flip(map_binary, axis=1)
    print(map_binary.shape)

    # Compress the binary map using OpenCV
    _, compressed_data = cv2.imencode('.jpg', map_binary)
    compressed_bytes = compressed_data.tobytes()

    # Create a CompressedImage message
    compressed_image_msg = CompressedImage()
    compressed_image_msg.format = 'jpeg'
    compressed_image_msg.data = compressed_bytes

    # Publish the compressed image
    publisher_compressed.publish(compressed_image_msg)

    # Create a MapMetaData message
    metadata_msg = MapMetaData()
    metadata_msg.height = height
    metadata_msg.width = width
    metadata_msg.resolution = map_data.info.resolution
    metadata_msg.origin = map_data.info.origin

    # Publish the map metadata
    publisher_metadata.publish(metadata_msg)

def main(args=None):
    rclpy.init(args=args)

    # Create a ROS 2 node
    node = rclpy.create_node('map_viewer')

    # Create a client for the map service
    client = node.create_client(GetMap, '/map_server/map')

    # Create a publisher for the compressed image
    publisher_compressed = node.create_publisher(CompressedImage, '/compressed_map', 10)

    # Create a publisher for the map metadata
    publisher_metadata = node.create_publisher(MapMetaData, '/map_metadata', 10)

    # Wait for the service to become available
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')

    # Create a request message for the GetMap service
    request = GetMap.Request()

    try:
        # Call the GetMap service and get the response
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            # Display the map and publish the compressed image and map metadata
            timer_period = 1.0 / 10  # 10Hz
            timer = node.create_timer(timer_period, lambda: display_map(future.result().map, publisher_compressed, publisher_metadata))

            rclpy.spin(node)

    except KeyboardInterrupt:
        # Keyboard interrupt (Ctrl+C) detected
        node.get_logger().info('Keyboard interrupt detected. Shutting down...')

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()