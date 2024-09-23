import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('ros2_publisher_camera')
        self.publisher = self.create_publisher(Image, '/racecar/raspicam_node/image', 10)
        self.bridge = CvBridge()

    def publish_image(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not cap.isOpened():
            self.get_logger().error("Could NOT open camera.")
            exit(-1)
        while cap.isOpened() and rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().error("Could NOT read from camera.")
                break
            else:
                # Convert the OpenCV image to a ROS Image message
                ros_compressed_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
                # Publish the ROS Image message
                self.publisher.publish(ros_compressed_image_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    camera_publisher.publish_image()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

