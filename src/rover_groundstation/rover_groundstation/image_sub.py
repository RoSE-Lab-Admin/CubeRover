import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageSub(Node):
    def __init__(self):
        super().__init__("image_sub")
        self.subscription = self.create_subscription(CompressedImage, '/camera/image_raw/compressed',self.image_callback,qos_profile=qos_profile_sensor_data)
        self.cv = CvBridge()

    def image_callback(self, data):
        self.get_logger().info('recieved frame')
        current_frame = self.cv.compressed_imgmsg_to_cv2(data)
        cv2.imshow("Camera", current_frame)
        cv2.waitKey(1)
    def destroy_node(self):
        return super().destroy_node()
    
def main(args = None):
    rclpy.init(args=args)
    Node = ImageSub()
    try:
        rclpy.spin(Node)
    except KeyboardInterrupt:
        pass
    finally:
        Node.destroy_node()
        rclpy.shutdown()
    