import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

#cam1, 2, 3, 4
REGIONS = [(0,0,960,540), (960, 0, 960, 540), (0,540,960,540), (960,540,960,540)]
PATTERN = (7, 11)
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
FRAMES = 600


#list of calibrated cameras
Calibrated = []

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,PATTERN[0] * PATTERN[1]
objp = np.zeros((PATTERN[0]*PATTERN[1],3), np.float32)
objp[:,:2] = np.mgrid[0:PATTERN[1],0:PATTERN[0]].T.reshape(-1,2)

# Vector to store 3D points
obj_points = [[] * 4]

# Vector to store 2D points
img_points = [[] * 4]

#frame count variables
count = np.zeros(4)

#calibration outputs
calib = [[] * 4]

class ImageSub(Node):
    def __init__(self):
        super().__init__("image_sub")
        self.subscription = self.create_subscription(CompressedImage, '/camera/image_raw/compressed',self.image_callback,qos_profile=qos_profile_sensor_data)
        self.cv = CvBridge()



    def image_callback(self, data):
        self.get_logger().info('recieved frame')
        current_frame = self.cv.compressed_imgmsg_to_cv2(data)
        if len(Calibrated) < 4:
            for i, (x,y,w,h) in enumerate(REGIONS):
                cropped_image = current_frame[x:x+w, y:y+h]
                gray_cropped_image = cv2.cvtColor(cropped_image)
                ret, corners = cv2.findChessboardCorners(gray_cropped_image, PATTERN, None)
                if ret and (i not in Calibrated):
                    obj_points[i].append(objp)
                    corners2 = cv2.cornerSubPix(gray_cropped_image,corners, PATTERN, (-1,-1), CRITERIA) #? on (11,11) and (-1,-1)
                    img_points[i].append(corners2)
                    count[i] = count[i] + 1
                    cv2.drawChessboardCorners(cropped_image,PATTERN,corners,ret)
                    cv2.imshow(f'cam {i}', cropped_image)
                    cv2.waitKey(500)
                    if count[i] == FRAMES:
                        calib[i] = cv2.calibrateCamera(obj_points[i],img_points[i],gray_cropped_image.shape[::-1], None, None)
                        Calibrated.append(i)
        else:
            self.get_logger().info("all cameras calibrated")
            self.destroy_subscription(self.subscription)
            self.print_calib()


    def print_calib(self):
        for i, (ret, mtx, dist, rvecs, tvecs) in enumerate(calib):
            print(f"Error in projection - cam {i} : \n", ret)
            print(f"\nCamera matrix : \n", mtx)
            print(f"\nDistortion coefficients : \n", dist)
            print(f"\nRotation vector : \n", rvecs)
            print(f"\nTranslation vector : \n", tvecs)
            print("=======================================")


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
    

