import rospy
import message_filters
from message_filters import ApproximateTimeSynchronizer
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

T_wdc =  np.array([[ 0,  0,  1, 0.0],     
                    [-1,  0,  0, 0.0],
                    [ 0, -1,  0, 0.0],
                    [ 0,  0,  0, 1]])


class PosePublisher:
    def __init__(self):
        self.pos_pub = rospy.Publisher("/reference_pos", PointStamped, queue_size=10)

        rgb_raw_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        rgb_info_sub = message_filters.Subscriber("/camera/color/camera_info", CameraInfo)
        depth_raw_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
        depth_info_sub = message_filters.Subscriber("/camera/depth/camera_info", CameraInfo)
        ts = ApproximateTimeSynchronizer([rgb_raw_sub, rgb_info_sub, depth_raw_sub, depth_info_sub], 10, 0.1)
        ts.registerCallback(self.image_callback)

            
    def image_callback(self, rgb_image, rgb_info, depth_image, depth_info):
        # Load RGB and Depth image
        bridge = CvBridge()
        try:
            cv_rgb = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
            cv_depth = bridge.imgmsg_to_cv2(depth_image, "16UC1")
        except CvBridgeError as e:
            print(e)
            return


        # Load external values
        X_CAR = 10
        rgb_intrinsic = np.array(rgb_info.K).reshape(3, 3)
        depth_intrinsic = np.array(depth_info.K).reshape(3, 3)
        rgb_w = rgb_info.width
        rgb_h = rgb_info.height
        depth_w = depth_info.width
        depth_h = depth_info.height


        # Calculate x-axis length in RGB image

        # Convert world coordinate to image coordinate
        target_point = np.array([X_CAR, 0, 0, 1])
        cam_pts = np.linalg.inv(T_wdc) @ target_point
        img_pts_hom = rgb_intrinsic @ cam_pts[:3]
        x, y = img_pts_hom[:2] / img_pts_hom[2]
        x = int(x)
        y = int(y)
        print(x, y)


        # Calculate centerline(highest gradient)
        gray = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2GRAY)
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=5)
        abs_sobel_x = np.absolute(sobel_x)
        scaled_sobel = np.uint8(255 * abs_sobel_x / np.max(abs_sobel_x))
        # edges = cv2.Canny(gray, threshold1=50, threshold2=150)


        
        # x_center = np.argmax(scaled_sobel[y, :])

        row_gradient = scaled_sobel[y, :]
        indices = np.argsort(row_gradient)[:2]
        x_center = np.mean(indices).astype(int)
        
        print(x_center)

        # Measure depth value and calculate delta_y value
        depth_value = cv_depth[y, x_center]
        if depth_value != 0:
            delta_y = np.sqrt(depth_value**2 - T_wdc[2, 3]**2 - X_CAR**2)
        else:
            delta_y = 0
        print(f"Depth at pixel ({x_center}, {y}): {depth_value} mm")
        
        # print target 2D coordinate
        print(delta_y)
        
        cv2.circle(cv_rgb, (x_center, y), 2, (0, 0, 255), -1)
        cv2.imshow('RGB Image', cv_rgb)
        # cv2.imshow('Canny Edges', edges)
        cv2.waitKey(0)



if __name__ == "__main__":
    rospy.init_node("pose_publisher")
    node = PosePublisher()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

# depth_to_color_extrinsics - constant?
# rotation: [0.9999845623970032, -0.003664492629468441, -0.004174978472292423,
#             0.0036358407232910395, 0.9999699592590332, -0.006849836092442274, 
#            0.0041999537497758865, 0.006834551226347685, 0.9999678134918213]
# translation: [0.014983640983700752, 0.00022463918139692396, 6.477197166532278e-05]
