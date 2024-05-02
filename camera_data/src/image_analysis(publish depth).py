# ros
import rospy
import message_filters
from message_filters import ApproximateTimeSynchronizer
# from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError


class DepthPublisher:
    def __init__(self):
        # self.pos_pub = rospy.Publisher("/current_pos", PointStamped, queue_size=10)

        rgb_raw_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        rgb_info_sub = message_filters.Subscriber("/camera/color/camera_info", CameraInfo)
        depth_raw_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
        depth_info_sub = message_filters.Subscriber("/camera/depth/camera_info", CameraInfo)
        ts = ApproximateTimeSynchronizer([rgb_raw_sub, rgb_info_sub, depth_raw_sub, depth_info_sub], 10, 0.1)
        ts.registerCallback(self.image_callback)


    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            depth_img = param
            x = 320
            y = 240
            # depth image가 16-bit 이미지라고 가정
            depth_value = depth_img[y, x] if depth_img[y, x] != 0 else 0
            print(f"Depth at pixel ({x}, {y}): {depth_value} mm")
            
            
    def image_callback(self, rgb_image, rgb_info, depth_image, depth_info):
        bridge = CvBridge()
        try:
            cv_rgb = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
            cv_depth = bridge.imgmsg_to_cv2(depth_image, "16UC1")
        except CvBridgeError as e:
            print(e)
            return

        cv2.namedWindow('RGB Image')
        # cv2.setMouseCallback('RGB Image', self.mouse_callback, cv_depth)
        x = 333
        y = 333
        depth_value = cv_depth[y, x]
        print(f"Depth at pixel ({x}, {y}): {depth_value} mm")
        cv2.circle(cv_rgb, (x, y), 2, (0, 0, 255), -1)
        cv2.imshow('RGB Image', cv_rgb)
        cv2.waitKey(1)



if __name__ == "__main__":
    rospy.init_node("depth_publisher")
    node = DepthPublisher()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()