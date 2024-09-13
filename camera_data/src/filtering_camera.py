import pyrealsense2 as rs
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge

class RealSensePublisher:
    def __init__(self):
        # Publishers for RGB and Depth
        self.color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.aligned_depth_pub = rospy.Publisher('/camera/aligned_depth_to_color/image_raw', Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=10)

        self.bridge = CvBridge()

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(self.config)

        # Create filters with specified settings
        self.threshold_filter = rs.threshold_filter()
        self.threshold_filter.set_option(rs.option.min_distance, 0.0)
        self.threshold_filter.set_option(rs.option.max_distance, 6.0)

        self.spatial_filter = rs.spatial_filter()
        self.spatial_filter.set_option(rs.option.filter_magnitude, 2.0)
        self.spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.5)
        self.spatial_filter.set_option(rs.option.filter_smooth_delta, 20.0)

        self.temporal_filter = rs.temporal_filter()
        self.temporal_filter.set_option(rs.option.filter_smooth_alpha, 0.4)
        self.temporal_filter.set_option(rs.option.filter_smooth_delta, 20.0)

        self.hole_filling_filter = rs.hole_filling_filter()
        self.hole_filling_filter.set_option(rs.option.holes_fill, 1)  # Farthest from camera

        self.depth_to_disparity = rs.disparity_transform(True)
        self.disparity_to_depth = rs.disparity_transform(False)

        # Align object to align depth frames to color frames
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        rospy.loginfo("Realsense is now publishing!")


    def get_camera_info(self, frame, profile):
        intrinsics = profile.as_video_stream_profile().get_intrinsics()
        camera_info = CameraInfo()
        camera_info.header.stamp = rospy.Time.now()
        camera_info.header.frame_id = "camera_color_optical_frame"
        camera_info.height = intrinsics.height
        camera_info.width = intrinsics.width
        camera_info.distortion_model = "plumb_bob"

        # D, K, R, P matrices
        camera_info.D = [intrinsics.coeffs[i] for i in range(5)]
        camera_info.K = [0] * 9
        camera_info.R = [0] * 9
        camera_info.P = [0] * 12

        camera_info.K[0] = intrinsics.fx
        camera_info.K[2] = intrinsics.ppx
        camera_info.K[4] = intrinsics.fy
        camera_info.K[5] = intrinsics.ppy
        camera_info.K[8] = 1

        camera_info.R[0] = camera_info.R[4] = camera_info.R[8] = 1

        camera_info.P[0] = intrinsics.fx
        camera_info.P[2] = intrinsics.ppx
        camera_info.P[5] = intrinsics.fy
        camera_info.P[6] = intrinsics.ppy
        camera_info.P[10] = 1

        return camera_info


    def process_frames(self):
        while not rospy.is_shutdown():
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Apply filters
            depth_frame = self.threshold_filter.process(depth_frame)
            depth_frame = self.depth_to_disparity.process(depth_frame)
            depth_frame = self.spatial_filter.process(depth_frame)
            depth_frame = self.temporal_filter.process(depth_frame)
            depth_frame = self.hole_filling_filter.process(depth_frame)
            depth_frame = self.disparity_to_depth.process(depth_frame)

            # Convert images to ROS Image messages
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")

            # Get camera info
            camera_info_msg = self.get_camera_info(color_frame, color_frame.profile)

            # Publish the images and camera info
            self.color_pub.publish(color_msg)
            self.aligned_depth_pub.publish(depth_msg)
            self.camera_info_pub.publish(camera_info_msg)


    def stop(self):
        self.pipeline.stop()

if __name__ == '__main__':
    try:
        rospy.init_node('realsense_publisher')
        rsp = RealSensePublisher()
        rsp.process_frames()
    except rospy.ROSInterruptException:
        rsp.stop()