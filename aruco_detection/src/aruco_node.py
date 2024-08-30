import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Pose
from aruco_detection.msg import ArucoMarkers
from std_msgs.msg import Int8
import tf


class ArucoNode:
    def __init__(self):
        self.marker_size = 0.06 # marker width: 60mm
        self.aruco_dictionary_id = 'DICT_7X7_250'
        self.image_topic = '/camera/color/image_raw'
        self.camera_info_topic = '/camera/color/camera_info'

        # Set up TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.loginfo(f"Marker size: {self.marker_size}")
        rospy.loginfo(f"Marker type: {self.aruco_dictionary_id}")
        rospy.loginfo(f"Image topic: {self.image_topic}")
        rospy.loginfo(f"Camera info topic: {self.camera_info_topic}")

        # Validate and set up ArUco dictionary
        try:
            dictionary_id = cv2.aruco.__getattribute__(self.aruco_dictionary_id)
            if not isinstance(dictionary_id, int):
                raise AttributeError
        except AttributeError:
            rospy.logerr(f"Invalid aruco_dictionary_id: {self.aruco_dictionary_id}")
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            rospy.logerr(f"Valid options: {options}")
            return

        # Set up subscriptions, publishers
        self.info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.info_callback)
        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.markers_pub = rospy.Publisher("/aruco_markers", ArucoMarkers, queue_size=10)
        self.command_pub = rospy.Publisher("/command", Int8, queue_size=10)  # Publisher for marker_id

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()


    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.K), (3, 3))
        self.distortion = np.array(self.info_msg.D)
        rospy.loginfo("Camera info received, unsubscribing from the camera info topic.")
        self.info_sub.unregister()


    def image_callback(self, img_msg):
        if self.info_msg is None:
            rospy.logwarn("No camera info has been received!")
            return

        cv_image_c = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")  # Convert to color image
        cv_image = cv2.cvtColor(cv_image_c, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        markers = ArucoMarkers()

        markers.header.frame_id = self.info_msg.header.frame_id
        markers.header.stamp = img_msg.header.stamp

        # Detect markers in the image
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
        )

        if marker_ids is not None:
            # Estimate pose of each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.intrinsic_mat, self.distortion
            )

            for i, marker_id in enumerate(marker_ids):
                # Draw detected markers and axes on the image
                cv2.aruco.drawDetectedMarkers(cv_image_c, corners)
                cv2.drawFrameAxes(cv_image_c, self.intrinsic_mat, self.distortion, rvecs[i], tvecs[i], 0.03)

                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]
                if pose.position.z <= 0.3:
                    # 가까워지면 marker_id를 publish
                    self.command_pub.publish(Int8(marker_id[0]))

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf.transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

                # Broadcast TF for marker
                self.tf_broadcaster.sendTransform(
                    (tvecs[i][0][0], tvecs[i][0][1], tvecs[i][0][2]),
                    quat,
                    rospy.Time.now(),  # Use current ROS time instead of image time
                    f"marker_{marker_id[0]}",  # Child frame (e.g., the marker ID)
                    "base_link"  # Parent frame (e.g., camera or base_link)
                )



            # Publish the detected markers and poses
            self.markers_pub.publish(markers)

        cv2.imshow('Realsense Image', cv_image_c)  # Show the image with detected markers
        cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("aruco_node")
    aruco_node = ArucoNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
