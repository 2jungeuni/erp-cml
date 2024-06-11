import sys
import os

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
import numpy as np
import cv2
import time
import copy

import rospy
import message_filters
from message_filters import ApproximateTimeSynchronizer
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError

from b_spline_func import bspline
from kalman_filter_3rd import kalman_filter
from avg_input import *
from funcs import *
import config


def world_to_img_pts(img, intrinsic):   
    pts = np.array([
        [config.world_x_min, config.world_y_min, 0, 1],
        [config.world_x_min, config.world_y_max, 0, 1],
        [config.world_x_max, config.world_y_min, 0, 1],
        [config.world_x_max, config.world_y_max, 0, 1],
    ], dtype=np.float32)
     
    img_pts = []
    
    for i in range(pts.shape[0]):
        cam_pts = config.extrinsic @ pts[i]
        img_pts_hom = intrinsic @ cam_pts[:3]
        x,y = img_pts_hom[:2] / img_pts_hom[2]
        # print(int(x),int(y))
        img_pts.append((x,y))
        
        cv2.circle(img, (int(x), int(y)), 4, (0,0,255), -1)
    
    cv2.imshow("world pts on img", img)
    
    return img_pts


class PosePublisher:
    def __init__(self):
        self.pos_pub = rospy.Publisher("/reference_pos", PointStamped, queue_size=10)
        self.rgb_info = None
        self.depth_info = None

        # Subscribe to camera info topics once to get the camera parameters
        self.rgb_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback, "rgb")

        # Subscribe to image topics with ApproximateTimeSynchronizer
        self.rgb_raw_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        # self.depth_raw_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
        self.depth_raw_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

        self.ts = ApproximateTimeSynchronizer([self.rgb_raw_sub, self.depth_raw_sub], 10, 0.1)
        self.ts.registerCallback(self.image_callback)

    def camera_info_callback(self, data, camera_type):
        if camera_type == "rgb" and not self.rgb_info:
            self.rgb_info = data
            self.rgb_info_sub.unregister()  # Unsubscribe after receiving the info


    def image_callback(self, rgb_image, depth_image):
        # Load RGB and Depth image
        bridge = CvBridge()
        try:
            cv_rgb = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
            cv_depth = bridge.imgmsg_to_cv2(depth_image, "16UC1")
        except CvBridgeError as e:
            print(e)
            return
        
        #--------------------------------
        
        rgb_intrinsic = np.array(self.rgb_info.K).reshape(3, 3)
        
        bev_pts = world_to_img_pts(cv_rgb, rgb_intrinsic)
        print(bev_pts)
        if lane_det_main(cv_rgb, bev_pts) == None:
            return
        else:
            final_Q_l, final_Q_r, bspline_est_left_pts, bspline_est_right_pts, inv_matrix = lane_det_main(cv_rgb, bev_pts)
        # final_Q_l, final_Q_r, bspline_est_left_pts, bspline_est_right_pts, inv_matrix = lane_det_main(cv_rgb, bev_pts)
        
        #! bev img pts -> img -> world -> mid point
        #! 1. bev pts -> img pts   (mid lane)
        final_pt = (bspline_est_left_pts + bspline_est_right_pts)/2  # mid line
        bevpts = np.array(final_pt, dtype=np.float32)
        if len(bevpts.shape) == 2:
            bevpts = bevpts[:, np.newaxis, :]  # Add the required dimension
        img_pts = cv2.perspectiveTransform(bevpts, inv_matrix)
        img_pts = img_pts[:, 0, :].astype(int)
        # for pt in img_pts: # visualization
        #     cv2.circle(final, pt, 7, (255,0,0), 2)
        # cv2.imshow('Final', final)

        #! 2. X_CAR -> img
        cam_pts = config.extrinsic @ np.array([config.REF_POINT[0],config.REF_POINT[1], config.REF_POINT[2], 1])
        img_pts_hom = rgb_intrinsic @ cam_pts[:3]
        ref_x, ref_y = img_pts_hom[:2] / img_pts_hom[2]
        # cv2.circle(final, (int(ref_x), int(ref_y)), 3, (0,255,0), -1) # visualization
        # cv2.imshow('asdasd', final)
        
        #! 3. Find a ref point on lane
        found_point = None
        for pt in img_pts:
            x, y = pt[0], pt[1]
            if x == int(ref_x):
                found_point = (x, y)
                print("REF POINT FOUND")
                break

        #! found ref pt -> world
        depth_value = cv_depth[found_point[0], found_point[1]] * 0.1  # mm -> cm
        cam_coords = depth_value * np.linalg.inv(rgb_intrinsic) @ np.array([found_point[0], found_point[1], 1])
        world_coords = config.extrinsic @ np.append(cam_coords, 1)
        world_target_point = world_coords[:3]
        
        #! Publish 'world_target_point'
        print(world_target_point)
        #--------------------------------



prev_Q_l = None
prev_Q_r = None

P_0 = 100*np.eye(6)

prev_esti = None

frame_count = 0
start_time = time.time()
x_esti = None
prev_P = None
no_line_cnt = [0,0,0,0]
    
def lane_det_main(raw_img, bev_pts):    
    print()
    print(f"--------{config.q}--------")
    
    gray = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
    bev, inv_matrix = BEV(gray, bev_pts)
    sig = sigmoid(bev)
                    
    ret, thres1 = cv2.threshold(sig, 150, 255, cv2.THRESH_BINARY)
    ret, thres2 = cv2.threshold(sig, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (1,1))
    dilate = cv2.dilate(thres2, k)
    canny_dilate = auto_canny(dilate)
    # cv2.imwrite("visualizations/dilate/"+str(config.q)+".jpg", dilate)
    # cv2.imwrite("visualizations/visualizationscanny/"+str(config.q)+".jpg", canny_dilate)
    # cv2.imwrite("visualizations/preprocessed/"+str(config.q)+".jpg", sig)
    # cv2.imwrite("visualizations/binary/"+str(config.q)+".jpg", thres1)
    # cv2.imwrite("visualizations/binary_otsu/"+str(config.q)+".jpg", thres2)
    # cv2.imshow("BEV", bev)
    # cv2.imshow("BEV_sig", sig)
    # cv2.imshow("binary", thres2)
    # cv2.imshow("canny_dilate", canny_dilate)
    
    bev = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
    
    if config.initial_not_found:
        print("@@@@@@@@@ FINDING INITIAL LANE @@@@@@@@@")
        lines_in_section, lines_in_section_img, Q_l, Q_r = extract_lines_in_section_initial(canny_dilate, no_line_cnt)
        R = np.zeros((6, 1))
    else:
        print("@@@@@@@@@ INITIAL LANE FOUND @@@@@@@@@")
        lines_in_section, lines_in_section_img, Q_l, Q_r = extract_lines_in_section(canny_dilate, prev_Q_l, prev_Q_r, no_line_cnt)
        R = R_set_considering_control_points(Q_l, Q_r, prev_esti, no_line_cnt)
    
    # print("R >>>>>>")
    # print(R)
    min_dist_set(no_line_cnt, lines_in_section)
    

    if config.initial_not_found: #! If initial lane not found, pass everything below
        print(f"Initial lane not found: {x_esti}")
        return None
    
    # Prepare KF states
    first_elements = []
    for inner_list in Q_l:
        if len(inner_list) > 0:
            first_elements.append(inner_list[0])
        else:
            first_elements.append(0)
    KF_Q_l = np.array(first_elements)
    
    first_elements = []
    for inner_list in Q_r:
        if len(inner_list) > 0:
            first_elements.append(inner_list[0])
        else:
            first_elements.append(0)
    KF_Q_r = np.array(first_elements)
    
    #- print("KF_Q_l:",KF_Q_l)
    #- print("KF_Q_r:",KF_Q_r)
    
    if KF_Q_l is not None and KF_Q_r is not None:
        z_meas = np.concatenate((KF_Q_l, KF_Q_r), axis=0)
        z_meas = np.expand_dims(z_meas, axis=1)
    else:
        z_meas = []
    
    if prev_Q_l == None:
        x_esti = z_meas.copy()
        P = P_0
        prev_P = P_0
        R = np.diag(1000 * np.ones(6))
    
    #- print(x_esti)
    
    x_esti, P = kalman_filter(x_esti, z_meas, P, R, config.q)
    prev_esti = copy.deepcopy(x_esti)
    #- print("@@Update@", x_esti)
    new_Q_l = x_esti[0:len(config.section_list)].tolist()
    new_Q_r = x_esti[len(config.section_list):len(config.section_list)*2].tolist()
    new_Q_l = [[int(new_Q_l[i][j]) for j in range(len(new_Q_l[i]))] for i in range(len(new_Q_l))]
    new_Q_r = [[int(new_Q_r[i][j]) for j in range(len(new_Q_r[i]))] for i in range(len(new_Q_r))]
    for i in range(len(new_Q_l)):
        new_Q_l[i].append(config.section_list[i])
        new_Q_r[i].append(config.section_list[i])
        cv2.circle(lines_in_section_img, new_Q_l[i], 7, (255,0,0), 2)
        cv2.circle(lines_in_section_img, new_Q_r[i], 7, (255,0,0), 2)
    
    cv2.imshow("Final lanes after filtering", lines_in_section_img)
    # cv2.imwrite("visualizations/unist_line_in_section_af_filter/"+str(config.q)+".jpg", lines_in_section_img)

    
    img = np.zeros((500, 300, 3), dtype=np.uint8)
    
    bspline_img, bspline_est_left_pts = bspline(new_Q_l, bev, (0, 255, 0)) # estimation
    bspline_img, bspline_est_right_pts = bspline(new_Q_r, bspline_img, (0, 255, 0)) # estimation
    bspline_img, bspline_meas_left_pts = bspline(Q_l, bspline_img, (0, 0, 255)) # measurement
    bspline_img, bspline_meas_right_pts = bspline(Q_r, bspline_img, (0, 0, 255)) # measurement
    
    # For merging two images
    img, bspline_left_pts = bspline(new_Q_l, img, (0, 255, 0)) # estimation
    img, bspline_right_pts = bspline(new_Q_r, img, (0, 255, 0)) # estimation

    prev_Q_l = copy.deepcopy(new_Q_l)
    prev_Q_r = copy.deepcopy(new_Q_r)
    prev_P = copy.deepcopy(P)
    
    cv2.imshow("B-spline on BEV", bspline_img)
    # cv2.imwrite("visualizations/unist_bspline/"+str(config.q)+".jpg", bspline_img)

    if img is not None:
        newwarp1 = cv2.warpPerspective(img, inv_matrix, (raw_img.shape[1], raw_img.shape[0]))
        final = cv2.addWeighted(raw_img, 1, newwarp1, 1, 0) # original 이미지에 복구한 이미지 합성
        
        # cv2.imshow('B-spline', img)
        cv2.imshow('Final', final)
        # cv2.imwrite("visualizations/unist_final/"+str(config.q)+".jpg", final)
    
    
    frame_count += 1
    if frame_count % 100 == 0:
        end_time = time.time()
        fps = frame_count / (end_time - start_time)
        print("Processed {0} frames in {1:.2f} seconds, approx FPS: {2:.2f}".format(frame_count, end_time - start_time, fps))
        frame_count = 0
        start_time = time.time()
        # cv2.waitKey(1000)
    cv2.waitKey(1)
    # print("===============================================")

    config.q += 1 #* for drawing
            
    return new_Q_l, new_Q_r, bspline_est_left_pts, bspline_est_right_pts, inv_matrix   



if __name__ == "__main__":

    rospy.init_node("pose_publisher")
    node = PosePublisher()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()