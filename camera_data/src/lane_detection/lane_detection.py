# built-in
import os
import sys
import time
import copy
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

# install needed
import cv2

def preprocessing_newnew(bev):
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 10))
    k2 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 1))
    gradient = cv2.morphologyEx(bev, cv2.MORPH_GRADIENT, k2)
    # cv2.imshow("gradient", gradient)

    max_percent = 9
    hist = cv2.calcHist([gradient], [0], None, [256], [0, 256])
    threshold = np.searchsorted(np.cumsum(hist), bev.size * (1 - 0.01 * max_percent))
    ret, thres2 = cv2.threshold(gradient, threshold, 255, cv2.THRESH_BINARY)
    reopening = cv2.morphologyEx(thres2, cv2.MORPH_OPEN, k)
    reclosing = cv2.morphologyEx(reopening, cv2.MORPH_CLOSE, k)
    dilate = cv2.dilate(reclosing, k2, iterations=2)
    # cv2.imshow("thres3", dilate)
    canny_dilate2 = cv2.Canny(dilate, 0, 255)
    # cv2.imshow("canny_dilate2", canny_dilate2)
    return canny_dilate2

class LaneDetection:
    def __init__(self, config, img, margins):
        self.img = img
        self.margins = np.float32(margins.T)
        self.config = config
        self.gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # transformaton matrix
        self.img_to_bev = cv2.getPerspectiveTransform(self.margins, self.config.bev_margins)
        self.bev_to_img = cv2.getPerspectiveTransform(self.config.bev_margins, self.margins)

    def get_bev_img(self):
        return cv2.warpPerspective(self.gray_img, self.img_to_bev, (self.config.bev_margin_max_x, self.config.bev_margin_max_y))
    
    def get_lane_edges(self):
        


# def lane_det_main(self, raw_img, bev_pts):
    #     # cv2.imshow('raw_img', raw_img)
    #     # print(f"--------{config.q}--------")
    #     gray = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
    #     bev, inv_matrix = BEV(gray, bev_pts)
    #     cv2.imshow('bev', bev)
    #     canny_dilate = preprocessing_newnew(bev)
    #     # canny_dilate = preprocessing(bev, config.q, self.min_val_queue)
    #     bev = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
        
    #     if config.initial_not_found:
    #         # print("@@@@@@@@@ FINDING INITIAL LANE @@@@@@@@@")
    #         lines_in_section, lines_in_section_img, Q_l, Q_r = extract_lines_in_section_initial(canny_dilate, self.no_line_cnt)
    #         # R = np.zeros((6, 1))
    #     else:
    #         # print("@@@@@@@@@ INITIAL LANE FOUND @@@@@@@@@")
    #         lines_in_section, lines_in_section_img, Q_l, Q_r = extract_lines_in_section(canny_dilate, self.prev_Q_l, self.prev_Q_r, self.no_line_cnt)
    #         R = R_set_considering_control_points(Q_l, Q_r, self.prev_esti, self.no_line_cnt)
        
    #     min_dist_set(self.no_line_cnt, lines_in_section)
    #     if config.initial_not_found: #! If initial lane not found, skip everything below
    #         print(f"Initial lane not found: {self.x_esti}")
    #         return None
        
    #     # print(Q_l)
    #     # print(Q_r)
    #     # Prepare KF state vectors
    #     first_elements = []
    #     for inner_list in Q_l:
    #         if len(inner_list) > 0:
    #             first_elements.append(inner_list[0])
    #         else:
    #             first_elements.append(0)
    #     KF_Q_l = np.array(first_elements)
        
    #     first_elements = []
    #     for inner_list in Q_r:
    #         if len(inner_list) > 0:
    #             first_elements.append(inner_list[0])
    #         else:
    #             first_elements.append(0)
    #     KF_Q_r = np.array(first_elements)
        
    #     # print("KF_Q_l:", KF_Q_l)
    #     # print("KF_Q_r:", KF_Q_r)
        
    #     if KF_Q_l is not None and KF_Q_r is not None:
    #         z_meas = np.concatenate((KF_Q_l, KF_Q_r), axis=0)
    #         z_meas = np.expand_dims(z_meas, axis=1)
    #     else:
    #         z_meas = []
        
    #     if self.prev_Q_l == None:
    #         self.x_esti = z_meas.copy()
    #         self.P = self.P_0
    #         self.prev_P = self.P_0
    #         R = np.diag(1000 * np.ones(6))
        
    #     #- print(x_esti)
        
    #     self.x_esti, self.P = kalman_filter(self.x_esti, z_meas, self.P, R, config.q)
    #     self.prev_esti = copy.deepcopy(self.x_esti)
    #     #- print("@@Update@", x_esti)
    #     new_Q_l = self.x_esti[0:len(config.section_list)].tolist()
    #     new_Q_r = self.x_esti[len(config.section_list):len(config.section_list)*2].tolist()
    #     new_Q_l = [[int(new_Q_l[i][j]) for j in range(len(new_Q_l[i]))] for i in range(len(new_Q_l))]
    #     new_Q_r = [[int(new_Q_r[i][j]) for j in range(len(new_Q_r[i]))] for i in range(len(new_Q_r))]
    #     for i in range(len(new_Q_l)):
    #         new_Q_l[i].append(config.section_list[i])
    #         new_Q_r[i].append(config.section_list[i])
    #         cv2.circle(lines_in_section_img, new_Q_l[i], 7, (255,0,0), 2)
    #         cv2.circle(lines_in_section_img, new_Q_r[i], 7, (255,0,0), 2)
        
    #     # cv2.imshow("Final lanes after filtering", lines_in_section_img)
    #     # cv2.imwrite("visualizations/unist_line_in_section_af_filter/"+str(config.q)+".jpg", lines_in_section_img)

        
    #     img = np.zeros((500, 300, 3), dtype=np.uint8)
    #     # print(Q_l)
    #     # print(Q_r)
    #     bspline_img, bspline_est_left_pts = bspline(new_Q_l, bev, (0, 255, 0)) # estimation
    #     bspline_img, bspline_est_right_pts = bspline(new_Q_r, bspline_img, (0, 255, 0)) # estimation
    #     bspline_img, bspline_meas_left_pts = bspline(Q_l, bspline_img, (0, 0, 255)) # measurement
    #     bspline_img, bspline_meas_right_pts = bspline(Q_r, bspline_img, (0, 0, 255)) # measurement
        
    #     # For merging two images
    #     img, bspline_left_pts = bspline(new_Q_l, img, (0, 255, 0)) # estimation
    #     img, bspline_right_pts = bspline(new_Q_r, img, (0, 255, 0)) # estimation

    #     self.prev_Q_l = copy.deepcopy(new_Q_l)
    #     self.prev_Q_r = copy.deepcopy(new_Q_r)
    #     self.prev_P = copy.deepcopy(self.P)
        
    #     # cv2.imshow("B-spline on BEV", bspline_img)
    #     # cv2.imwrite("visualizations/unist_bspline/"+str(config.q)+".jpg", bspline_img)

    #     if img is not None:
    #         newwarp1 = cv2.warpPerspective(img, inv_matrix, (raw_img.shape[1], raw_img.shape[0]))
    #         self.final = cv2.addWeighted(raw_img, 1, newwarp1, 1, 0) # original 이미지에 복구한 이미지 합성
            
            
    #         # cv2.imshow('B-spline', img)
    #         # cv2.imshow('Final', self.final)
    #         # cv2.imwrite("visualizations/unist_final/"+str(config.q)+".jpg", final)
        
        
    #     self.frame_count += 1
    #     if self.frame_count % 100 == 0:
    #         end_time = time.time()
    #         fps = self.frame_count / (end_time - self.start_time)
    #         # print("Processed {0} frames in {1:.2f} seconds, approx FPS: {2:.2f}".format(self.frame_count, end_time - self.start_time, fps))
    #         self.frame_count = 0
    #         self.start_time = time.time()
    #         # cv2.waitKey(1000)
    #     cv2.waitKey(1)

    #     config.q += 1 #* for drawing
                
    #     return new_Q_l, new_Q_r, bspline_est_left_pts, bspline_est_right_pts, inv_matrix   