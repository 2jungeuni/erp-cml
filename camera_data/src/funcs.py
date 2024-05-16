import numpy as np
import cv2
import math
import config as config


def roi_extractor(img, x1, y1, x2, y2):
    mask = np.zeros_like(img)
    mask[y1:y2, x1:x2] = 255
    roi = cv2.bitwise_and(img, mask)
    return roi

def auto_canny(image, sigma=0.33):
    v = np.median(image)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)
    # edged = cv2.Canny(image, 50, 150)
    return edged

def draw_hough_lines(img, lines, color=(0, 255, 0), thickness=2):
    line_img = np.copy(img)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
                # cv2.circle(line_img, (x1,y1), 10, (0,0,255), 1)
                # cv2.circle(line_img, (x2,y2), 10, (255,255,0), 1)
    return line_img

def slope(x1, y1, x2, y2):
    if x2 - x1 == 0:
        return float('inf')
    else:
        return (y2 - y1) / (x2 - x1)
    
def angle(x1,y1,x2,y2):
    return (math.atan2(y2-y1, x2-x1) * 180) / math.pi # degree

def line_equation(points):
        x1, y1, x2, y2 = points[0], points[1], points[2], points[3]
        if x2 - x1 == 0: 
            return float('inf'), y1
        m = slope(x1,y1,x2,y2)
        c = y1 - m * x1
        return m, c  
    
def line_length(points):
    """Calculate the length of a line segment."""
    x1, y1, x2, y2 = points[0], points[1], points[2], points[3]
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
def extend_lines_fit_section(points, current_section):
    #* Extending lines to fit in the section
    # print("EXTENDING")
    m, c = line_equation(points)
    
    if line_length(points) < 70:
        return None
        
    # print(points)
    # print(m,c)
    if m < 0 and m != np.NINF:
        y2 = config.section_list[current_section+1]
        y1 = config.section_list[current_section]
        x1 = int((y1-c)/m)
        x2 = int((y2-c)/m)
    elif m > 0 and m != np.inf:
        y2 = config.section_list[current_section+1]
        y1 = config.section_list[current_section]
        x1 = int((y1-c)/m)
        x2 = int((y2-c)/m)
    elif m == np.inf: # vertical line
        y2 = config.section_list[current_section+1]
        y1 = config.section_list[current_section]
        x1 = points[0]
        x2 = points[0]
    else: # Excluding horizontal line (which can't be a lane)
        return None

    return [x1,y1,x2,y2]

def calculate_distance(x1,y1,x2,y2, x3,y3,x4,y4):
    """ Calculate the average distance between the midpoints of two lines """
    midpoint1 = ((x1 + x2) / 2, (y1 + y2) / 2)
    midpoint2 = ((x3 + x4) / 2, (y3 + y4) / 2)
    return np.sqrt((midpoint2[0] - midpoint1[0])**2 + (midpoint2[1] - midpoint1[1])**2), midpoint1, midpoint2
    

def filter_lines_initial(lines, current_section, temp2, temp, all_lines, real_all_lines):
    """ called at each section """
    closest_l_line = []
    closest_r_line = []
    max_slope = 110
    min_slope = 70
        
    if lines is not None:
        for line in lines:
            extended_line = extend_lines_fit_section(line[0], current_section)       
            if extended_line is not None:
                x1, y1, x2, y2 = line[0]
                cv2.line(temp2, (x1, y1), (x2, y2), (0, 255, 0), 1)
                cv2.line(temp, (x1, y1), (x2, y2), (0, 255, 0), 1)
                x1, y1, x2, y2 = extended_line
                # cv2.line(temp, (x1, y1), (x2, y2), (0, 255, 0), 1)
                ext_line_angle = angle(x1,y1,x2,y2)

                if min_slope < ext_line_angle < max_slope:
                    if x1 < temp.shape[1]//2:
                        closest_l_line = extended_line
                    else:
                        closest_r_line = extended_line

        # #*-----Drawing selected lines in each section
        if len(closest_l_line) != 0:         
            cv2.line(temp, (closest_l_line[0],closest_l_line[1]), (closest_l_line[2],closest_l_line[3]), (0,255,255), 2)
        if len(closest_r_line) != 0:
            cv2.line(temp, (closest_r_line[0],closest_r_line[1]), (closest_r_line[2],closest_r_line[3]), (0,255,255), 2)
        # cv2.imshow("midpoint", temp2)
        
    # cv2.imshow("Hough lines extended", temp)
    cv2.imwrite("visualizations/selected_yellow/"+str(config.q)+".jpg", temp)

    real_all_lines.append([closest_l_line, closest_r_line])
    all_lines.append([closest_l_line, closest_r_line])
    
    # for t in range(len(real_all_lines)):
    #     for tt in range(len(real_all_lines[t])):
    #         if len(real_all_lines[t][tt]) == 0:
    #             real_all_lines[t] = [[],[]]

    # print(real_all_lines)        

    #* Drawing filtered lines
    for i in range(len(real_all_lines)):
        if len(real_all_lines[i][0]) != 0:
            cv2.line(temp, (real_all_lines[i][0][0], real_all_lines[i][0][1]), (real_all_lines[i][0][2], real_all_lines[i][0][3]), (0,255,255), 2)
        if len(real_all_lines[i][1]) != 0:
            cv2.line(temp, (real_all_lines[i][1][0], real_all_lines[i][1][1]), (real_all_lines[i][1][2], real_all_lines[i][1][3]), (0,255,255), 2)
            
    # Drawing detected lines
    if len(closest_l_line) > 0: 
        cv2.line(temp2, (closest_l_line[0],closest_l_line[1]), (closest_l_line[2],closest_l_line[3]), (0,255,255), 2)
    if len(closest_r_line) > 0:
        cv2.line(temp2, (closest_r_line[0],closest_r_line[1]), (closest_r_line[2],closest_r_line[3]), (0,255,255), 2)    


def filter_lines(lines, prev_Q_l, prev_Q_r, current_section, temp2, temp, no_line_cnt_l, no_line_cnt_r, all_lines, real_all_lines):
    """ called at each section """   
    prev_l_x1 = prev_Q_l[current_section][0]
    prev_l_y1 = prev_Q_l[current_section][1]
    prev_l_x2 = prev_Q_l[current_section+1][0]
    prev_l_y2 = prev_Q_l[current_section+1][1]
    prev_r_x1 = prev_Q_r[current_section][0]
    prev_r_y1 = prev_Q_r[current_section][1]
    prev_r_x2 = prev_Q_r[current_section+1][0]
    prev_r_y2 = prev_Q_r[current_section+1][1]
    prev_l_angle = angle(prev_l_x1, prev_l_y1, prev_l_x2, prev_l_y2)
    prev_r_angle = angle(prev_r_x1, prev_r_y1, prev_r_x2, prev_r_y2)
    
    # cv2.line(temp, (prev_l_x1, prev_l_y1), (prev_l_x2, prev_l_y2), (0, 0, 255), 2)
    # cv2.line(temp, (prev_r_x1, prev_r_y1), (prev_r_x2, prev_r_y2), (0, 0, 255), 2)
    
    
    closest_l_line = []
    closest_r_line = []
    min_abs_distance_l = min_abs_distance_r = 25
    min_slope_diff = 15
    comparison_distance_l = np.inf
    comparison_angle_l = np.inf
    comparison_distance_r = np.inf
    comparison_angle_r = np.inf
    
    # print(no_line_cnt_l, no_line_cnt_r)
    if no_line_cnt_l >= 20:
        #- print("NO LEFT LINE DETECTED for 20F --> SETTING SEARCH RANGE to 100")
        min_abs_distance_l = 100
        min_slope_diff = 40
    elif no_line_cnt_l >= 10:
        #- print("NO LEFT LINE DETECTED for 10F --> SETTING SEARCH RANGE to 50")
        min_abs_distance_l = 50
        # min_slope_diff = 30
        
    if no_line_cnt_r >= 20:
        #- print("NO RIGHT DETECTED for 20F --> SETTING SEARCH RANGE to 100")
        min_abs_distance_r = 100
        min_slope_diff = 40
    elif no_line_cnt_r >= 10:
        #- print("NO RIGHT DETECTED for 10F --> SETTING SEARCH RANGE to 50")
        min_abs_distance_r = 50
        # min_slope_diff = 30
    

    if lines is not None:
        for line in lines:
            extended_line = extend_lines_fit_section(line[0], current_section)       
            if extended_line is not None:
                x1, y1, x2, y2 = line[0]
                cv2.line(temp2, (x1, y1), (x2, y2), (0, 255, 0), 1)
                cv2.line(temp, (x1, y1), (x2, y2), (0, 255, 0), 1)
                x1, y1, x2, y2 = extended_line
                # cv2.line(temp, (x1, y1), (x2, y2), (0, 255, 0), 1)
                ext_line_angle = angle(x1,y1,x2,y2)
                
                angle_diff_Q_l = abs(prev_l_angle - ext_line_angle)
                angle_diff_Q_r = abs(prev_r_angle - ext_line_angle)
                distance_diff_Q_l, mp1, mp2 = calculate_distance(prev_l_x1, prev_l_y1, prev_l_x2, prev_l_y2, x1,y1,x2,y2)
                cv2.circle(temp2, (int(mp2[0]), int(mp2[1])), 3, (255,0,255),-1)
                cv2.circle(temp2, (int(mp1[0]), int(mp1[1])), 5, (255,255,0),-1)
                distance_diff_Q_r, mp1, mp2 = calculate_distance(prev_r_x1, prev_r_y1, prev_r_x2, prev_r_y2, x1,y1,x2,y2)
                cv2.circle(temp2, (int(mp2[0]), int(mp2[1])), 3, (255,0,255),-1)
                cv2.circle(temp2, (int(mp1[0]), int(mp1[1])), 5, (255,255,0),-1)

                # left line first
                if distance_diff_Q_l < min_abs_distance_l and distance_diff_Q_l < comparison_distance_l and angle_diff_Q_l < min_slope_diff and angle_diff_Q_l < comparison_angle_l:
                    comparison_distance_l = distance_diff_Q_l
                    comparison_angle_l = angle_diff_Q_l
                    closest_l_line = extended_line
                # right line afterward
                elif distance_diff_Q_r < min_abs_distance_r and distance_diff_Q_r < comparison_distance_r and angle_diff_Q_r < min_slope_diff and angle_diff_Q_r < comparison_angle_r:
                    comparison_distance_r = distance_diff_Q_r
                    comparison_angle_r = angle_diff_Q_r
                    closest_r_line = extended_line

        # #*-----Drawing selected lines in each section
        if len(closest_l_line) != 0:         
            cv2.line(temp, (closest_l_line[0],closest_l_line[1]), (closest_l_line[2],closest_l_line[3]), (0,255,255), 2)
        if len(closest_r_line) != 0:
            cv2.line(temp, (closest_r_line[0],closest_r_line[1]), (closest_r_line[2],closest_r_line[3]), (0,255,255), 2)
        # cv2.imshow("midpoint", temp2)
        # cv2.waitKey(1000)
        
    #*-----Drawing selected lines (including prev lines) in each section            
    # if closest_l_line is None:       
    #     closest_l_line = [prev_l_x1, prev_l_y1, prev_l_x2, prev_l_y2]
    #     cv2.line(temp2, (closest_l_line[0],closest_l_line[1]), (closest_l_line[2],closest_l_line[3]), (0,0,255), 2)
    # else:
    #     cv2.line(temp2, (closest_l_line[0],closest_l_line[1]), (closest_l_line[2],closest_l_line[3]), (0,255,255), 2)
    # if closest_r_line is None:
    #     closest_r_line = [prev_r_x1, prev_r_y1, prev_r_x2, prev_r_y2]
    #     cv2.line(temp2, (closest_r_line[0],closest_r_line[1]), (closest_r_line[2],closest_r_line[3]), (0,0,255), 2)
    # else:        
    #     cv2.line(temp2, (closest_r_line[0],closest_r_line[1]), (closest_r_line[2],closest_r_line[3]), (0,255,255), 2)        
    # cv2.imshow("midpoint", temp2)
    # cv2.waitKey(1000)

    # cv2.imshow("Hough lines extended", temp)
    cv2.imwrite("visualizations/selected_yellow/"+str(config.q)+".jpg", temp)


    #todo
    #*-----Filling empty section----------
    '''
    This is for temporarily filling all lines in all sections to use it in filtering.
    Previously, filtering is done only when both lines are detected in a section.
    But, this yields undesired result when the one line is wrongly detected and the other one is not detected. (slide 80)
    To solve this issue, we're gonna use 'all_lines' to enable filtering in the case of only one line is detected. 
    '''
    real_all_lines.append([closest_l_line, closest_r_line])
    all_lines.append([closest_l_line, closest_r_line])
    
    # Using prev cp as lines in missing section (not connecting with detected line using slope)
    for i in range(len(all_lines)):
        if len(all_lines[i][0]) == 0:
            all_lines[i][0] = [prev_l_x1, prev_l_y1, prev_l_x2, prev_l_y2]
            cv2.line(temp2, (all_lines[i][0][0],all_lines[i][0][1]), (all_lines[i][0][2],all_lines[i][0][3]), (0,0,255), 2)
        if len(all_lines[i][1]) == 0:
            all_lines[i][1] = [prev_r_x1, prev_r_y1, prev_r_x2, prev_r_y2]
            cv2.line(temp2, (all_lines[i][1][0],all_lines[i][1][1]), (all_lines[i][1][2],all_lines[i][1][3]), (0,0,255), 2)


    if current_section == 1:
        sec_0_dist_dif = None
        #- print(real_all_lines)
        for i in range(len(all_lines)):
            #* If lane angle difference is larger than 5, delete it  
            if len(all_lines[i][0]) != 0 and len(all_lines[i][1]) != 0:
                angle_l = angle(all_lines[i][0][0], all_lines[i][0][1], all_lines[i][0][2], all_lines[i][0][3])
                angle_r = angle(all_lines[i][1][0], all_lines[i][1][1], all_lines[i][1][2], all_lines[i][1][3])
                angle_diff = abs(angle_l - angle_r)
                print("angle diff: ", angle_diff)
                if angle_diff >= 10:
                    print(f"TOO LARGE ANGLE DIFFERENCE ({angle_diff}) --> DELETING")
                    real_all_lines[i][0] = []
                    real_all_lines[i][1] = []
                    
            #* If lane width is smaller or larger than usual, delete it
            if len(all_lines[i][0]) != 0 and len(all_lines[i][1]) != 0:
                distance_l_r,_,_ = calculate_distance(all_lines[i][0][0], all_lines[i][0][1], all_lines[i][0][2], all_lines[i][0][3], all_lines[i][1][0], all_lines[i][1][1], all_lines[i][1][2], all_lines[i][1][3])
                #- print("dist diff:",distance_l_r)
                if i == 0:
                    sec_0_dist_dif = distance_l_r
                else:
                    if sec_0_dist_dif - distance_l_r > 0: # if dist in sec 0 is larger than in sec 1 is checking non-sense lane states
                        print(f"NON SENSE LINE (S0) ({sec_0_dist_dif}, {distance_l_r})--> DELETING")
                        real_all_lines[0][0] = []
                        real_all_lines[0][1] = []
                    if distance_l_r <= 160 or distance_l_r >= 280:
                        print(f"NARROW LANE WIDTH (S1)({distance_l_r})--> DELETING")
                        real_all_lines[1][0] = []
                        real_all_lines[1][1] = []
                
                
        #* Drawing filtered lines
        for i in range(len(real_all_lines)):
            if len(real_all_lines[i][0]) != 0:
                cv2.line(temp, (real_all_lines[i][0][0], real_all_lines[i][0][1]), (real_all_lines[i][0][2], real_all_lines[i][0][3]), (0,255,255), 2)
            if len(real_all_lines[i][1]) != 0:
                cv2.line(temp, (real_all_lines[i][1][0], real_all_lines[i][1][1]), (real_all_lines[i][1][2], real_all_lines[i][1][3]), (0,255,255), 2)
                
    # print("---------Real all lines >>")    
    # print(real_all_lines)
    # print("---------All lines >>")    
    # print(all_lines)
    # print("---------")
    
    
    #!------------------------
    
    # Drawing detected lines
    if len(closest_l_line) > 0: 
        # broaden_search_distance_l = False    
        cv2.line(temp2, (closest_l_line[0],closest_l_line[1]), (closest_l_line[2],closest_l_line[3]), (0,255,255), 2)
    if len(closest_r_line) > 0:
        # broaden_search_distance_r = False
        cv2.line(temp2, (closest_r_line[0],closest_r_line[1]), (closest_r_line[2],closest_r_line[3]), (0,255,255), 2)    
    

def control_point(all_lines):
    """Averaging two points"""
    Q_l = []
    Q_r = []
        
    for i in range(len(all_lines)):
        if i == 0:
            if len(all_lines[i][0]) > 0:
                Q_l.append([all_lines[i][0][0], all_lines[i][0][1]])
                Q_l.append([all_lines[i][0][2], all_lines[i][0][3]])
            else:
                Q_l.append([])
            if len(all_lines[i][1]) > 0:
                Q_r.append([all_lines[i][1][0], all_lines[i][1][1]])
                Q_r.append([all_lines[i][1][2], all_lines[i][1][3]])
            else:
                Q_r.append([])
        
        else:
            if len(all_lines[i][0]) > 0:
                if len(Q_l) == 1:
                    Q_l.append([all_lines[i][0][0], all_lines[i][0][1]])
                    Q_l.append([all_lines[i][0][2], all_lines[i][0][3]])
                else:
                    Q_l[i] = [ int((Q_l[i][0] + all_lines[i][0][0])/2), all_lines[i][0][1] ]
                    Q_l.append([all_lines[i][0][2], all_lines[i][0][3]])
            else:
                Q_l.append([])
                if len(Q_l) == 2:
                    Q_l.append([])
                    
            if len(all_lines[i][1]) > 0:
                if len(Q_r) == 1:
                    Q_r.append([all_lines[i][1][0], all_lines[i][1][1]])
                    Q_r.append([all_lines[i][1][2], all_lines[i][1][3]])
                else:
                    Q_r[i] = [ int((Q_r[i][0] + all_lines[i][1][0])/2), all_lines[i][1][1] ]
                    Q_r.append([all_lines[i][1][2], all_lines[i][1][3]])
            else:
                Q_r.append([])
                if len(Q_r) == 2:
                    Q_r.append([])
                    
    return Q_l, Q_r
    
def extract_lines_in_section_initial(roi, no_line_cnt):
    # global q #* for drawing
    # global initial_not_found
    temp = np.copy(roi)
    temp = cv2.cvtColor(temp, cv2.COLOR_GRAY2BGR)
    temp2 = np.copy(temp) # for drawing hough lines
    a = np.copy(temp) # for drawing hough lines
    all_lines = []
    all_lines_for_filtering = []
    
    ## ? Section Line Drawing
    # for i in config.section_list:
    #     cv2.line(temp2, (0,i), (temp2.shape[1],i), (255,0,0), 1)
    #     cv2.line(temp, (0,i), (temp.shape[1],i), (255,0,0), 1)
    #     cv2.line(a, (0,i), (a.shape[1],i), (255,0,0), 1)
    # cv2.imwrite("try/left_right_lane_bspline_KF_BEV/3rd_KF_try/section_divided/"+str(q)+".jpg", a)
    
    for i in range(len(config.section_list)-1): #* for each section
        #- print(i)
        separated = np.copy(roi)
        separated = roi_extractor(separated, 0, config.section_list[i], separated.shape[1], config.section_list[i+1]) #? extract each section from img
        lines = cv2.HoughLinesP(separated, 1, np.pi/180, threshold=40, minLineLength=20, maxLineGap=50) # Probabilistic Hough Transform
        # # ? Drawing Hough lines
        # if lines is not None:
        #     for line in lines:
        #         x1, y1, x2, y2 = line[0]
        #         cv2.line(temp, (x1, y1), (x2, y2), (0, 255, 0), 1)
        # cv2.imshow("Hough lines", temp)
        # cv2.waitKey(3000)
        
        filter_lines_initial(lines, i, temp2, temp, all_lines_for_filtering, all_lines) 
        #- print("Filtered:", all_lines)       
        
        # cv2.imshow("1st", temp)
        # cv2.waitKey(150)
    
    # if lines found in all sections
    if len(all_lines[0][0]) != 0 and len(all_lines[0][1]) != 0 and len(all_lines[1][0]) != 0 and len(all_lines[1][1]) != 0:
        print("&&&&& FOUND")
        config.initial_not_found = False
            
    Q_l, Q_r = control_point(all_lines)
    for point_l, point_r in zip(Q_l, Q_r):
        if len(point_l) > 0:
            cv2.circle(temp2, point_l[0:2], 7, (255, 255, 255), 2)
            cv2.circle(temp, point_l[0:2], 7, (255, 255, 255), 2)
        if len(point_r) > 0:
            cv2.circle(temp2, point_r[0:2], 7, (255, 255, 255), 2)
            cv2.circle(temp, point_r[0:2], 7, (255, 255, 255), 2)
        
    
    
    # #? Detected Line Drawing
    # for line in all_lines:
    #     for x1, y1, x2, y2 in line:
    #         cv2.line(temp, (x1, y1), (x2, y2), (0,255,255), 2)
        
    # cv2.imshow("Final lanes before filtering", temp2)
    # cv2.imshow("Final lanes after filtering", temp)
    # cv2.imwrite("visualizations/unist_line_in_section_bf_filter/"+str(q)+".jpg", temp2)
    # cv2.imwrite("try/left_right_lane_bspline_KF_BEV/3rd_KF_try/unist_line_in_section_af_filter/"+str(q)+".jpg", temp)

    return all_lines, temp, Q_l, Q_r


def extract_lines_in_section(roi, prev_Q_l, prev_Q_r, no_line_cnt):
    temp = np.copy(roi)
    temp = cv2.cvtColor(temp, cv2.COLOR_GRAY2BGR)
    temp2 = np.copy(temp) # for drawing hough lines
    a = np.copy(temp) # for drawing hough lines
    all_lines = []
    all_lines_for_filtering = []
    
    ## ? Section Line Drawing
    # for i in config.section_list:
    #     cv2.line(temp2, (0,i), (temp2.shape[1],i), (255,0,0), 1)
    #     cv2.line(temp, (0,i), (temp.shape[1],i), (255,0,0), 1)
    #     cv2.line(a, (0,i), (a.shape[1],i), (255,0,0), 1)
    # cv2.imwrite("try/left_right_lane_bspline_KF_BEV/3rd_KF_try/section_divided/"+str(q)+".jpg", a)
    
    for i in range(len(config.section_list)-1): #* for each section
        separated = np.copy(roi)
        separated = roi_extractor(separated, 0, config.section_list[i], separated.shape[1], config.section_list[i+1]) #? extract each section from img
        lines = cv2.HoughLinesP(separated, 1, np.pi/180, threshold=40, minLineLength=20, maxLineGap=50) # Probabilistic Hough Transform
        # # ? Drawing Hough lines
        # if lines is not None:
        #     for line in lines:
        #         x1, y1, x2, y2 = line[0]
        #         cv2.line(temp, (x1, y1), (x2, y2), (0, 255, 0), 1)
        # cv2.imshow("Hough lines", temp)
        # cv2.imwrite("try/left_right_lane_bspline_KF_BEV/3rd_KF_try/hough_section/"+str(q)+".jpg", temp)
        
        filter_lines(lines, prev_Q_l, prev_Q_r, i, temp2, temp, no_line_cnt[2*i], no_line_cnt[2*i+1], all_lines_for_filtering, all_lines) #todo pass line dist threshold
    print(all_lines)
    for point_l, point_r in zip(prev_Q_l, prev_Q_r):
        cv2.circle(temp2, point_l[0:2], 7, (0, 0, 255), 2)
        cv2.circle(temp2, point_r[0:2], 7, (0, 0, 255), 2)
        cv2.circle(temp, point_l[0:2], 7, (0, 0, 255), 2)
        cv2.circle(temp, point_r[0:2], 7, (0, 0, 255), 2)
    Q_l, Q_r = control_point(all_lines)
    print(Q_l)
    print(Q_r)
    for point_l, point_r in zip(Q_l, Q_r):
        if len(point_l) > 0:
            cv2.circle(temp2, point_l[0:2], 7, (255, 255, 255), 2)
            cv2.circle(temp, point_l[0:2], 7, (255, 255, 255), 2)
        if len(point_r) > 0:
            cv2.circle(temp2, point_r[0:2], 7, (255, 255, 255), 2)
            cv2.circle(temp, point_r[0:2], 7, (255, 255, 255), 2)
        
    
    
    # #? Detected Line Drawing
    # for line in all_lines:
    #     for x1, y1, x2, y2 in line:
    #         cv2.line(temp, (x1, y1), (x2, y2), (0,255,255), 2)
        
    cv2.imshow("Final lanes before filtering", temp2)
    # cv2.imshow("Final lanes after filtering", temp)
    cv2.imwrite("visualizations/unist_line_in_section_bf_filter/"+str(config.q)+".jpg", temp2)
    
    return all_lines, temp, Q_l, Q_r

def BEV(img, bev_pts):
    bev_pts1 = np.array(bev_pts).astype(np.float32)
    bev_pts2 = np.float32([[0,0],[300,0],[300,500],[0,500]])
    matrix = cv2.getPerspectiveTransform(bev_pts1, bev_pts2)
    inv_matrix = cv2.getPerspectiveTransform(bev_pts2, bev_pts1)
    bev = cv2.warpPerspective(img, matrix,(300,500))
    
    return bev, inv_matrix

def R_set_considering_control_points(Q_l, Q_r, prev_esti, no_line_cnt):
    weight = [1, 10, 200, 1, 10, 200]
    
    if prev_esti is not None:
        R_ = np.zeros((6, 1))
        for i in range(len(Q_l)):
            if len(Q_l[i]) > 0:
                dist_diff = abs(Q_l[i][0] - prev_esti[i])
                # angle_diff = angle()
                R_[i] = dist_diff * weight[i] + 0.0000001
            else: # if no control point
                R_[i] = 100000
                
            if len(Q_r[i]) > 0:
                diff = abs(Q_r[i][0] - prev_esti[i+3])
                R_[i+3] = diff * weight[i+3] + 0.0000001
            else: # if no control point
                R_[i+3] = 100000
                
        return np.diag(R_.reshape((6,)))
    
    return np.diag(1000 * np.ones(6))

def sigmoid(x):    
    alpha = 0.5
    beta = 160
    # alpha = 1
    # beta = 100
    
    img_float = x.astype(np.float32)
    sigmoid = ( 255 / (1 + np.exp(-alpha * (img_float - beta))) ).astype(np.uint8)

    return sigmoid

def min_dist_set(no_line_cnt, lines_in_section):
    #- print(lines_in_section)
    for i in range(len(lines_in_section)): # 1st: section 1 / 2nd: section 2
        for j in range(2): # 1st: left / 2nd: right
            if len(lines_in_section[i][j]) == 0:
                no_line_cnt[2*i+j] += 1
            else:
                no_line_cnt[2*i+j] = 0
            # broaden_search_distance[2*i+j] = no_line_cnt[2*i+j] >= 10
    #- print(no_line_cnt)
