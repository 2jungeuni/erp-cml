import numpy as np
import cv2
import math
import config as config
import copy
from scipy.optimize import minimize


# -------------------- main functions(main에서 바로 호출) -------------------------
def world_to_img_pts(img, intrinsic):   
    pts = np.array([
        [config.world_x_max, config.world_y_max, 0, 1],  
        [config.world_x_max, config.world_y_min, 0, 1],
        [config.world_x_min, config.world_y_min, 0, 1],
        [config.world_x_min, config.world_y_max, 0, 1],
    ], dtype=np.float32)
    img_pts = []
    
    for i in range(pts.shape[0]):
        cam_pts = np.linalg.inv(config.extrinsic) @ pts[i]
        # cam_pts = config.extrinsic @ pts[i]
        img_pts_hom = intrinsic @ cam_pts[:3]
        x,y = img_pts_hom[:2] / img_pts_hom[2]
        img_pts.append((x, y))
        cv2.circle(img, (int(x), int(y)), 4, (0, 0, 255), -1)
        # cv2.imshow("img", img)

    return img_pts


def BEV(img, bev_pts):
    bev_pts1 = np.array(bev_pts).astype(np.float32)
    bev_pts2 = np.float32([[0,0],[config.bev_x,0],[config.bev_x,config.bev_y],[0,config.bev_y]])
    matrix = cv2.getPerspectiveTransform(bev_pts1, bev_pts2)
    inv_matrix = cv2.getPerspectiveTransform(bev_pts2, bev_pts1)
    bev = cv2.warpPerspective(img, matrix,(config.bev_x,config.bev_y))
    return bev, inv_matrix
    

def preprocessing_newnew(bev):
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 5))
    k2 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 1))

    gradient = cv2.morphologyEx(bev, cv2.MORPH_GRADIENT, k2)
    # cv2.imshow("gradient", gradient)

    max_percent = 9
    hist = cv2.calcHist([gradient], [0], None, [256], [0, 256])
    threshold = np.searchsorted(np.cumsum(hist), bev.size * (1 - 0.01 * max_percent))
    ret, thres2 = cv2.threshold(gradient, threshold, 255, cv2.THRESH_BINARY)
    # cv2.imshow("bin", thres2)
    reopening = cv2.morphologyEx(thres2, cv2.MORPH_OPEN, k)
    reclosing = cv2.morphologyEx(reopening, cv2.MORPH_CLOSE, k)
    dilate = cv2.dilate(reclosing, k2, iterations=2)
    # cv2.imshow("thres3", dilate)
    canny_dilate2 = cv2.Canny(dilate, 0, 255)
    # cv2.imshow("canny_dilate2", canny_dilate2)

    # return canny_dilate2, thres2
    return canny_dilate2, dilate

def preprocessing(bev, iteration, max_val_queue, iteration_interval=100):
    # sigmoid 삭제, gaussian 도입
    x, y = 150, 480
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 10))
    opening = cv2.morphologyEx(bev, cv2.MORPH_OPEN, k)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, k)
    dilate = cv2.dilate(closing, k, iterations=2)
    # cv2.imshow("dilate", dilate)
    blur = cv2.GaussianBlur(dilate,(5,5),0)
    ret3, th3 = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # cv2.imshow("thres3", th3)
    if iteration % iteration_interval == 0:
        max_val = np.max(dilate[y-2:y+3, x-40:x+40])
        max_val_queue.append(max_val)
        # print(np.mean(max_val_queue))

    dilate_shifted = np.where(dilate == 0, 0, int(np.mean(max_val_queue)) - dilate)
    # cv2.imshow("dilate_shifted", dilate_shifted)

    mu, sigma = 180, 50  # 평균과 표준 편차 값 설정
    max_percent = 20
    filtered = gaussian_transform(dilate_shifted, mu, sigma)
    # cv2.imshow("gaussian", filtered)
    hist = cv2.calcHist([filtered], [0], None, [256], [0, 256]).flatten()
    threshold = np.searchsorted(np.cumsum(hist), bev.size * (1 - 0.01 * max_percent))
    ret, thres2 = cv2.threshold(filtered, threshold, 255, cv2.THRESH_BINARY)
    # cv2.imshow("thres2", thres2)

    canny_dilate = cv2.Canny(thres2, 0, 255)
    cv2.imshow("canny_dilate", canny_dilate)

    num_labels, labels_im, stats, _ = cv2.connectedComponentsWithStats(canny_dilate)
    new_binary_img = np.zeros_like(canny_dilate)
    for label in range(1, num_labels):  # 0은 배경이므로 제외
        if stats[label, cv2.CC_STAT_AREA] > 50:  # Component area size check
            component_mask = (labels_im == label).astype(np.uint8) * 255
            new_binary_img = cv2.bitwise_or(new_binary_img, component_mask)

    return new_binary_img


def calculate_curvature(pts):
    if len(pts) < 3:
        return 0  # 점이 2개 이하인 경우 0으로 설정 (직선으로 간주)
    
    x = np.array([pt[0] for pt in pts])
    y = np.array([pt[1] for pt in pts])
    
    fit = np.polyfit(y, x, 2)
    curvature = np.max(np.abs(2 * fit[0] * y + fit[1]))
    
    return curvature

def is_continuous(img, pts, max_gap=200):
    for i in range(1, len(pts)):
        if np.linalg.norm(np.array(pts[i]) - np.array(pts[i-1])) > max_gap:
            print(pts[i])
            cv2.circle(img, pts[i], radius=10, color=(203, 192, 255), thickness=1)
            # print("IS CONTINUOS:", np.linalg.norm(np.array(pts[i]) - np.array(pts[i-1])))
            return False
    return True

def is_pixel_valid(canny_dilate, x, y, threshold=50):
    """
    현재 점이 흰색 픽셀 위에 있는지 확인하고, 주변의 흰색 픽셀 분포를 검사하는 함수.
    """
    if x < config.bev_x and y < config.bev_y and canny_dilate[y, x] > 0:
        return True
    
    # 주변 영역 내 흰색 픽셀의 비율 계산
    region = canny_dilate[max(0, y - 5):min(canny_dilate.shape[0], y + 5),
                          max(0, x - 5):min(canny_dilate.shape[1], x + 5)]
    white_pixel_count = np.sum(region > 0)
    total_pixel_count = region.size
    
    return white_pixel_count / total_pixel_count > threshold / 100.0


def sliding_window(canny_dilate, start_points_l, start_points_r, prev_curvature_l, prev_curvature_r, prev_coef_l, prev_coef_r, prev_pts_l, prev_pts_r, a_history, sw_no_update):
    l_pts = []
    r_pts = []
    
    def predict_next_point(points):
        if len(points) < 2:
            return points[-1]
        
        dx = points[-1][0] - points[-2][0]
        dy = points[-1][1] - points[-2][1]

        # 이전 이동 경로를 기반으로 다음 점을 예측하되, 약간의 랜덤 변동성을 추가
        predicted_x = points[-1][0] + dx + np.random.randint(-2, 3)
        predicted_y = points[-1][1] + dy + np.random.randint(-2, 3)

        return (predicted_x, predicted_y)


    def find_next_point(current_x, current_y, side, prev_curvature, estimated_current_state, direction="down"):
        dynamic_window_width = config.window_width
        dynamic_window_height = config.window_height

        curvature_calc_interval = 10  # 곡률 계산 빈도를 조절하는 변수
        pts_buffer = []  # 곡률 계산을 위한 임시 버퍼
        max_retries = 5  # 시도 횟수 제한
        retries = 0  # 시도 횟수 초기화
        
        while (current_y >= 0 and current_y < canny_dilate.shape[0]):
            window_x_min = max(current_x - dynamic_window_width // 4, 0) #! // 2
            window_x_max = min(current_x + dynamic_window_width // 4, canny_dilate.shape[1]) #! // 2
            window_y_min = max(current_y - dynamic_window_height, 0)
            window_y_max = min(current_y + dynamic_window_height, canny_dilate.shape[0])

            window = canny_dilate[window_y_min:window_y_max, window_x_min:window_x_max]
            non_zero_points = np.nonzero(window)

            if side == 'left':
                cv2.rectangle(output_image, (window_x_min, window_y_min), (window_x_max, window_y_max), (0, 0, 255), 1)
            else:
                cv2.rectangle(output_image, (window_x_min, window_y_min), (window_x_max, window_y_max), (255, 0, 0), 1)

            if len(non_zero_points[0]) > config.min_pix:
                weighted_x_sum = np.sum(non_zero_points[1] + window_x_min)
                weighted_y_sum = np.sum(non_zero_points[0] + window_y_min)
                total_weight = len(non_zero_points[0])

                mean_x = weighted_x_sum / total_weight
                mean_y = weighted_y_sum / total_weight

                dx = int(mean_x) - current_x
                cv2.circle(output_image, (int(mean_x), int(mean_y)), radius=3, color=(0, 255, 0), thickness=-1) #!!!
                
                # 픽셀 유효성 검사
                if not is_pixel_valid(canny_dilate, int(mean_x), int(mean_y)):
                    dynamic_window_width = max(dynamic_window_width // 2, 20)  # 최소 윈도우 크기 제한
                    dynamic_window_height = max(dynamic_window_height // 2, 20)
                    retries += 1
                    if retries > max_retries:  # 시도 횟수 초과 시 예측으로 넘어감
                        #! print("Too many retries, predicting next point.")
                        # max_retries를 초과했을 경우, 탐색을 멈추는 대신 현재까지 탐지된 점들의 경향을 바탕으로 다음 점을 예측하고, 그 예측된 점이 유효한지 확인. 
                        # 유효한 경우, 탐색을 계속 진행하고, 그렇지 않은 경우 탐색을 중단.

                        # 현재까지의 점들을 기반으로 다음 점을 예측
                        if side == 'left' and len(l_pts) != 0:
                            predicted_point = predict_next_point(l_pts)
                        elif side == 'right' and len(r_pts) != 0:
                            predicted_point = predict_next_point(r_pts)
                        else:
                            break
                            
                        # 예측한 점의 픽셀 유효성 검사
                        predicted_x, predicted_y = int(predicted_point[0]), int(predicted_point[1])
                        if is_pixel_valid(canny_dilate, predicted_x, predicted_y):
                            mean_x, mean_y = predicted_x, predicted_y
                            #! print(f"Predicted point ({predicted_x}, {predicted_y}) accepted.")
                            cv2.circle(output_image, (predicted_x, predicted_y), radius=15, color=(0, 255, 255), thickness=1)
                        else:
                            #! print(f"Predicted point ({predicted_x}, {predicted_y}) rejected.")
                            cv2.circle(output_image, (predicted_x, predicted_y), radius=15, color=(0, 255, 255), thickness=1)
                            break  # 예측한 점이 유효하지 않으면 루프 종료
                    else:
                        cv2.circle(output_image, (int(mean_x), int(mean_y)), radius=15, color=(0, 255, 0), thickness=1)
                        temp_window_x_min = max(current_x - dynamic_window_width // 2, 0)
                        temp_window_x_max = min(current_x + dynamic_window_width // 2, canny_dilate.shape[1])
                        temp_window_y_min = max(current_y - dynamic_window_height, 0)
                        temp_window_y_max = min(current_y + dynamic_window_height, canny_dilate.shape[0])
                        cv2.rectangle(output_image, (temp_window_x_min, temp_window_y_min), (temp_window_x_max, temp_window_y_max), (0, 255, 0), 2)
                        continue  # 현재 윈도우 크기로 다시 시도
                
                pts_buffer.append((int(mean_x), int(mean_y)))
                retries = 0  # 유효한 픽셀을 찾았으므로 시도 횟수 초기화

                # 곡률 계산 빈도에 도달하거나, 남은 점이 마지막인 경우 곡률 검사 수행
                if len(pts_buffer) >= curvature_calc_interval or current_y <= dynamic_window_height or current_y >= canny_dilate.shape[0] - dynamic_window_height:
                    # print("check after 10")
                    current_curvature = calculate_curvature(pts_buffer)
                    
                    if abs(current_curvature - prev_curvature) > config.curvature_threshold:
                        break
                    
                    if side == 'left':
                        l_pts.extend(pts_buffer)
                    else:
                        r_pts.extend(pts_buffer)
                    
                    pts_buffer.clear()
                    prev_curvature = current_curvature
                
                current_x += dx * 2 if abs(dx) > config.straight_threshold * 2 else dx
                # current_x += dx 
                
                # 방향에 따라 Y 좌표 업데이트
                if direction == "down":
                    current_y += dynamic_window_height
                elif direction == "up":
                    current_y -= dynamic_window_height
                elif direction == "both":
                    # 위쪽으로 한 스텝, 아래쪽으로 한 스텝을 동시에 실행
                    find_next_point(current_x, current_y + dynamic_window_height, side, prev_curvature, estimated_current_state, "down")
                    find_next_point(current_x, current_y - dynamic_window_height, side, prev_curvature, estimated_current_state, "up")
                    break

                if abs(dx) < config.straight_threshold:
                    dynamic_window_width = config.window_width
                    dynamic_window_height = config.window_height
                else:
                    dynamic_window_width = config.window_width * 2
                    dynamic_window_height = config.window_height // 2
            else:
                break


    def fit_lanes_and_draw(points, color):
        '''
        sliding window로 찾은 점들로 2차 다항식 피팅. -> output_image_copy에 그림.
        피팅된 점들과 식 반환.
        '''
        if len(points) < 2:
            print("Lack of points to be fitted.")
            return [], [], None
        points = np.array(points)
        
        x = points[:, 0]
        y = points[:, 1]

        # 2차 다항식으로 피팅
        fit = np.polyfit(y, x, 2) # x와 y 데이터 포인트들을 가장 잘 맞추는 2차 다항식의 계수 [a, b, c]를 계산
        fit_fn = np.poly1d(fit) # 계수들을 이용해 다항식을 표현하는 함수 fit_fn을 만듭 / fit_fn(x)을 호출하면 해당 x 값에 대한 다항식의 y 값을 계산
                
        y_new = np.linspace(np.min(y), np.max(y), num=100) # y 범위 내에서 100개의 점 생성
        x_new = fit_fn(y_new).astype(int)

        # 피팅한 곡선 그리기 (remove for actual deployment)
        points = np.array([[int(x), int(y)] for x, y in zip(x_new, y_new)])
        if len(points) > 1:
            cv2.polylines(output_image_poly, [points], isClosed=False, color=color, thickness=2)
                
        return x_new, y_new, fit_fn

    def evaluate_lane_width(l_coef, r_coef, tolerance=0.4):
        """
        두 차선의 계수를 기반으로 두 차선 사이의 폭이 합리적인지 판단하는 함수.

        :param coeffs_left: 왼쪽 차선의 계수 (a, b, c)
        :param coeffs_right: 오른쪽 차선의 계수 (a, b, c)
        :param tolerance: 허용 오차 (기본값은 0.1)
        :return: 합리적인 폭이면 True, 그렇지 않으면 False
        """
        
        if l_coef is None or r_coef is None:
            print("Cannot evaluate the width.")
            return False
        
        a_left, b_left, c_left = l_coef.c[0], l_coef.c[1], l_coef.c[2]
        a_right, b_right, c_right = r_coef.c[0], r_coef.c[1], r_coef.c[2]

        # y 좌표 범위에서 y 좌표를 생성
        y_values = np.linspace(0, config.bev_y, num=100)
        
        # 왼쪽 차선과 오른쪽 차선의 x 좌표 계산
        x_left = a_left * y_values**2 + b_left * y_values + c_left
        x_right = a_right * y_values**2 + b_right * y_values + c_right

        # 두 차선 사이의 x 좌표 차이 계산
        lane_widths = np.abs(x_right - x_left)
        
        # 평균 차선 폭 계산
        mean_lane_width = np.mean(lane_widths)
        
        # 차선 폭이 허용 범위 내에 있는지 판단
        if (config.lane_width_bev - config.lane_width_bev * tolerance) <= mean_lane_width <= (config.lane_width_bev + config.lane_width_bev * tolerance):
            print(f"\t차선 폭 OK: {mean_lane_width} 픽셀 (허용 범위: {config.lane_width_bev} ± {tolerance} 픽셀)")
            return True
        else:
            print(f"\t차선 폭 X: {mean_lane_width} 픽셀 (허용 범위: {config.lane_width_bev} ± {tolerance} 픽셀)")
            return False

    def history_state_comparison(l_coef, a_history):
        '''
        15 프레임 (0.5초)동안의 차선 식의 a값을 평균내어 현재 상황이 직선일지 곡선 (부호로 방향 구별) 예측.
        예측 값이랑 현재 프레임의 값이랑 다르면 false, 맞으면 true 반환.
        '''
        import statistics
        mean_a_history = statistics.mean(a_history)
        
        # history state
        if abs(mean_a_history) < 1e-4:
            estimated_current_state = "straight"
        else:
            if mean_a_history < 0:
                estimated_current_state = "left_curve"
            else:
                estimated_current_state = "right_curve"
        
        # current state
        if abs(l_coef.c[0]) < 1e-4:
            current_state = "straight"
        else:
            if l_coef.c[0] < 0:
                print("left_curve", l_coef.c[0])
                current_state = "left_curve"
            else:
                print("right_curve", l_coef.c[0])
                current_state = "right_curve"
            
        print("\t",estimated_current_state)
        print("\t",current_state)
        
        if current_state != estimated_current_state:
            return False, current_state, estimated_current_state
        else:
            return True, current_state, estimated_current_state
        
    def coef_thres_check(l_coef, r_coef, current_state, estimated_current_state):
        if l_coef.c[2] - r_coef.c[2] > config.lane_width_bev + 30:
            print("Too large 'c'")
            return False
        
        if l_coef.c[0] * r_coef.c[0] < 0:
            print("Diff 'a'")
            return False
        
        if estimated_current_state == "straight":
            # these values are obtained from graph.py
            a_thres = 9e-5 # - < ... < +
            b_thres = 0.15 # - < ... < +
            l_c_thres = config.bev_x / 2 # <
            r_c_thres= config.bev_x / 2
            
            l_a_check = abs(l_coef.c[0]) < a_thres
            l_b_check = abs(l_coef.c[1]) < b_thres
            l_c_check = l_coef.c[2] < l_c_thres
            r_a_check = abs(r_coef.c[0]) < a_thres
            r_b_check = abs(r_coef.c[1]) < b_thres
            r_c_check = r_coef.c[2] > r_c_thres
            
            print(l_a_check, r_a_check, l_b_check, r_b_check, l_c_check, r_c_check)
            if l_a_check and l_b_check and l_c_check and r_a_check and r_b_check and r_c_check:
                return True
            else:
                return False
        
        else:
            #todo
            return True
        
    def mask_lane_area(image, coeffs_left, coeffs_right, margin=20):        
        # 이미지 크기와 y 값 생성
        height, width = image.shape
        y_values = np.arange(height)

        # 왼쪽 차선과 오른쪽 차선의 x 좌표 계산
        x_left = np.polyval(coeffs_left, y_values) - margin
        x_right = np.polyval(coeffs_right, y_values) + margin

        # x 좌표를 이미지 경계 내로 제한
        x_left = np.clip(x_left, 0, width).astype(int)
        x_right = np.clip(x_right, 0, width).astype(int)

        # 마스크 생성
        masked_image = np.zeros_like(image)

        # 벡터화된 연산으로 마스크 적용
        for y in range(height):
            masked_image[y, x_left[y]:x_right[y]] = image[y, x_left[y]:x_right[y]]

        return masked_image
       
       
       
    # cv2.imshow("before mask", canny_dilate)
    # if prev_coef_l is not None and prev_coef_r is not None:
    #     canny_dilate = mask_lane_area(canny_dilate, prev_coef_l, prev_coef_r, margin=20)
    #     cv2.imshow("masked", canny_dilate)
    
    global output_image # (remove for actual deployment)
    output_image = cv2.cvtColor(canny_dilate, cv2.COLOR_GRAY2BGR) # (remove for actual deployment)
    output_image_poly = np.copy(output_image) # (remove for actual deployment)
    output_image_after_valid = np.copy(output_image) 

    # font = cv2.FONT_HERSHEY_SIMPLEX
    # font_scale = 1
    # color = (0, 255, 0)
    # thickness = 1
    
    l_survived = True
    r_survived = True

    global estimated_current_state
    if 'estimated_current_state' in globals() or 'estimated_current_state' in locals():
        #*----- sliding window -----
        # 여러 개의 시작점을 순차적으로 시도
        for i, (current_x_l, current_y_l) in enumerate(reversed(start_points_l)):
            actual_index = len(start_points_l) - 1 - i # 실제 인덱스를 계산
            # print(f"left start idx: {actual_index}")
            if current_y_l == 0:
                cv2.circle(output_image, (current_x_l, current_y_l), radius=5, color=(0, 255, 0), thickness=2) # (remove for actual deployment)
                find_next_point(current_x_l, current_y_l, 'left', prev_curvature_l, estimated_current_state, direction="down")
            elif actual_index == len(start_points_l) - 1: # 처음 인덱스에서 위로만 탐색
                cv2.circle(output_image, (current_x_l, current_y_l), radius=5, color=(0, 255, 0), thickness=2) # (remove for actual deployment)
                find_next_point(current_x_l, current_y_l, 'left', prev_curvature_l, estimated_current_state, direction="up") 
            elif current_y_l > 0 and current_y_l < canny_dilate.shape[0]:
                cv2.circle(output_image, (current_x_l, current_y_l), radius=5, color=(0, 255, 0), thickness=2) # (remove for actual deployment)
                find_next_point(current_x_l, current_y_l, 'left', prev_curvature_l, estimated_current_state, direction="both")
            if len(l_pts) > 20: # 유효한 차선을 찾았으면 중단 #TODO Need good metric for deciding this
                for i in l_pts: # (remove for actual deployment)
                    cv2.circle(output_image, i, radius=3, color=(0, 0, 255), thickness=-1) # (remove for actual deployment)
                break
        for i, (current_x_r, current_y_r) in enumerate(reversed(start_points_r)):
            actual_index = len(start_points_r) - 1 - i # 실제 인덱스를 계산
            if current_y_r == 0:
                cv2.circle(output_image, (current_x_r, current_y_r), radius=5, color=(0, 255, 0), thickness=-1) # (remove for actual deployment)
                find_next_point(current_x_r, current_y_r, 'right', prev_curvature_r, estimated_current_state, direction="down")
            elif actual_index == len(start_points_r) - 1: # 처음 인덱스에서 위로만 탐색
                cv2.circle(output_image, (current_x_r, current_y_r), radius=5, color=(0, 255, 0), thickness=-1) # (remove for actual deployment)
                find_next_point(current_x_r, current_y_r, 'right', prev_curvature_r, estimated_current_state, direction="up")
            elif current_y_r > 0 and current_y_r < canny_dilate.shape[0]:
                cv2.circle(output_image, (current_x_r, current_y_r), radius=5, color=(0, 255, 0), thickness=-1) # (remove for actual deployment)
                find_next_point(current_x_r, current_y_r, 'right', prev_curvature_r, estimated_current_state, direction="both")
            if len(r_pts) > 20: # 유효한 차선을 찾았으면 중단 #TODO Need good metric for deciding this
                for i in r_pts: # (remove for actual deployment)
                    cv2.circle(output_image, i, radius=3, color=(255, 0, 0), thickness=-1) # (remove for actual deployment)
                break
        #*----------------------
    else:
        #*----- sliding window -----
        # 여러 개의 시작점을 순차적으로 시도
        for i, (current_x_l, current_y_l) in enumerate(reversed(start_points_l)):
            actual_index = len(start_points_l) - 1 - i # 실제 인덱스를 계산
            # print(f"left start idx: {actual_index}")
            if current_y_l == 0:
                cv2.circle(output_image, (current_x_l, current_y_l), radius=5, color=(0, 255, 0), thickness=2) # (remove for actual deployment)
                find_next_point(current_x_l, current_y_l, 'left', prev_curvature_l, "straight", direction="down")
            elif actual_index == len(start_points_l) - 1: # 처음 인덱스에서 위로만 탐색
                cv2.circle(output_image, (current_x_l, current_y_l), radius=5, color=(0, 255, 0), thickness=2) # (remove for actual deployment)
                find_next_point(current_x_l, current_y_l, 'left', prev_curvature_l, "straight", direction="up")
            elif current_y_l > 0 and current_y_l < canny_dilate.shape[0]:
                cv2.circle(output_image, (current_x_l, current_y_l), radius=5, color=(0, 255, 0), thickness=2) # (remove for actual deployment)
                find_next_point(current_x_l, current_y_l, 'left', prev_curvature_l, "straight", direction="both")
            if len(l_pts) > 20: # 유효한 차선을 찾았으면 중단 #TODO Need good metric for deciding this
                for i in l_pts: # (remove for actual deployment)
                    cv2.circle(output_image, i, radius=3, color=(0, 0, 255), thickness=-1) # (remove for actual deployment)
                break
        for i, (current_x_r, current_y_r) in enumerate(reversed(start_points_r)):
            actual_index = len(start_points_r) - 1 - i # 실제 인덱스를 계산
            if current_y_r == 0:
                cv2.circle(output_image, (current_x_r, current_y_r), radius=5, color=(0, 255, 0), thickness=-1) # (remove for actual deployment)
                find_next_point(current_x_r, current_y_r, 'right', prev_curvature_r, "straight", direction="down")
            elif actual_index == len(start_points_r) - 1: # 처음 인덱스에서 위로만 탐색 
                cv2.circle(output_image, (current_x_r, current_y_r), radius=5, color=(0, 255, 0), thickness=-1) # (remove for actual deployment)
                find_next_point(current_x_r, current_y_r, 'right', prev_curvature_r, "straight", direction="up")
            elif current_y_r > 0 and current_y_r < canny_dilate.shape[0]:
                cv2.circle(output_image, (current_x_r, current_y_r), radius=5, color=(0, 255, 0), thickness=-1) # (remove for actual deployment)
                find_next_point(current_x_r, current_y_r, 'right', prev_curvature_r, "straight", direction="both")
            if len(r_pts) > 20: # 유효한 차선을 찾았으면 중단 #TODO Need good metric for deciding this
                for i in r_pts: # (remove for actual deployment)
                    cv2.circle(output_image, i, radius=3, color=(255, 0, 0), thickness=-1) # (remove for actual deployment)
                break
        #*----------------------
    
    #* 찾은 점들의 곡률이 합리적인지 확인.
    cur_l_curvature = calculate_curvature(l_pts)
    cur_r_curvature = calculate_curvature(r_pts)
    is_left_curv_valid = cur_l_curvature < config.max_curvature
    is_right_curv_valid = cur_r_curvature < config.max_curvature
    
    #* 합리적이면, 다항식 찾고, output_image_copy에 그림.
    if l_survived and is_left_curv_valid:
        left_x_new, left_y_new, l_coef = fit_lanes_and_draw(l_pts, (0, 0, 255))
        l_pts = [(int(x), int(y)) for x, y in zip(left_x_new, left_y_new)]
    else:
        l_survived = False

    if r_survived and is_right_curv_valid:
        right_x_new, right_y_new, r_coef = fit_lanes_and_draw(r_pts, (255, 0, 0))
        r_pts = [(int(x), int(y)) for x, y in zip(right_x_new, right_y_new)]

    else:
        r_survived = False
    #! 현재까지 곡률만 확인됨.
        

    #* lane width check
    if l_survived and r_survived:
        is_lane_feasible = evaluate_lane_width(l_coef, r_coef) #, 40 # 곡률 확인단계에서 통과됐으면, l_pts는 업데이트 되었을 것임.
        if not is_lane_feasible:
            l_survived = False
            r_survived = False
            
            if 'estimated_current_state' in globals() or 'estimated_current_state' in locals():

                if estimated_current_state == "right_curve":
                    r_coef = np.poly1d([l_coef.c[0], l_coef.c[1], l_coef.c[2] + config.lane_width_bev])
                    is_lane_feasible = evaluate_lane_width(l_coef, r_coef)
                    if is_lane_feasible:
                        print("\tWidth OK after copying left lane")
                        right_x_new = left_x_new + config.lane_width_bev
                        right_y_new = left_y_new
                        cur_r_curvature = cur_l_curvature
                        l_survived = True
                        r_survived = True
                
                elif estimated_current_state == "left_curve"                :
                    l_coef = np.poly1d([r_coef.c[0], r_coef.c[1], r_coef.c[2] + config.lane_width_bev])
                    is_lane_feasible = evaluate_lane_width(l_coef, r_coef)
                    if is_lane_feasible:
                        print("\tWidth OK after copying right lane")
                        left_x_new = right_x_new + config.lane_width_bev
                        left_y_new = right_y_new
                        cur_l_curvature = cur_r_curvature
                        l_survived = True
                        r_survived = True
        
            #- text = "width"
            #- org = (50, 100)
            #- output_image_poly = cv2.putText(output_image_poly, text, org, font, font_scale, color, thickness, cv2.LINE_AA)

        
    #* parallelism check
    if l_survived and r_survived: # l_coef is not None and r_coef is not None:
        epsilon_a = 0.001
        # epsilon_b = 0.2
        parallel_check = abs(l_coef.c[0] - r_coef.c[0]) < epsilon_a and r_coef.c[2] - l_coef.c[2] > 30 # and abs(l_coef.c[1] - r_coef.c[1]) < epsilon_b
        if not parallel_check: # if not parallel, use prev
            print("\t평행 X.")
            l_survived = False
            r_survived = False

            if 'estimated_current_state' in globals() or 'estimated_current_state' in locals():

                if estimated_current_state == "right_curve":
                    r_coef = np.poly1d([l_coef.c[0], l_coef.c[1], l_coef.c[2] + config.lane_width_bev])
                    is_lane_feasible = evaluate_lane_width(l_coef, r_coef)
                    if is_lane_feasible:
                        print("\tParallel OK after copying left lane")
                        right_x_new = left_x_new + config.lane_width_bev
                        right_y_new = left_y_new
                        cur_r_curvature = cur_l_curvature
                        l_survived = True
                        r_survived = True
                
                elif estimated_current_state == "left_curve"                :
                    l_coef = np.poly1d([r_coef.c[0], r_coef.c[1], r_coef.c[2] + config.lane_width_bev])
                    is_lane_feasible = evaluate_lane_width(l_coef, r_coef)
                    if is_lane_feasible:
                        print("\tParallel OK after copying right lane")
                        left_x_new = right_x_new + config.lane_width_bev
                        left_y_new = right_y_new
                        cur_l_curvature = cur_r_curvature
                        l_survived = True
                        r_survived = True
                
            #- text = "Not parallel"
            #- org = (50, 150)
            #- output_image_poly = cv2.putText(output_image_poly, text, org, font, font_scale, color, thickness, cv2.LINE_AA)
        else: # (remove for actual deployment)
            print("\t평행 O.") # (remove for actual deployment)
    #! 현재까지, 곡률, 차선 폭, 평행성 확인됨.
        
    
    #* copy if not survived
    if l_survived and not r_survived: # only left lane survived
        # 계수 이동: c1을 차선 폭만큼 조정하여 새로운 c2 생성
        r_coef = np.poly1d([l_coef.c[0], l_coef.c[1], l_coef.c[2] + config.lane_width_bev])
        # 점 이동: x 좌표를 차선 폭만큼 이동
        right_x_new = left_x_new + config.lane_width_bev
        right_y_new = left_y_new  # y 좌표는 그대로 유지
        cur_r_curvature = cur_l_curvature
        
        print("\t차선 r이 불안정하여 차선 l을 이동시켜 대체.")
        
        #- text = "copy l"
        #- org = (50, 200)
        #- output_image_poly = cv2.putText(output_image_poly, text, org, font, font_scale, color, thickness, cv2.LINE_AA)

    elif not l_survived and r_survived: # only right lane survived
        # 계수 이동: c1을 차선 폭만큼 조정하여 새로운 c2 생성
        l_coef.c[2] = r_coef.c[2] + config.lane_width_bev
        # 점 이동: x 좌표를 차선 폭만큼 이동
        left_x_new = right_x_new + config.lane_width_bev
        left_y_new = right_y_new  # y 좌표는 그대로 유지
        cur_l_curvature = cur_r_curvature
        
        print("\t차선 l이 불안정하여 차선 r을 이동시켜 대체.")
        
        #- text = "copy r"
        #- org = (50, 250)
        #- output_image_poly = cv2.putText(output_image_poly, text, org, font, font_scale, color, thickness, cv2.LINE_AA)
    #! 현재까지, 곡률, 차선 폭, 평행성, 확인됨.
    
    #* history check
    if l_survived and r_survived:
        if len(a_history) == 15:
            history_check, current_state, estimated_current_state = history_state_comparison(l_coef, a_history)
            if history_check:
                if estimated_current_state == "left_curve" and current_state == "right_curve":
                    print("\tHistory check failed")
                    a_history.append(prev_coef_l.c[0])
                elif estimated_current_state == "right_curve" and current_state == "left_curve":
                    print("\tHistory check failed")
                    a_history.append(prev_coef_l.c[0])
                else:
                    print("\tHistory check passed")
                    a_history.append(l_coef.c[0])
                
                #- text = "pass"
                #- org = (100, 50)
                #- output_image_after_valid = cv2.putText(output_image_after_valid, text, org, font, font_scale, color, thickness, cv2.LINE_AA)
            
            else:
                print("\tHistory check failed")
                
                l_survived = False
                r_survived = False

                #- text = "fail"
                #- org = (100, 50)
                #- output_image_after_valid = cv2.putText(output_image_after_valid, text, org, font, font_scale, color, thickness, cv2.LINE_AA)
    
                a_history.append(l_coef.c[0])


    #* 각 계수 한계점 확인.
    if l_survived and r_survived and len(a_history) == 16:
        if not coef_thres_check(l_coef, r_coef, current_state, estimated_current_state):
            print("\tCoef thres check failed")
            l_survived = False
            r_survived = False
        else:
            print("\tCoef thres OK.")

    #* Checks end.
    if not l_survived or not r_survived:
        cur_l_curvature = prev_curvature_l
        cur_r_curvature = prev_curvature_r
        l_coef = prev_coef_l
        r_coef = prev_coef_r
        l_pts = prev_pts_l
        r_pts = prev_pts_r
        
        left_x_new, left_y_new, l_coef = fit_lanes_and_draw(prev_pts_l, (0, 0, 255))
        right_x_new, right_y_new, r_coef = fit_lanes_and_draw(prev_pts_r, (255, 0, 0))
        
        
    for i in range(len(left_x_new) - 1):
        cv2.line(output_image_after_valid, (int(left_x_new[i]), int(left_y_new[i])), (int(left_x_new[i + 1]), int(left_y_new[i + 1])), (0,0,255), 3)
    for i in range(len(right_x_new) - 1):
        cv2.line(output_image_after_valid, (int(right_x_new[i]), int(right_y_new[i])), (int(right_x_new[i + 1]), int(right_y_new[i + 1])), (255,0,0), 3)

        
    # cv2.imshow("SW points", output_image_poly)
    cv2.imshow('SW', output_image) # contains points found by SW
    cv2.imshow('Method 2', output_image_after_valid) # after all checks (either prev or cur lanes)
    # cv2.imwrite("sw/"+str(config.idx)+".png", output_image_poly)
    # cv2.imwrite("s/"+str(config.idx)+".png", output_image_after_valid)
    # print(config.idx)
    # config.idx += 1
    
    return cur_l_curvature, cur_r_curvature, l_coef, r_coef, l_pts, r_pts




def filter_lines_initial(lines, current_section, temp2, temp, all_lines, real_all_lines):
    '''
    #- Function for finding the entire initial lanes. (contains filtering)
    #- 1) x-value of lane < half of image width --> belongs to the left lane candidate.
    #-    x-value of lane > half of image width --> belongs to the right lane candidate.
    #- 2) Among candidates, take a pair which has plausible lane width and tends to be parallel.
    '''
    """ called from each section """
    left_lines = []
    right_lines = []
    # print("filter")
    if lines is not None:
        for line in lines:
            extended_line = extend_lines_fit_section(line[0], current_section)       
            if extended_line is not None:
                x1, y1, x2, y2 = line[0]
                x1, y1, x2, y2 = extended_line
                ext_line_angle = angle(x1,y1,x2,y2)

                if config.min_angle_init < ext_line_angle < config.max_angle_init: 
                    ''' 이미지의 왼쪽 절반 = 왼쪽 차선 / 오른쪽 절반 = 오른쪽 차선 '''
                    if x1 < temp.shape[1]//2:
                        left_lines.append(extended_line)
                    else:
                        right_lines.append(extended_line)

        # Drawing selected lines in each section
        for l_line, r_line in zip(left_lines, right_lines): # (remove for actual deployment)
            cv2.line(temp, (l_line[0],l_line[1]), (l_line[2],l_line[3]), (0,255,0), 2) # (remove for actual deployment)
            cv2.line(temp, (r_line[0],r_line[1]), (r_line[2],r_line[3]), (0,255,0), 2) # (remove for actual deployment)


    # 차선의 폭 & parallelism & lane thickness을 기준으로 필터링
    best_left_line = []
    best_right_line = []
    min_diff = float('inf')

    for l_line in left_lines:
        for r_line in right_lines:
            distance = distance_between_lines(l_line, r_line)
            diff = abs(distance - config.lane_width_bev)
            l_end_x_diff = abs(l_line[0] - l_line[2])
            r_end_x_diff = abs(r_line[0] - r_line[2])
            
            if config.lane_width_bev * 0.8 <= distance <= config.lane_width_bev * 1.2 and diff < min_diff\
                and l_end_x_diff < config.validate_lane_endpt_diff and r_end_x_diff < config.validate_lane_endpt_diff:
                angle_l = angle(l_line[0], l_line[1], l_line[2], l_line[3])
                angle_r = angle(r_line[0], r_line[1], r_line[2], r_line[3])
                if abs(angle_l - angle_r) < 10: # 기울기 차이가 작을수록 평행에 가까움
                    best_left_line = l_line
                    best_right_line = r_line
                    min_diff = diff
    
    real_all_lines.append([best_left_line, best_right_line])
    all_lines.append([best_left_line, best_right_line])
        
    #* Drawing filtered lines
    for i in range(len(real_all_lines)):
        if len(real_all_lines[i][0]) != 0:
            cv2.line(temp, (real_all_lines[i][0][0], real_all_lines[i][0][1]), (real_all_lines[i][0][2], real_all_lines[i][0][3]), (0,255,255), 2)
            cv2.line(temp2, (real_all_lines[i][0][0], real_all_lines[i][0][1]), (real_all_lines[i][0][2], real_all_lines[i][0][3]), (0,255,255), 2)
        if len(real_all_lines[i][1]) != 0:
            cv2.line(temp, (real_all_lines[i][1][0], real_all_lines[i][1][1]), (real_all_lines[i][1][2], real_all_lines[i][1][3]), (0,255,255), 2)
            cv2.line(temp2, (real_all_lines[i][1][0], real_all_lines[i][1][1]), (real_all_lines[i][1][2], real_all_lines[i][1][3]), (0,255,255), 2)
    
    # config.idx += 1
    
def filter_lines(lines, prev_Q_l, prev_Q_r, current_section, temp2, temp, no_line_cnt_l, no_line_cnt_r, all_lines, real_all_lines):
    '''
    #- prev_angle & cur_angle comparison
    #- parallelism & width check
    '''
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
    

    closest_l_line = []
    closest_r_line = []
    comparison_distance_l = np.inf
    comparison_angle_l = np.inf
    comparison_distance_r = np.inf
    comparison_angle_r = np.inf
    
    # print(no_line_cnt_l, no_line_cnt_r)
    if no_line_cnt_l >= 20:
        #- print("NO LEFT LINE DETECTED for 20F --> SETTING SEARCH RANGE to 100")
        config.min_abs_distance_l = 100
        config.min_angle_diff = 40
    elif no_line_cnt_l >= 10:
        #- print("NO LEFT LINE DETECTED for 10F --> SETTING SEARCH RANGE to 50")
        config.min_abs_distance_l = 50
        # min_slope_diff = 30
        
    if no_line_cnt_r >= 20:
        #- print("NO RIGHT DETECTED for 20F --> SETTING SEARCH RANGE to 100")
        config.min_abs_distance_r = 100
        config.min_angle_diff = 40
    elif no_line_cnt_r >= 10:
        #- print("NO RIGHT DETECTED for 10F --> SETTING SEARCH RANGE to 50")
        config.min_abs_distance_r = 50
        # min_slope_diff = 30
    
    # Compare all lanes with previous lanes
    if lines is not None:
        for line in lines:
            extended_line = extend_lines_fit_section(line[0], current_section)       
            if extended_line is not None:
                x1, y1, x2, y2 = line[0] # just for visualizations
                # cv2.line(temp2, (x1, y1), (x2, y2), (0, 255, 0), 1)
                cv2.line(temp, (x1, y1), (x2, y2), (0, 255, 0), 1)
                x1, y1, x2, y2 = extended_line
                ext_line_angle = angle(x1,y1,x2,y2)
                
                # Compute angle and distance differences from previous lanes
                angle_diff_Q_l = abs(prev_l_angle - ext_line_angle)
                angle_diff_Q_r = abs(prev_r_angle - ext_line_angle)
                distance_diff_Q_l, mp1, mp2 = calculate_distance(prev_l_x1, prev_l_y1, prev_l_x2, prev_l_y2, x1,y1,x2,y2)
                # cv2.circle(temp, (int(mp2[0]), int(mp2[1])), 3, (255,0,255),-1)
                # cv2.circle(temp, (int(mp1[0]), int(mp1[1])), 5, (255,255,0),-1)
                distance_diff_Q_r, mp1, mp2 = calculate_distance(prev_r_x1, prev_r_y1, prev_r_x2, prev_r_y2, x1,y1,x2,y2)
                # cv2.circle(temp, (int(mp2[0]), int(mp2[1])), 3, (255,0,255),-1)
                # cv2.circle(temp, (int(mp1[0]), int(mp1[1])), 5, (255,255,0),-1)

                # left line first
                if distance_diff_Q_l < config.min_abs_distance_l and distance_diff_Q_l < comparison_distance_l and\
                    angle_diff_Q_l < config.min_angle_diff and angle_diff_Q_l < comparison_angle_l:
                    comparison_distance_l = distance_diff_Q_l
                    comparison_angle_l = angle_diff_Q_l
                    closest_l_line = extended_line
                # right line afterward
                elif distance_diff_Q_r < config.min_abs_distance_r and distance_diff_Q_r < comparison_distance_r and\
                      angle_diff_Q_r < config.min_angle_diff and angle_diff_Q_r < comparison_angle_r:
                    comparison_distance_r = distance_diff_Q_r
                    comparison_angle_r = angle_diff_Q_r
                    closest_r_line = extended_line

                #* Filtering as in initial lane search (**only possible when both lanes are found)
                if len(closest_l_line) != 0 and len(closest_r_line) != 0:
                    # print(closest_l_line, closest_r_line)
                    distance_bw_cur_lanes = distance_between_lines(closest_l_line, closest_r_line)
                    angle_l = angle(closest_l_line[0], closest_l_line[1], closest_l_line[2], closest_l_line[3])
                    angle_r = angle(closest_r_line[0], closest_r_line[1], closest_r_line[2], closest_r_line[3])
                    if not (config.lane_width_bev * 0.8 <= distance_bw_cur_lanes <= config.lane_width_bev * 1.2 and abs(angle_l - angle_r) < 10):
                        print(f"S({current_section}) Lanes are found, but deleted because of either non-parallelism and width")
                        print(f"\t{distance_bw_cur_lanes}, {angle_l}, {angle_r}")
                        print(f"\tdist_diff: {config.lane_width_bev * 0.8 <= distance_bw_cur_lanes <= config.lane_width_bev * 1.2}, ang_diff: {abs(angle_l - angle_r) < 10}")
                        closest_l_line = []
                        closest_r_line = []
                #****** 2024.08.08       
                
            
        #*-----Drawing selected lines in each section
        if len(closest_l_line) != 0:         
            cv2.line(temp, (closest_l_line[0],closest_l_line[1]), (closest_l_line[2],closest_l_line[3]), (255, 165, 0), 2)
        if len(closest_r_line) != 0:
            cv2.line(temp, (closest_r_line[0],closest_r_line[1]), (closest_r_line[2],closest_r_line[3]), (255, 165, 0), 2)

    real_all_lines.append([closest_l_line, closest_r_line])
    all_lines.append([closest_l_line, closest_r_line])
    
    # cv2.imshow("after filter_lines",temp)
    # cv2.imshow("after filter_lines2",temp2)
    
    
    '''
    if only one lane found:
        copy
    elif both not found
        use prev ones
    '''
    # # Using prev cp as lines in missing section (X connecting with detected line using slope)
    # for i in range(len(all_lines)):
    #     if len(all_lines[i][0]) == 0:
    #         all_lines[i][0] = [prev_l_x1, prev_l_y1, prev_l_x2, prev_l_y2]
    #         cv2.line(temp2, (all_lines[i][0][0],all_lines[i][0][1]), (all_lines[i][0][2],all_lines[i][0][3]), (0,0,255), 2)
    #     if len(all_lines[i][1]) == 0:
    #         all_lines[i][1] = [prev_r_x1, prev_r_y1, prev_r_x2, prev_r_y2]
    #         cv2.line(temp2, (all_lines[i][1][0],all_lines[i][1][1]), (all_lines[i][1][2],all_lines[i][1][3]), (0,0,255), 2)

    # #* 
    # if current_section == 1:
    #     sec_0_dist_dif = None
    #     # print(all_lines)
    #     for i in range(len(all_lines)):
    #         # If lane angle difference is larger than 5, delete it  
    #         if len(all_lines[i][0]) != 0 and len(all_lines[i][1]) != 0:
    #             angle_l = angle(all_lines[i][0][0], all_lines[i][0][1], all_lines[i][0][2], all_lines[i][0][3])
    #             angle_r = angle(all_lines[i][1][0], all_lines[i][1][1], all_lines[i][1][2], all_lines[i][1][3])
    #             angle_diff = abs(angle_l - angle_r)
    #             # print("angle diff: ", angle_diff)
    #             if angle_diff >= 100: # 10
    #                 # print(f"TOO LARGE ANGLE DIFFERENCE ({angle_diff}) --> DELETING")
    #                 real_all_lines[i][0] = []
    #                 real_all_lines[i][1] = []
                    
    #         # If lane width is smaller or larger than usual, delete it
    #         if len(all_lines[i][0]) != 0 and len(all_lines[i][1]) != 0:
    #             distance_l_r,_,_ = calculate_distance(all_lines[i][0][0], all_lines[i][0][1], all_lines[i][0][2], all_lines[i][0][3], all_lines[i][1][0], all_lines[i][1][1], all_lines[i][1][2], all_lines[i][1][3])
    #             #- print("dist diff:",distance_l_r)
    #             if i == 0:
    #                 sec_0_dist_dif = distance_l_r
    #             # else:
    #             #     if sec_0_dist_dif - distance_l_r > 0: # if dist in sec 0 is larger than in sec 1 is checking non-sense lane states
    #             #         print(f"NON SENSE LINE (S0) ({sec_0_dist_dif}, {distance_l_r})--> DELETING")
    #             #         real_all_lines[0][0] = []
    #             #         real_all_lines[0][1] = []
    #                 # if distance_l_r <= 160 or distance_l_r >= 280:    # 검증 단계에서 생각나는 아이디어 있다면 쓰기
    #                 #     print(f"NARROW LANE WIDTH (S1)({distance_l_r})--> DELETING")
    #                 #     real_all_lines[1][0] = []
    #                 #     real_all_lines[1][1] = []
 
def validate_lane(temp2, temp, all_lines):
    if all_lines[0][0] and all_lines[1][0]: # full left lane found
        upper_line_ang = angle(all_lines[0][0][0], all_lines[0][0][1], all_lines[0][0][2], all_lines[0][0][3])
        lower_line_ang = angle(all_lines[1][0][0], all_lines[1][0][1], all_lines[1][0][2], all_lines[1][0][3])
        if abs(upper_line_ang - lower_line_ang) < config.min_angle_diff and abs(all_lines[0][0][2] - all_lines[1][0][0]) > config.validate_lane_endpt_diff:
            # if section lines are tend to be parallel but located far from each other, remove
            print("(L) Far from each other --> Deleted")
            print(f"\t{abs(upper_line_ang - lower_line_ang)} < {config.min_angle_diff}, {abs(all_lines[0][0][2] - all_lines[1][0][0])} > config.validate_lane_endpt_diff")
            print(f"\t{abs(upper_line_ang - lower_line_ang) < config.min_angle_diff}, {abs(all_lines[0][0][2] - all_lines[1][0][0]) > config.validate_lane_endpt_diff}")
            cv2.line(temp, (all_lines[0][0][0], all_lines[0][0][1]), (all_lines[0][0][2], all_lines[0][0][3]), (0, 255, 0), 2)
            cv2.line(temp, (all_lines[1][0][0], all_lines[1][0][1]), (all_lines[1][0][2], all_lines[1][0][3]), (0, 255, 0), 2)
            all_lines[0][0] = []
            all_lines[1][0] = []

    if all_lines[0][1] and all_lines[1][1]:
        upper_line_ang = angle(all_lines[0][1][0], all_lines[0][1][1], all_lines[0][1][2], all_lines[0][1][3])
        lower_line_ang = angle(all_lines[1][1][0], all_lines[1][1][1], all_lines[1][1][2], all_lines[1][1][3])
        if abs(upper_line_ang - lower_line_ang) < config.min_angle_diff and abs(all_lines[0][1][2] - all_lines[1][1][0]) > config.validate_lane_endpt_diff:
            # if section lines are tend to be parallel but located far from each other, remove
            print("(R) Far from each other --> Deleted")
            print(f"\t{abs(upper_line_ang - lower_line_ang)} < {config.min_angle_diff}, {abs(all_lines[0][1][2] - all_lines[1][1][0])} > config.validate_lane_endpt_diff")
            print(f"\t{abs(upper_line_ang - lower_line_ang) < config.min_angle_diff}, {abs(all_lines[0][1][2] - all_lines[1][1][0]) > config.validate_lane_endpt_diff}")
            cv2.line(temp, (all_lines[0][1][0], all_lines[0][1][1]), (all_lines[0][1][2], all_lines[0][1][3]), (0, 255, 0), 2)
            cv2.line(temp, (all_lines[1][1][0], all_lines[1][1][1]), (all_lines[1][1][2], all_lines[1][1][3]), (0, 255, 0), 2)
            all_lines[0][1] = []
            all_lines[1][1] = [] 
     
    # cv2.imshow("after validate_lane",temp)
     
     
def copy_one_full_lane(temp2, temp, all_lines):
    # print("**copy_one_full_lane**")
    '''
    if only one lane found:
        copy
    '''
    lane_full = (all(sublist for sublist in all_lines[0]) and all(sublist for sublist in all_lines[1]))
    left_full = len(all_lines[0][0]) > 0 and len(all_lines[1][0]) > 0
    right_full = len(all_lines[0][1]) > 0 and len(all_lines[1][1]) > 0
    # print(all_lines)
    # print(f"lane_full: {lane_full}")
    # print(f"left_full: {left_full}")
    # print(f"right_full: {right_full}")
    if (not lane_full and left_full and not right_full) or (not lane_full and right_full and not left_full): # only one full lane is found
        empty_indices = [(i, j) for i, outer_list in enumerate(all_lines) for j, inner_list in enumerate(outer_list) if not inner_list]
        # cv2.imwrite("b/"+str(config.idx)+".png", temp)
        # config.idx+=1
        # print(all_lines)
        if empty_indices[0][1] == 0: # left lane is empty
            all_lines[0][0] = copy.deepcopy(all_lines[0][1])
            all_lines[1][0] = copy.deepcopy(all_lines[1][1])
            all_lines[0][0][0] -= int(config.lane_width_bev)
            all_lines[0][0][2] -= int(config.lane_width_bev)
            all_lines[1][0][0] -= int(config.lane_width_bev)
            all_lines[1][0][2] -= int(config.lane_width_bev)
            cv2.line(temp, (all_lines[0][0][0], all_lines[0][0][1]), (all_lines[0][0][2], all_lines[0][0][3]), (0, 165, 255), 2)
            cv2.line(temp, (all_lines[1][0][0], all_lines[1][0][1]), (all_lines[1][0][2], all_lines[1][0][3]), (0, 165, 255), 2)
        else: # right lane is empty
            all_lines[0][1] = copy.deepcopy(all_lines[0][0])
            all_lines[1][1] = copy.deepcopy(all_lines[1][0])
            all_lines[0][1][0] += int(config.lane_width_bev)
            all_lines[0][1][2] += int(config.lane_width_bev)
            all_lines[1][1][0] += int(config.lane_width_bev)
            all_lines[1][1][2] += int(config.lane_width_bev)
            cv2.line(temp, (all_lines[0][1][0], all_lines[0][1][1]), (all_lines[0][1][2], all_lines[0][1][3]), (0, 165, 255), 2)
            cv2.line(temp, (all_lines[1][1][0], all_lines[1][1][1]), (all_lines[1][1][2], all_lines[1][1][3]), (0, 165, 255), 2)

        # cv2.imshow("copy_one_full_lane", temp)
        # cv2.waitKey(4000)
        

def extract_lines_in_section_initial(roi, no_line_cnt):
    ''' Note: Vehicle should start on the straight lane '''
    temp = np.copy(roi)
    temp = cv2.cvtColor(temp, cv2.COLOR_GRAY2BGR)
    temp2 = np.copy(temp) # for drawing hough lines

    all_lines = []
    all_lines_for_filtering = [] # (remove for actual deployment)

    for i in range(len(config.section_list)-1): #* for each section
        separated = np.copy(roi)
        separated = roi_extractor(separated, 0, config.section_list[i], separated.shape[1], config.section_list[i+1]) #? extract each section from img
        lines = cv2.HoughLinesP(separated, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=100)
        filter_lines_initial(lines, i, temp2, temp, all_lines_for_filtering, all_lines) 
    
    # if lines found in all sections
    if len(all_lines[0][0]) != 0 and len(all_lines[0][1]) != 0 and len(all_lines[1][0]) != 0 and len(all_lines[1][1]) != 0:
        print("&&&&& FOUND")
        config.initial_not_found = False
            
    Q_l, Q_r = control_point(all_lines)
    for point_l, point_r in zip(Q_l, Q_r):
        if len(point_l) > 0:
            cv2.circle(temp2, (int(point_l[0]), int(point_l[1])), 7, (255, 255, 255), 2)
            cv2.circle(temp, (int(point_l[0]), int(point_l[1])), 7, (255, 255, 255), 2)
        if len(point_r) > 0:
            cv2.circle(temp2, (int(point_r[0]), int(point_r[1])), 7, (255, 255, 255), 2)
            cv2.circle(temp, (int(point_r[0]), int(point_r[1])), 7, (255, 255, 255), 2)
    
    # cv2.imwrite("a/"+str(config.idx)+".png", temp)    
    config.idx+=1
    
    return all_lines, temp, Q_l, Q_r



def extract_lines_in_section(roi, prev_Q_l, prev_Q_r, no_line_cnt):
    temp = np.copy(roi)
    temp = cv2.cvtColor(temp, cv2.COLOR_GRAY2BGR)
    temp2 = np.copy(temp) # for drawing hough lines

    all_lines = []
    all_lines_for_filtering = []
    
    for i in range(len(config.section_list)-1): #* for each section
        separated = np.copy(roi)
        separated = roi_extractor(separated, 0, config.section_list[i], separated.shape[1], config.section_list[i+1]) #? extract each section from img
        lines = cv2.HoughLinesP(separated, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=100) # Probabilistic Hough Transform
        
        filter_lines(lines, prev_Q_l, prev_Q_r, i, temp2, temp, no_line_cnt[2*i], no_line_cnt[2*i+1], all_lines_for_filtering, all_lines) #todo pass line dist threshold
    # print(all_lines)
    
    #* Check whether sections' endpoints are too far (nonsense lane)
    validate_lane(temp2, temp, all_lines)
    #* Copy a lane if only one lane is found
    copy_one_full_lane(temp2, temp, all_lines)
    #******* 2024.08.08
    #!====
    # cv2.imshow("after filter",temp)
    # cv2.imshow("after filter2",temp2)
    #!====
    
    # for point_l, point_r in zip(prev_Q_l, prev_Q_r): #* Previous control pts
    #     # Draw circles on temp2
    #     # cv2.circle(temp2, (int(point_r[0]), int(point_r[1])), 7, (0, 0, 255), 2)
    #     # cv2.circle(temp2, (int(point_l[0]), int(point_l[1])), 7, (0, 0, 255), 2)
    #     # Draw circles on temp
    #     cv2.circle(temp, (int(point_l[0]), int(point_l[1])), 7, (0, 0, 255), 2)
    #     cv2.circle(temp, (int(point_r[0]), int(point_r[1])), 7, (0, 0, 255), 2)
    
    # print("alllines:",all_lines)
    Q_l, Q_r = control_point(all_lines)
    # print("CP", Q_l, Q_r)
    
    # for point_l, point_r in zip(Q_l, Q_r): #* Current control pts
    #     if len(point_l) > 0:
    #         # cv2.circle(temp2, (int(point_l[0]), int(point_l[1])), 7, (255, 255, 255), 2)
    #         cv2.circle(temp, (int(point_l[0]), int(point_l[1])), 7, (255, 255, 255), 2)
    #     if len(point_r) > 0:
    #         # cv2.circle(temp2, (int(point_r[0]), int(point_r[1])), 7, (255, 255, 255), 2)
    #         cv2.circle(temp, (int(point_r[0]), int(point_r[1])), 7, (255, 255, 255), 2)

        
    # cv2.imshow("Lanes before filtering", temp2)
    # cv2.imshow("Searching end", temp)
    
    return all_lines, temp, Q_l, Q_r



def R_set_considering_control_points(Q_l, Q_r, prev_esti, no_line_cnt):
    weight = [10, 10, 200, 10, 10, 200]
    # print("prev esti\n", prev_esti)
    
    if prev_esti is not None:
        R_ = np.zeros((6, 1))
        for i in range(len(Q_l)):
            if len(Q_l[i]) > 0:
                dist_diff = abs(Q_l[i][0] - prev_esti[i])
                R_[i] = dist_diff * weight[i] + 1e-6
            else: # if no control point
                R_[i] = math.inf
                
            if len(Q_r[i]) > 0:
                diff = abs(Q_r[i][0] - prev_esti[i+3])
                R_[i+3] = diff * weight[i+3] + 1e-6
            else: # if no control point
                R_[i+3] = math.inf
                
        return np.diag(R_.reshape((6,)))
    
    return np.diag(1000 * np.ones(6))



def no_line_cnt_update(no_line_cnt, lines_in_section):
    for i in range(len(lines_in_section)): # 1st: section 1 / 2nd: section 2
        for j in range(2): # 1st: left / 2nd: right
            if len(lines_in_section[i][j]) == 0:
                no_line_cnt[2*i+j] += 1
            else:
                no_line_cnt[2*i+j] = 0


def merge(bin, prev_Q_l, prev_Q_r, sw_cur_coef_l, sw_cur_coef_r, Q_l, Q_r, a_history):
    bin = cv2.cvtColor(bin, cv2.COLOR_GRAY2RGB)
    bin2 = bin.copy()

    # B-spline 제어점을 사용하여 2차 다항식으로 근사
    def bspline_to_poly(control_points, degree=2):
        # 빈 리스트 제거
        control_points_filtered = [point for point in control_points if point]
        
        if len(control_points_filtered) < 2:
            print("Not enough points to form a valid polynomial.")
            return None
        
        # numpy array로 변환
        control_points_filtered = np.array(control_points_filtered)

        # x, y 좌표 분리
        x = control_points_filtered[:, 0]
        y = control_points_filtered[:, 1]
        
        # 다항식 근사 (y를 독립 변수, x를 종속 변수로)
        coeffs = np.polyfit(y, x, degree, full=False)
        
        return coeffs
            
    # 두 결과 간의 유사성을 평가하는 함수
    def evaluate_similarity(prev_poly_left, prev_poly_right, sw_cur_coef_l, sw_cur_coef_r, current_poly_left, current_poly_right):
        '''
        coeffs_1: SW
        coeffs_2: 1st method
        '''
        
        if current_poly_left is None or current_poly_right is None:
            print("\tSW chosen.")
            #todo if a diff, use prev
            return sw_cur_coef_l, sw_cur_coef_r
    
        sw_a_diff = abs(sw_cur_coef_l.c[0] - sw_cur_coef_l.c[0])
        cp_a_diff = abs(current_poly_left[0] - sw_cur_coef_r[0])

        if sw_a_diff < cp_a_diff:
            return sw_cur_coef_l, sw_cur_coef_r
        elif sw_a_diff > cp_a_diff:
            return current_poly_left, current_poly_right
        
        
    
    def draw_polyline_from_polycoeff(img, poly_coeffs, Q, color, thickness=2):
        if poly_coeffs is None:
            return img

        non_empty_indices = [index for index, sublist in enumerate(Q) if sublist]

        if non_empty_indices == [0,1]:
            y_min = 0
            y_max = 199
        elif non_empty_indices == [1,2]:
            y_min = 199
            y_max = 499
        else:
            # Default to full image height if no match
            y_min = 0
            y_max = 499
        
        y_new = np.linspace(y_min, y_max, num=100) # y 범위 내에서 100개의 점 생성
        x_new = np.polyval(poly_coeffs, y_new).astype(int)
        
        points = np.array([[int(x), int(y)] for x, y in zip(x_new, y_new)])
        
        # 피팅한 곡선 그리기
        if len(points) > 1:
            cv2.polylines(img, [points], isClosed=False, color=color, thickness=thickness)
        
        return img
    
    # 각각의 y 값에 대한 x 값을 계산하는 함수
    def calculate_x_for_y(coeffs, y_values):
        x_values = []
        for y in y_values:
            x = np.polyval(coeffs, y)
            x_values.append([int(x), y])
        return x_values
    
    
        
    # B-spline 제어점을 사용하여 다항식으로 근사
    current_poly_left_2 = bspline_to_poly(Q_l)
    current_poly_right_2 = bspline_to_poly(Q_r)
    prev_poly_left = bspline_to_poly(prev_Q_l)
    prev_poly_right = bspline_to_poly(prev_Q_r)
    
    # 이미지에 다항식으로 그리기 (temp)
    img = draw_polyline_from_polycoeff(bin, current_poly_left_2, Q_l, (0,0,255), thickness=2)
    img = draw_polyline_from_polycoeff(img, current_poly_right_2, Q_r, (255,0,0), thickness=2)
    cv2.imshow("CP", img)
    # cv2.imwrite("s/"+str(config.idx)+".png", img)
    
    # state estimation
    import statistics
    if len(a_history) == 16:
        mean_a_history = statistics.mean(a_history)
        
        # history state
        if abs(mean_a_history) < 1e-4:
            estimated_current_state = "straight"
            estimated_current_detailed_state = "straight"
        else:
            if mean_a_history < 0:
                estimated_current_state = "left_curve"
                if abs(mean_a_history) < 0.0003:
                    estimated_current_detailed_state = "before_in_curve"
                else:
                    estimated_current_detailed_state = "in_curve"
            else:
                estimated_current_state = "right_curve"
                if abs(mean_a_history) < 0.0003:
                    estimated_current_detailed_state = "before_in_curve"
                else:
                    estimated_current_detailed_state = "in_curve"
                    
        # current state
        if abs(sw_cur_coef_l.c[0]) < 1e-4:
            current_state = "straight"
        else:
            if abs(sw_cur_coef_l.c[0]) < 0:
                current_state = "left_curve"
            else:
                current_state = "right_curve"
            
        # print(estimated_current_state, estimated_current_detailed_state)
        # print(current_state)
        
        if current_state == "straight":
            # 두 결과 간 유사성 평가
            final_left_coef, final_right_coef = evaluate_similarity(prev_poly_left, prev_poly_right, sw_cur_coef_l, sw_cur_coef_r, current_poly_left_2, current_poly_right_2)
        elif current_state == "right_curve" or current_state == "left_curve":
            final_left_coef = sw_cur_coef_l
            final_right_coef = sw_cur_coef_r
    
    else: # a_history doesn't have 15 elements yet
        final_left_coef, final_right_coef = evaluate_similarity(prev_poly_left, prev_poly_right, sw_cur_coef_l, sw_cur_coef_r, current_poly_left_2, current_poly_right_2)
        
            
            
    # Convert coef to CP
    y_values = [0, 199, 499]

    # 왼쪽 차선에 대한 x 값 계산 (CP)
    Q_l = calculate_x_for_y(final_left_coef, y_values)
    # 오른쪽 차선에 대한 x 값 계산 (CP)
    Q_r = calculate_x_for_y(final_right_coef, y_values)

    img2 = draw_polyline_from_polycoeff(bin2, final_left_coef, Q_l, (0,0,255), thickness=4)
    img2 = draw_polyline_from_polycoeff(img2, final_right_coef, Q_r, (255,0,0), thickness=4)
    
    for i in range(len(Q_l)):
        cv2.circle(img2, tuple(Q_l[i]), 10, (0, 255, 0), 2)
        cv2.circle(img2, tuple(Q_r[i]), 10, (0, 255, 0), 2)

    cv2.imshow("Chosen method", img2)

    return Q_l, Q_r

# -------------------- method functions(해당 파일 안에서 호출) ---------------------------------

def check_Q(Q_l, Q_r):
    print(Q_l, Q_r)
    for i in range(len(Q_l)):
        width = abs(Q_l[i][0] - Q_r[i][0])
        if config.lane_width_bev - 30 < width < config.lane_width_bev + 30:
            return False
        if Q_l[i][0] > Q_r[i][0]:
            return False
        
    return True

def gaussian_transform(image, mu, sigma):
    img_float = image.astype(np.float32)
    gaussian_img = np.exp(-0.5 * ((img_float - mu) / sigma) ** 2) * 255
    return gaussian_img.astype(np.uint8)

def roi_extractor(img, x1, y1, x2, y2):
    mask = np.zeros_like(img)
    mask[y1:y2, x1:x2] = 255
    roi = cv2.bitwise_and(img, mask)
    return roi


def slope(x1, y1, x2, y2):
    # if x2 - x1 == 0:
    #     return float('inf')
    # else:
    return (y2 - y1) / (x2 - x1 + 1e-6)
    

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
    x1, y1, x2, y2 = points[0], points[1], points[2], points[3]
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def extend_lines_fit_section(points, current_section):
    m, c = line_equation(points)
    
    if line_length(points) < 70:
        return None
        
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
    """ Calculate the Euclidean distance between the midpoints of two lines """
    midpoint1 = ((x1 + x2) / 2, (y1 + y2) / 2)
    midpoint2 = ((x3 + x4) / 2, (y3 + y4) / 2)
    return np.sqrt((midpoint2[0] - midpoint1[0])**2 + (midpoint2[1] - midpoint1[1])**2), midpoint1, midpoint2
    
def distance_between_lines(line1, line2):
    x1, _, x2, _ = line1
    x3, _, x4, _ = line2
    return abs((x2 + x1) / 2 - (x4 + x3) / 2)

def draw_hough_lines(img, lines, color=(0, 255, 0), thickness=2):
    line_img = np.copy(img)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
    return line_img


def control_point(all_lines):
    # print("CP", all_lines)
    """Averaging two points"""
    # all_lines[][][]에서
    # 첫번째(0/1) -> 0이면 위, 1이면 아래
    # 두번째(0/1) -> 0이면 왼쪽, 1이면 오른쪽
    # 세번째(0~3) -> 0,1이 위쪽의 x,y좌표, 2,3이 아래의 x,y좌표
    Q_l = []
    Q_r = []

    for i in range(len(all_lines)):
        if i == 0: # first line
            if len(all_lines[i][0]) > 0: # left line in section 1
                Q_l.append([all_lines[i][0][0], all_lines[i][0][1]])
                Q_l.append([all_lines[i][0][2], all_lines[i][0][3]])
            else:
                Q_l.append([])
            if len(all_lines[i][1]) > 0: # right line in section 1
                Q_r.append([all_lines[i][1][0], all_lines[i][1][1]])
                Q_r.append([all_lines[i][1][2], all_lines[i][1][3]])
            else:
                Q_r.append([])
        
        else:  # second line
            if len(all_lines[i][0]) > 0: # if left exists
                if len(Q_l) == 1: # 위에 라인이 존재할때(1아니면 2니까 라인 없을때) -> 밑에 라인 끝점 그대로
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
    # print(Q_l, Q_r)
    # print()
    return Q_l, Q_r