import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
import cv2

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


#* New: B-spline -> Line fitting
def bspline(Q, img, color):
    # 다항식 계수 계산
    coeffs = bspline_to_poly(Q)
    
    if coeffs is None:
        print("Failed to generate polynomial.")
        return img
    
    # y 값의 범위를 이미지 높이에 맞게 설정
    y_values = np.arange(0, img.shape[0])
    
    # 근사된 다항식을 사용하여 x 값 계산
    x_values = np.polyval(coeffs, y_values).astype(int)
    
    # x 값을 이미지 경계 내로 클리핑
    # x_values = np.clip(x_values, 0, img.shape[1]).astype(int)
    
    # 차선을 그리기 위한 좌표 배열 생성
    points = np.array([x_values, y_values]).T
    
    if len(points) > 1:
        cv2.polylines(img, [points], isClosed=False, color=color, thickness=1)
    
    return img, points



#* Original
# def bspline(Q, img, color):
#     """ 
#     Originally, although Q contains more one empty list, it only draws existing lines.
#     But, somewhat it brings error if Q has empty list.
#     Taking only the existing list solves this issue.
#     """    
#     Q_filtered = [sublist for sublist in Q if sublist]
#     if not Q_filtered:
#         return img, []

#     ctr = np.array(Q_filtered)

#     x = []
#     y = []
#     for i in range(len(ctr)):
#         if len(ctr[i]) > 0:
#             x.append(ctr[i][0])
#             y.append(ctr[i][1])
    
#     if len(x) < 2:
#         print("Not enough points to form a spline.")
#         return img, []
    
#     l=len(x)
#     Order = l-1
    
#     t=np.linspace(0,1,l-(Order-1),endpoint=True)
#     t=np.append(np.zeros(Order),t)
#     t=np.append(t,np.zeros(Order)+1)
    
#     tck=[t,[x,y],Order]
#     u3=np.linspace(0,1,(max(l*2,70)),endpoint=True)
#     out = interpolate.splev(u3,tck)
    
#     # plt.plot(x,y,'k--',label='Control polygon',marker='o',markerfacecolor='red')
#     # plt.plot(out[0],out[1],'b',linewidth=2.0,label='B-spline curve')
#     # plt.legend(loc='best')
#     # plt.axis([0, 1280, 720, 0])  # Set plot limits to image size
#     # plt.title('Cubic B-spline curve evaluation in image coordinate system')
#     # plt.show()
    
#     for x_,y_ in zip(x,y):
#         cv2.circle(img, (x_,y_), 7, color, 1)
#     points = np.int32(np.column_stack(out))
#     cv2.polylines(img, [points], False, color, 1)

#     return img, points
            
    
# img = np.zeros((500, 300, 3), dtype=np.uint8)
# Q = [[168, 0], [143, 100], [51, 500]]
# img, pts =bspline(Q, img, (0,255,0))
# cv2.imshow("ASD", img)
# cv2.waitKey(10000)