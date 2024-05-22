#거의 원본
# import cv2
# import numpy as np

# def sigmoid(x):    
#     alpha = 0.5
#     beta = 150
#     img_float = x.astype(np.float32)
#     sigmoid_img = (255 / (1 + np.exp(-alpha * (img_float - beta)))).astype(np.uint8)
#     return sigmoid_img

# def auto_canny(image, sigma=0.33):
#     v = np.median(image)
#     lower = int(max(0, (1.0 - sigma) * v))
#     upper = int(min(255, (1.0 + sigma) * v))
#     edged = cv2.Canny(image, lower, upper)
#     return edged


# image_path = '1.png'
# bev = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
# # sig = sigmoid(bev)
# # ret, thres2 = cv2.threshold(sig, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
# # k = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
# # dilate = cv2.dilate(thres2, k, iterations=2)
# # canny_dilate = auto_canny(dilate, sigma=0.2)
# lines = cv2.HoughLinesP(bev, 1, np.pi/180, threshold=40, minLineLength=20, maxLineGap=50)

# bev = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)  # bev 이미지를 컬러로 변환

# if lines is not None:
#     print(lines)
#     for line in lines:
#         x1, y1, x2, y2 = line[0]
#         cv2.line(bev, (x1, y1), (x2, y2), (0, 255, 255), 2)
#     print(f"Number of lines detected: {len(lines)}")
# else:
#     print("No lines detected")

# # 결과 이미지 표시
# cv2.imshow('Detected Lines', bev)
# cv2.waitKey(0)
# cv2.destroyAllWindows()



# sigmoid의 alpha와 beta를 바꾸어가면서 넣어보기
# import cv2
# import numpy as np
# import matplotlib.pyplot as plt

# # Sigmoid 함수 정의
# def sigmoid(x, alpha, beta):
#     img_float = x.astype(np.float32)
#     sigmoid_img = (255 / (1 + np.exp(-alpha * (img_float - beta)))).astype(np.uint8)
#     return sigmoid_img

# # Auto Canny 함수 정의
# def auto_canny(image, sigma=0.33):
#     v = np.median(image)
#     lower = int(max(0, (1.0 - sigma) * v))
#     upper = int(min(255, (1.0 + sigma) * v))
#     # print(lower, upper)
#     edged = cv2.Canny(image, lower, 255)
#     return edged

# # 이미지 파일 읽기
# image_path = 'aaa.png'
# bev = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# # 최대 선 개수와 최적의 alpha, beta 초기화
# max_lines = 0
# best_alpha = 0
# best_beta = 0
# best_image = None

# # alpha와 beta 값을 바꿔가며 테스트

# for beta in range(50, 151, 10):
#     for alpha in np.arange(0, 0.51, 0.01):
#         # Sigmoid 변환
#         sig = sigmoid(bev, alpha, beta)

#         # 이진화
#         ret, thres2 = cv2.threshold(sig, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

#         # 팽창
#         k = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
#         dilate = cv2.dilate(thres2, k, iterations=2)

#         # Canny 엣지 검출
#         canny_dilate = auto_canny(dilate, sigma=0.2)
#         cv2.imshow("A", canny_dilate)
#         cv2.waitKey(50)
#         num_labels, labels_im = cv2.connectedComponents(canny_dilate)
#         min_size = 50  # 최소 구성 요소 크기 (필요에 따라 조정)
#         new_binary_img = np.zeros_like(canny_dilate)
#         for label in range(1, num_labels):  # 0은 배경이므로 제외
#             component = (labels_im == label).astype(np.uint8) * 255
#             if cv2.countNonZero(component) > min_size:
#                 new_binary_img = cv2.bitwise_or(new_binary_img, component)
#         cv2.imshow("new_binary_img", new_binary_img)
        

#         # HoughLinesP 적용
#         lines = cv2.HoughLinesP(new_binary_img, 1, np.pi/180, threshold=40, minLineLength=250, maxLineGap=100)
#         # 검출된 선의 개수 확인 및 최적의 alpha, beta 저장
#         if lines is not None:
#             num_lines = len(lines)
#             # print(num_lines)
#             if num_lines > max_lines:
#                 max_lines = num_lines
#                 best_alpha = alpha
#                 best_beta = beta
#                 best_line = lines
                
#                 best_image = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR) 
#                 for line in best_line:
#                     x1, y1, x2, y2 = line[0]
#                     cv2.line(best_image, (x1, y1), (x2, y2), (255, 0, 0), 2) 
#                     cv2.imshow("line", best_image)
                    

# # best_image = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)  # bev 이미지를 컬러로 변환
# # for line in best_line:
# #     x1, y1, x2, y2 = line[0]
# #     cv2.line(best_image, (x1, y1), (x2, y2), (255, 0, 0), 2)  # 파란색 선을 그립니다.

# # 최적의 alpha와 beta 값을 출력
# print(f"Best alpha: {best_alpha}, Best beta: {best_beta}")
# print(f"Maximum number of lines detected: {max_lines}")

# # 최적의 결과 이미지 표시
# if best_image is not None:
#     # cv2.imshow('Best Detected Lines', best_image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
# else:
#     print("No lines detected with any combination of alpha and beta.")






# canny의 sigma값 바꿔보기 - 효과 미미
# import cv2
# import numpy as np
# import matplotlib.pyplot as plt


# def sigmoid(x):    
#     alpha = 0.5
#     beta = 150
#     img_float = x.astype(np.float32)
#     sigmoid_img = (255 / (1 + np.exp(-alpha * (img_float - beta)))).astype(np.uint8)
#     return sigmoid_img


# # Auto Canny 함수 정의
# def auto_canny(image, sigma=0.33):
#     v = np.median(image)
#     lower = int(max(0, (1.0 - sigma) * v))
#     upper = int(min(255, (1.0 + sigma) * v))
#     edged = cv2.Canny(image, lower, upper)
#     return edged

# # 이미지 파일 읽기
# image_path = 'a.png'  # 업로드한 이미지 파일 경로
# bev = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
# sig = sigmoid(bev)
# ret, thres2 = cv2.threshold(sig, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
# k = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
# dilate = cv2.dilate(thres2, k, iterations=2)
# # sigma 값을 바꿔가며 테스트 및 결과 이미지 저장
# sigma_values = np.arange(0.1, 1.1, 0.05)
# fig, axs = plt.subplots(4, 5, figsize=(20, 8))

# for i, sigma in enumerate(sigma_values):
#     # Canny 엣지 검출
#     canny_dilate = auto_canny(dilate, sigma=sigma)
    
#     # 결과 이미지 표시
#     row, col = divmod(i, 5)
#     axs[row, col].imshow(canny_dilate, cmap='gray')
#     axs[row, col].set_title(f'Sigma: {sigma}')
#     axs[row, col].axis('off')

# plt.suptitle('Canny Edge Detection with Different Sigma Values')
# plt.tight_layout()
# plt.show()



# import cv2
# import numpy as np

# def sigmoid(x, alpha, beta):
#     img_float = x.astype(np.float32)
#     sigmoid_img = (255 / (1 + np.exp(-alpha * (img_float - beta)))).astype(np.uint8)
#     return sigmoid_img

# def auto_canny(image, sigma=0.33):
#     v = np.median(image)
#     lower = int(max(0, (1.0 - sigma) * v))
#     upper = int(min(255, (1.0 + sigma) * v))
#     edged = cv2.Canny(image, lower, upper)
#     return edged

# image_path = 'a.png'
# bev = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# # 최적의 alpha, beta 초기화
# best_alpha = 0
# best_beta = 0
# max_valid_lines = 0
# best_image = None

# # 기울기 범위 설정
# min_slope = 20
# max_slope = 150
# min_size = 100


# Houghline의 파라미터 바꿔가면서 실험
# for beta in range(50, 151, 10):
#     for alpha in np.arange(0, 0.31, 0.01):

#         sig = sigmoid(bev, alpha, beta)
#         cv2.imshow("sig", sig)
#         ret, thres2 = cv2.threshold(sig, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
#         k = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
#         dilate = cv2.dilate(thres2, k, iterations=2)
#         canny_dilate = auto_canny(dilate, sigma=0.2)
#         num_labels, labels_im = cv2.connectedComponents(canny_dilate)


#         new_binary_img = np.zeros_like(canny_dilate)
#         for label in range(1, num_labels):  # 0은 배경이므로 제외
#             component = (labels_im == label).astype(np.uint8) * 255
#             if cv2.countNonZero(component) > min_size:
#                 new_binary_img = cv2.bitwise_or(new_binary_img, component)
#         cv2.imshow("new_binary_img", new_binary_img)
#         cv2.waitKey(10)
#         # lines = cv2.HoughLinesP(new_binary_img, 1, np.pi/180, threshold=65, minLineLength=
#         # 50, maxLineGap=50)
#         lines = cv2.HoughLinesP(new_binary_img, 1, np.pi/180, threshold=90, minLineLength=100, maxLineGap=50) # aa best(0.1, 120), 50,//
#         # lines = cv2.HoughLinesP(new_binary_img, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=5)

#         # 유효한 기울기를 가지는 line의 갯수 계산
#         if lines is not None:
#             valid_lines_count = 0
#             for line in lines:
#                 for x1, y1, x2, y2 in line:
#                     if x2 - x1 != 0:  # 수직선 피하기
#                         slope = (y2 - y1) / (x2 - x1)
#                         if min_slope <= abs(slope) <= max_slope:
#                             valid_lines_count += 1


#             # 유효한 line의 갯수가 최대인 경우 alpha, beta 갱신
#             if valid_lines_count > max_valid_lines:
#                 max_valid_lines = valid_lines_count
#                 best_alpha = alpha
#                 best_beta = beta
#                 best_image = best_image
#                 best_image = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR) 
#                 for line in lines:
#                     x1, y1, x2, y2 = line[0]
#                     cv2.line(best_image, (x1, y1), (x2, y2), (255, 0, 0), 2) 
#                     cv2.imshow("best_canny", new_binary_img)
#                     cv2.imshow("best_line", best_image)
#                     cv2.waitKey(1)

# print(f'Best alpha: {best_alpha}, Best beta: {best_beta}, Max valid lines: {max_valid_lines}')
# cv2.waitKey(0)



# beta 삭제 -> 차량 앞에 roi 생성해서 흰 차선 부분이 255로 세팅 하여 sigmoid 계산.
# import cv2
# import numpy as np

# def sigmoid(x, alpha, beta):
#     img_float = x.astype(np.float32)
#     sigmoid_img = (255 / (1 + np.exp(-alpha * (img_float - beta)))).astype(np.uint8)
#     return sigmoid_img

# def auto_canny(image, sigma=0.33):
#     v = np.median(image)
#     lower = int(max(0, (1.0 - sigma) * v))
#     upper = int(min(255, (1.0 + sigma) * v))
#     edged = cv2.Canny(image, lower, upper)
#     return edged

# image_path = 'a.png'
# bev = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# # (x, y) = (150, 495) 지점 기준 5x5 픽셀의 중간값 계산
# x, y = 150, 480
# roi = bev[y-2:y+3, x-40:x+40]
# median_val = np.max(roi)

# # 중간값을 0으로 지정하고 나머지 값들을 0 기준의 sigmoid로 변환
# bev_shifted = median_val - bev
# bev_sigmoid = sigmoid(bev_shifted, 0.1, 255)

# # 최적의 alpha, beta 초기화
# best_alpha = 0
# best_beta = 0
# max_valid_lines = 0
# best_image = None

# # 기울기 범위 설정
# min_slope = 20
# max_slope = 150
# min_size = 100

# # for beta in range(0, 201, 10):
# for alpha in np.arange(0, 0.51, 0.01):
#     sig = sigmoid(bev_shifted, alpha, 255)
#     ret, thres2 = cv2.threshold(sig, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
#     k = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
#     dilate = cv2.dilate(thres2, k, iterations=2)
#     canny_dilate = auto_canny(dilate, sigma=0.2)
#     num_labels, labels_im = cv2.connectedComponents(canny_dilate)

#     new_binary_img = np.zeros_like(canny_dilate)
#     for label in range(1, num_labels):  # 0은 배경이므로 제외
#         component = (labels_im == label).astype(np.uint8) * 255
#         if cv2.countNonZero(component) > min_size:
#             new_binary_img = cv2.bitwise_or(new_binary_img, component)
#     cv2.imshow("new_binary_img", new_binary_img)
#     cv2.waitKey(50)

#     lines = cv2.HoughLinesP(new_binary_img, 1, np.pi/180, threshold=60, minLineLength=100, maxLineGap=50)

#     # 유효한 기울기를 가지는 line의 갯수 계산
#     if lines is not None:
#         valid_lines_count = 0
#         for line in lines:
#             for x1, y1, x2, y2 in line:
#                 if x2 - x1 != 0:  # 수직선 피하기
#                     slope = (y2 - y1) / (x2 - x1)
#                     if min_slope <= abs(slope) <= max_slope:
#                         valid_lines_count += 1

#         # 유효한 line의 갯수가 최대인 경우 alpha, beta 갱신
#         if valid_lines_count > max_valid_lines:
#             max_valid_lines = valid_lines_count
#             best_alpha = alpha
#             # best_beta = beta
#             best_image = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 cv2.line(best_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
#                 cv2.imshow("sig", sig)
#                 cv2.imshow("best_canny", new_binary_img)
#                 cv2.imshow("best_line", best_image)
#                 cv2.waitKey(1)

# print(f'Best alpha: {best_alpha}, Best beta: best_beta, Max valid lines: {max_valid_lines}')
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# # dilate부터 먹이고 이미지 분석하기
# import cv2
# import numpy as np

# def sigmoid(x, alpha, beta):
#     img_float = x.astype(np.float32)
#     sigmoid_img = (255 / (1 + np.exp(-alpha * (img_float - beta)))).astype(np.uint8)
#     return sigmoid_img

# def auto_canny(image, sigma=0.33):
#     v = np.median(image)
#     lower = int(max(0, (1.0 - sigma) * v))
#     upper = int(min(255, (1.0 + sigma) * v))
#     edged = cv2.Canny(image, lower, upper)
#     return edged

# image_path = 'a.png'
# bev = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# x, y = 150, 480
# best_alpha = 0
# max_valid_lines = 0
# best_image = None
# min_slope = 20
# max_slope = 150
# min_size = 100

# k = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
# dilate = cv2.dilate(bev, k, iterations=2)
# cv2.imshow("dilate", dilate)
# max_val = np.max(dilate[y-2:y+3, x-40:x+40])
# bev_shifted = max_val - bev
# cv2.imshow("bev_shifted", bev_shifted)


# for alpha in np.arange(0, 0.36, 0.01):
#     sig = sigmoid(bev_shifted, alpha, 255)
#     cv2.imshow("sig", sig)
#     ret, thres2 = cv2.threshold(bev_shifted, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
#     canny_dilate = auto_canny(thres2, sigma=0.2)
#     num_labels, labels_im = cv2.connectedComponents(canny_dilate)


#     new_binary_img = np.zeros_like(canny_dilate)
#     for label in range(1, num_labels):  # 0은 배경이므로 제외
#         component = (labels_im == label).astype(np.uint8) * 255
#         if cv2.countNonZero(component) > min_size:
#             new_binary_img = cv2.bitwise_or(new_binary_img, component)
#     # cv2.imshow("new_binary_img", new_binary_img)
#     cv2.waitKey(100)

#     lines = cv2.HoughLinesP(new_binary_img, 1, np.pi/180, threshold=60, minLineLength=100, maxLineGap=50)

#     # 유효한 기울기를 가지는 line의 갯수 계산
#     if lines is not None:
#         valid_lines_count = 0
#         for line in lines:
#             for x1, y1, x2, y2 in line:
#                 if x2 - x1 != 0:  # 수직선 피하기
#                     slope = (y2 - y1) / (x2 - x1)
#                     if min_slope <= abs(slope) <= max_slope:
#                         valid_lines_count += 1

#         # 유효한 line의 갯수가 최대인 경우 alpha, beta 갱신
#         if valid_lines_count > max_valid_lines:
#             max_valid_lines = valid_lines_count
#             best_alpha = alpha
#             best_image = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 cv2.line(best_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
#                 cv2.imshow("best_sig", sig)
#                 # cv2.imshow("best_canny", new_binary_img)
#                 cv2.imshow("best_line", best_image)
#                 cv2.waitKey(1)

# print(f'Best alpha: {best_alpha}, Best beta: best_beta, Max valid lines: {max_valid_lines}')
# cv2.waitKey(0)
# cv2.destroyAllWindows()




# # alpha for문 삭제
# import cv2
# import numpy as np

# def sigmoid(x, alpha, beta):
#     img_float = x.astype(np.float32)
#     sigmoid_img = (255 / (1 + np.exp(-alpha * (img_float - beta)))).astype(np.uint8)
#     return sigmoid_img

# def auto_canny(image, sigma=0.33):
#     v = np.median(image)
#     lower = int(max(0, (1.0 - sigma) * v))
#     upper = int(min(255, (1.0 + sigma) * v))
#     edged = cv2.Canny(image, 0, 255)
#     return edged

# image_path = 'aa.png'
# bev = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# x, y = 150, 480
# best_image = None

# k = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
# dilate = cv2.dilate(bev, k, iterations=2)
# cv2.imshow("dilate", dilate)

# max_val = np.max(dilate[y-2:y+3, x-40:x+40])
# print(max_val)
# dilate_shifted = np.where(dilate == 0, 0, max_val - dilate)

# cv2.imshow("dilate_shifted", dilate_shifted)
# filtered = sigmoid(dilate_shifted, 0.02, 150)
# cv2.imshow("sigmoid", filtered)
# filtered = cv2.inRange(filtered, 100, 200)

# cv2.imshow("filtered", filtered)
# ret, thres2 = cv2.threshold(filtered, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)


# canny_dilate = cv2.Canny(thres2, 0, 255)
# cv2.imshow("canny_dilate", canny_dilate)

# num_labels, labels_im = cv2.connectedComponents(canny_dilate)
# new_binary_img = np.zeros_like(canny_dilate)
# for label in range(1, num_labels):  # 0은 배경이므로 제외
#     component = (labels_im == label).astype(np.uint8) * 255
#     if cv2.countNonZero(component) > 50:
#         new_binary_img = cv2.bitwise_or(new_binary_img, component)

# lines = cv2.HoughLinesP(new_binary_img, 1, np.pi/180, threshold=90, minLineLength=100, maxLineGap=50)


# best_image = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
# for line in lines:
#     x1, y1, x2, y2 = line[0]
#     cv2.line(best_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
#     cv2.imshow("best_line", best_image)
# cv2.waitKey(0)


# sigmoid 삭제, 가우시안 도입
import cv2
import numpy as np
from scipy.stats import norm

def gaussian_transform(x, mu, sigma):
    img_float = x.astype(np.float32)
    gaussian_pdf = norm.pdf(img_float, mu, sigma)
    gaussian_pdf = gaussian_pdf / np.max(gaussian_pdf)  # Normalize to range [0, 1]
    transformed_img = (gaussian_pdf * 255).astype(np.uint8)  # Scale to range [0, 255]
    return transformed_img

image_path = 'aa.png'
bev = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

x, y = 150, 480
best_image = None

k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
dilate = cv2.dilate(bev, k, iterations=2)
cv2.imshow("dilate", dilate)

max_val = np.min(dilate[y-2:y+3, x-40:x+40])
print(max_val)  # 이 값을 1초에 한번씩 긁어와서 저장한 다음 10짜리 리스트에 넣고 평균낸 값 사용하기?
dilate_shifted = np.where(dilate == 0, 0, max_val - dilate)
cv2.imshow("dilate_shifted", dilate_shifted)

# Gaussian 변환 적용
mu, sigma = 170, 50  # 평균과 표준 편차 값 설정
filtered = gaussian_transform(dilate_shifted, mu, sigma)
cv2.imshow("gaussian", filtered)

ret, thres2 = cv2.threshold(filtered, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
cv2.imshow("thres2", thres2)


canny_dilate = cv2.Canny(thres2, 0, 255)
cv2.imshow("canny_dilate", canny_dilate)

num_labels, labels_im = cv2.connectedComponents(canny_dilate)
new_binary_img = np.zeros_like(canny_dilate)
for label in range(1, num_labels):  # 0은 배경이므로 제외
    component = (labels_im == label).astype(np.uint8) * 255
    if cv2.countNonZero(component) > 50:
        new_binary_img = cv2.bitwise_or(new_binary_img, component)

lines = cv2.HoughLinesP(new_binary_img, 1, np.pi/180, threshold=70, minLineLength=100, maxLineGap=50)


best_image = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(best_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
    cv2.imshow("best_line", best_image)
cv2.waitKey(0)
