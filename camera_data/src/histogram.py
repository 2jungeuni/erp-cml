import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

def gaussian_transform(x, mu, sigma):
    img_float = x.astype(np.float32)
    gaussian_pdf = norm.pdf(img_float, mu, sigma)
    gaussian_pdf = gaussian_pdf / np.max(gaussian_pdf)  # Normalize to range [0, 1]
    transformed_img = (gaussian_pdf * 255).astype(np.uint8)  # Scale to range [0, 255]
    return transformed_img
# 이미지 로드

image_path = 'a.png'
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

x, y = 150, 480
k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
dilate = cv2.dilate(image, k, iterations=2)
cv2.imshow("dilate", dilate)

max_val = np.min(dilate[y-2:y+3, x-40:x+40])
dilate_shifted = np.where(dilate == 0, 0, max_val - dilate)
cv2.imshow("dilate_shifted", dilate_shifted)
masked_image = np.where((dilate_shifted > 15) & (dilate_shifted < 245), dilate_shifted, 0)
cv2.imshow("masked_image", masked_image)


mu, sigma = 170, 50  # 평균과 표준 편차 값 설정
filtered = gaussian_transform(masked_image, mu, sigma)
cv2.imshow("gaussian", filtered)
# 히스토그램 계산 및 시각화
hist = cv2.calcHist([filtered], [0], None, [256], [0, 256])

plt.figure(figsize=(10, 5))
plt.title("Grayscale Histogram")
plt.xlabel("Bins")
plt.ylabel("# of Pixels")
plt.plot(hist)
plt.xlim([0, 256])
plt.show()

# 히스토그램을 기반으로 임계값 추정
threshold_value = 200  # 임계값은 히스토그램을 보고 수동으로 조정할 수 있음

# Threshold 적용
_, binary_image = cv2.threshold(filtered, threshold_value, 255, cv2.THRESH_BINARY)

# 결과 이미지 저장 및 시각화
cv2.imshow('binary_image.png', binary_image)
cv2.waitKey(0)