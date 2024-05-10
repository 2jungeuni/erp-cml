import cv2
import numpy as np

mtx = np.array([[337.5550842285156,                   0, 322.2391662597656],
                [                0,  337.5550842285156, 179.506591796875],
                [                0,                   0,                 1]])

# T_ac = np.array(
#     [[-3.56859197e-02, -9.99357500e-01, -3.33192829e-03,  1.31136520e+01],
#     [-1.43720280e-02,  3.84691018e-03, -9.99889317e-01,  6.99875206e+01],
#     [ 9.99259706e-01, -3.56340833e-02, -1.45000746e-02, -1.31155830e+02],
#     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
# )

T_ac =  np.linalg.inv(np.array([[ 0,  0,  1, 132.0],     
                                [-1,  0,  0, 6.0],
                                [ 0, -1,  0, 68.0],
                                [ 0,  0,  0, 1]]))


if __name__ == "__main__":
    img = cv2.imread('image_0005.png')
    img = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA)

    world_x_max = 400 # (cm)
    world_x_min = 300
    world_y_max = 0
    world_y_min = 0

    points = np.array([    
        # [world_x_min, world_y_min, 0, 1],
        # [world_x_max, world_y_min, 0, 1],
        # [world_x_min, world_y_max, 0, 1],
        # [world_x_max, world_y_max, 0, 1],
        # [400, 0, 0, 1],
        # [282, 35, 0, 1]
        [400, 0, 0, 1],
        [380, 0, 0, 1],
        [360, 0, 0, 1],
        [340, 0, 0, 1],
        [320, 0, 0, 1],
        [300, 0, 0, 1],
        [280, 0, 0, 1]
    ], dtype=np.float32)

    for i in range(points.shape[0]):
        cam_pts = T_ac @ points[i]
        img_pts_hom = mtx @ cam_pts[:3]
        x, y = img_pts_hom[:2] / img_pts_hom[2]
        print(int(x), int(y))
        
        cv2.circle(img, (int(x), int(y)), 4, (0, 0, 255), -1)
    cv2.imshow("output", img)
    cv2.waitKey(0)