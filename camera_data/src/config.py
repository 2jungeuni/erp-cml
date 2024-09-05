import numpy as np

section_list = [0, 199, 499]
q = 0
initial_not_found = True

# extrinsic = np.array([
#     [ 0.0,  0.0, 1.0, 37.72],
#     [-1.0,  0.0, 0.0,  23.13],
#     [ 0.0, -1.0, 0.0, 89.65],
#     [ 0.0,  0.0, 0.0,  1.0]
#     ])

# extrinsic = np.array([
#     [-1.18236371e-01, -4.82596696e-02,  9.91812061e-01,  3.77263669e+01],
#     [-9.92950917e-01,  1.40795514e-02, -1.17687053e-01,  3.41349257e+01],
#     [-8.28473055e-03, -9.98735586e-01, -4.95841982e-02,  8.96559759e+01],
#     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
# ])
# extrinsic=np.array([
#     [-1.18236371e-01, -4.82596696e-02,  9.91812061e-01,  3.77263669e+01],
#     [-9.92950917e-01,  1.40795514e-02, -1.17687053e-01,  2.31349257e+01],
#     [-8.28473055e-03, -9.98735586e-01, -4.95841982e-02,  8.96559759e+01],
#     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
# ]) # realsense on tripod

# extrinsic = np.array([
#     [ 2.68605057e-02, -3.17822178e-03,  9.99634139e-01,  4.03668163e+01],
#     [-9.99629855e-01,  4.23655817e-03,  2.68738603e-02,  4.24718506e+00],
#     [-4.32041927e-03, -9.99985975e-01, -3.06324929e-03,  4.74541858e+01],
#     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
# ])

extrinsic = np.array([
    [5.17024455e-02, -4.11113337e-01, 9.10116850e-01, 4.13406651e+01],
    [-9.98662517e-01, -2.11158341e-02, 4.71942623e-02, 2.94619446e+00],
    [-1.84314268e-04, -9.11339643e-01, -4.11655220e-01, 4.70406460e+01],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])



REF_POINT = np.array([80, 0, 0]).astype(np.float32)


# BEV
world_x_max = 200 # (cm)
world_x_min = 100
world_y_max = 100
world_y_min = -100
bev_x = 300
bev_y = 500
bev_pts = ((150, 420), (160, 420), (170, 420), (180, 420), (190, 420)) # for test
bev_pts2 = ((150, 200), (160, 200), (170, 200), (180, 200), (190, 200)) # for test


idx = 0
x_px_in_world = (world_x_max - world_x_min) / bev_x
y_px_in_world = (world_y_max - world_y_min) / bev_y

# Lane filtering
lane_width_world = 77
lane_thickness_world = 5*3 #! x3 because of dilate makes lane wider
lane_width_bev =  lane_width_world / ((world_y_max - world_y_min) / bev_x)
max_angle_init = 100
min_angle_init = 80
min_abs_distance_l = min_abs_distance_r = 25
min_angle_diff = 10
validate_lane_endpt_diff = int(lane_thickness_world / x_px_in_world)

# sliding window
window_width = validate_lane_endpt_diff * 3
window_height = 10
margin = 100 # (int): 윈도우 이동 시 허용할 좌우 범위
min_pix = 30 # (int): 윈도우 내에서 차선 점으로 인식할 최소 픽셀 수
straight_threshold = validate_lane_endpt_diff
curvature_threshold = 10.0  # 곡률 변화 허용치
max_curvature = 1.0  # 허용되는 최대 곡률 (1/m)