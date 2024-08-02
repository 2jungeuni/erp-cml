import numpy as np

world_x_max = 250   # unit: cm
world_x_min = 170   # unit: cm
world_y_max = 70    # unit: cm
world_y_min = -70   # unit: cm

q = 0
section_list = [0, 199, 499]                    
initial_not_found = True

intrinsic = None

extrinsic = np.array([
    [ 4.14811235e-02, -1.67221987e-02,  9.98999342e-01,  4.58149165e+01],
    [-9.99030049e-01, -1.54790444e-02,  4.12232956e-02,  4.56362457e+00],
    [ 1.47742110e-02, -9.99740350e-01 ,-1.73480671e-02,  4.19248218e+01],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
])  # TODO: Question


T_wc = np.array(
    [[-3.56859197e-02, -9.99357500e-01, -3.33192829e-03,  1.31136520e+01],
    [-1.43720280e-02,  3.84691018e-03, -9.99889317e-01,  6.99875206e+01],
    [ 9.99259706e-01, -3.56340833e-02, -1.45000746e-02, -1.31155830e+02],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
)

# world_x_max = 200 # (cm)
# world_x_min = 100
# world_y_max = 250
# world_y_min = -250

REF_POINT = np.array([100, 0, 0]).astype(np.float32)