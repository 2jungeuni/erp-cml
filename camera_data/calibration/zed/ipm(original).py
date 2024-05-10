import cv2
import numpy as np

mtx = np.array([[337.5550842285156,                   0, 322.2391662597656],
                [                0,  337.5550842285156, 179.506591796875],
                [                0,                   0,                 1]])

T_ac = np.array(
    [[-3.56859197e-02, -9.99357500e-01, -3.33192829e-03,  1.31136520e+01],
    [-1.43720280e-02,  3.84691018e-03, -9.99889317e-01,  6.99875206e+01],
    [ 9.99259706e-01, -3.56340833e-02, -1.45000746e-02, -1.31155830e+02],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
)
# T_ac =  np.linalg.inv(np.array([[ 0,  0,  1, 132.0],     
#                                 [-1,  0,  0, 6.0],
#                                 [ 0, -1,  0, 68.0],
#                                 [ 0,  0,  0, 1]]))




#*===============================BEV 이미지와 Image 좌표간의 LUT (Look Up Table) 구하기===============================
def generate_direct_backward_mapping(
    world_x_min, world_x_max, world_x_interval,
    world_y_min, world_y_max, world_y_interval, extrinsic, intrinsic, img):
    
    # Calculate the world coordinates grid
    world_x_coords = np.arange(world_x_max, world_x_min, -world_x_interval)
    world_y_coords = np.arange(world_y_max, world_y_min, -world_y_interval)
    world_x_grid, world_y_grid = np.meshgrid(world_x_coords, world_y_coords, indexing='ij')

    # Flatten the grid for vectorized operations
    world_x_flat = world_x_grid.flatten()
    world_y_flat = world_y_grid.flatten()
    world_z_flat = np.zeros_like(world_x_flat)  # Z-coordinates are 0
    ones_flat = np.ones_like(world_x_flat)

    # Stack to get homogeneous coordinates [N, 4]
    world_coords = np.stack([world_x_flat, world_y_flat, world_z_flat, ones_flat], axis=-1)

    camera_coords = (extrinsic @ world_coords.T).T  # [N, 4]
    uv_coords_hom = (intrinsic @ camera_coords[:, :3].T).T  # Drop the homogeneous component for intrinsic multiplication
    uv_coords = uv_coords_hom[:, :2] / uv_coords_hom[:, 2:]
    
    
    points = np.array([    
        [world_x_min, world_y_min, 0, 1],
        [world_x_max, world_y_min, 0, 1],
        [world_x_min, world_y_max, 0, 1],
        [world_x_max, world_y_max, 0, 1],
    ], dtype=np.float32)
    img2 = img.copy()
    for i in range(points.shape[0]):
        cam_pts = extrinsic @ points[i]
        img_pts_hom = mtx @ cam_pts[:3]
        x, y = img_pts_hom[:2] / img_pts_hom[2]
        print(int(x), int(y))
        
        cv2.circle(img2, (int(x), int(y)), 4, (0, 0, 255), -1)
    cv2.imshow("a", img2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
        
        
    # Reshape back to the original grid shape
    map_x = uv_coords[:, 0].reshape(world_x_grid.shape)
    map_y = uv_coords[:, 1].reshape(world_y_grid.shape)
        
    return map_x, map_y

#*===============================backward 방식으로 IPM 처리하여 BEV 이미지 생성하기(bilinear interpolation)===============================
def bilinear_sampler(imgs, pix_coords):
    """
    Construct a new image by bilinear sampling from the input image.
    Args:
        imgs:               [H,W,C]
        pix_coords:         [h,w,2]
        
        :return:
            sampled image   [h,w,c]
    """
    img_h, img_w, img_c = imgs.shape
    pix_h, pix_w, pix_c = pix_coords.shape
    out_shape = (pix_h, pix_w, img_c)
    
    pix_x, pix_y = np.split(pix_coords, [1], axis=-1)
    pix_x = pix_x.astype(np.float32)
    pix_y = pix_y.astype(np.float32)
    
    # Rounding
    pix_x0 = np.floor(pix_x)
    pix_x1 = pix_x0 + 1
    pix_y0 = np.floor(pix_y)
    pix_y1 = pix_y0 + 1
    
    # Clip within image boundary
    y_max = (img_h - 1)
    x_max = (img_w - 1)
    zero = np.zeros([1])

    pix_x0 = np.clip(pix_x0, zero, x_max)
    pix_y0 = np.clip(pix_y0, zero, y_max)
    pix_x1 = np.clip(pix_x1, zero, x_max)
    pix_y1 = np.clip(pix_y1, zero, y_max)

    # Weights [pix_h, pix_w, 1]
    wt_x0 = pix_x1 - pix_x
    wt_x1 = pix_x - pix_x0
    wt_y0 = pix_y1 - pix_y
    wt_y1 = pix_y - pix_y0

    # indices in the image to sample from
    dim = img_w

    # Apply the lower and upper bound pix coord
    base_y0 = pix_y0 * dim
    base_y1 = pix_y1 * dim

    # 4 corner vertices
    idx00 = (pix_x0 + base_y0).flatten().astype(np.int32)
    idx01 = (pix_x0 + base_y1).astype(np.int32)
    idx10 = (pix_x1 + base_y0).astype(np.int32)
    idx11 = (pix_x1 + base_y1).astype(np.int32)

    # Gather pixels from image using vertices
    imgs_flat = imgs.reshape([-1, img_c]).astype(np.float32)
    im00 = imgs_flat[idx00].reshape(out_shape)
    im01 = imgs_flat[idx01].reshape(out_shape)
    im10 = imgs_flat[idx10].reshape(out_shape)
    im11 = imgs_flat[idx11].reshape(out_shape)

    # Apply weights [pix_h, pix_w, 1]
    w00 = wt_x0 * wt_y0
    w01 = wt_x0 * wt_y1
    w10 = wt_x1 * wt_y0
    w11 = wt_x1 * wt_y1
    output = w00 * im00 + w01 * im01 + w10 * im10 + w11 * im11
    return output

def remap_bilinear(image, map_x, map_y):
    pix_coords = np.concatenate([np.expand_dims(map_x, -1), np.expand_dims(map_y, -1)], axis=-1)
    bilinear_output = bilinear_sampler(image, pix_coords)
    output = np.round(bilinear_output).astype(np.uint8)
    return output    


def world_to_img_pts(img):
    # pts = np.array([
    #     [300, -70, 0, 1],
    #     [300, 70, 0, 1],
    #     [600, -70, 0, 1],
    #     [600, 70, 0, 1],
    # ], dtype=np.float32)
    
    # for i in range(pts.shape[0]):
    #     cam_pts = T_wc @ pts[i]
    #     img_pts_hom = mtx @ cam_pts[:3]
    #     x,y = img_pts_hom[:2] / img_pts_hom[2]
    #     cv2.circle(img, (int(x), int(y)), 4, (0,0,255), -1)
    # cv2.imshow("world pts on img", img)
    # cv2.waitKey(2000)
    # # cv2.destroyAllWindows()

    world_x_max = 900 # (cm)
    world_x_min = 500
    world_y_max = 200
    world_y_min = -200
    world_x_interval = 0.5
    world_y_interval = 1.0

    # Calculate the number of rows and columns in the output image
    output_width = int(np.ceil((world_y_max - world_y_min) / world_y_interval))
    output_height = int(np.ceil((world_x_max - world_x_min) / world_x_interval))
    # print("BEV img (width, height) :", "(", output_width, ",",  output_height, ")")

    #* For projecting world points onto image to use it in opencv bev conversion
    points = np.array([    
        [world_x_min, world_y_min, 0, 1],
        [world_x_max, world_y_min, 0, 1],
        [world_x_min, world_y_max, 0, 1],
        [world_x_max, world_y_max, 0, 1],
    ], dtype=np.float32)
    

if __name__ == "__main__":
    img = cv2.imread('image_0005.png')
    img = cv2.resize(img, (640,360), interpolation=cv2.INTER_AREA)
    # world_to_img_pts(img.copy())

    world_x_max = 400 # (cm)
    world_x_min = 300
    world_y_max = 70
    world_y_min = -70

    world_x_interval = 0.5
    world_y_interval = 0.5
    
    map_x, map_y = generate_direct_backward_mapping(world_x_min, world_x_max, world_x_interval, world_y_min, world_y_max, world_y_interval, T_ac, mtx, img)
    output_image_bilinear = remap_bilinear(img, map_x, map_y)
    
    cv2.imshow("OUTPUT", output_image_bilinear)
    cv2.waitKey(0)