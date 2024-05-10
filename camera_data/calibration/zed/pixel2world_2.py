import numpy as np
import cv2

intrinsic = np.array([[337.5550842285156, 0, 322.2391662597656],
                      [0, 337.5550842285156, 179.506591796875],
                      [0, 0, 1]])

T_wc = np.array([[ 0,  0,  1, 132.0],
                [-1,  0,  0, 6.0],
                [ 0, -1,  0, 68.0],
                [ 0,  0,  0, 1]])

convert = np.array([[1, 0, 0],
                    [0, 0, 1],
                    [0, -1, 0]])

def point_to_world(x, y):
    Y_scale = 68                                                # height from ground to camera
    img_points = np.array([x, y, 1])                            # [x,-z, 1]
    image_coords = Y_scale * img_points                         # [x,-z, y] with scaling
    cam_coords_temp = np.linalg.inv(intrinsic) @ image_coords   # [X,-Z, Y]
    cam_coords = convert @ cam_coords_temp                      # [X, Y, Z]
    cam_coords_hom = np.append(cam_coords, 1)                   # [X, Y, Z, 1]
    world_coords = T_wc @ cam_coords_hom
    return world_coords[:3]



def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        world_coords = point_to_world(x, y)
        print("Mouse Position: x={}, y={}, World Coordinates: {}".format(x, y, world_coords))


if __name__ == '__main__':
    img = cv2.imread('image_0005.png')
    img = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA)
    
    print(2* np.arctan(640/(2*337.5550842285156))*(180/np.pi))
    print(2* np.arctan(360/(2*337.5550842285156))*(180/np.pi))


    cv2.namedWindow('image')
    cv2.setMouseCallback('image', mouse_callback)
        
    while True:
        cv2.imshow('image', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()



# import cv2
# import numpy as np

# intrinsic = np.array([[337.5550842285156, 0, 322.2391662597656],
#                       [0, 337.5550842285156, 179.506591796875],
#                       [0, 0, 1]])

# T_wc = np.linalg.inv(np.array([[ 0,  0,  1, 132.0],
#                  [-1,  0,  0, 6],
#                  [ 0, -1,  0, 68.0],
#                  [ 0,  0,  0, 1]]))


# if __name__ == '__main__':
#     # world_coords = np.array([[400, 0, 0, 1]])
#     # print(np.hstack([T_wc[:3, :3], T_wc[:3, 3].transpose]))
#     # print(T_wc[:3])
#     # image_coord = intrinsic @ (T_wc[:3, :3] @ world_coords + T_wc[:3, 3])
#     # image_coord = intrinsic @  world_coords
#     # print(image_coord)


#     # world_coords = np.array([[400, 0, 0, 1]])
#     # cam_pts = T_wc @ world_coords[0]
#     # print(cam_pts)
#     # img_pts_hom = intrinsic @ cam_pts[:3]
#     # print(img_pts_hom)
#     # x, y = img_pts_hom[:2] / img_pts_hom[2]
#     # print(int(x), int(y))


#     # 이미지 좌표
#     # image_coords = np.array([88385.42706299, 71061.5123291,268])
#     # image_coords = np.array([329, 265, 1])
#     # cam_coords = np.linalg.inv(intrinsic) @ image_coords
#     # print(-T_wc[2,3] / (T_wc[2,0:3] @ cam_coords))
#     # cam_coords_hom = np.append(cam_coords, 1)
#     # print(cam_coords_hom)
#     # world_coords = T_wc @ cam_coords_hom
#     # print(world_coords[:3])


#     Z_scale = 268
#     img_points = np.array([329, 265])
#     image_coords = Z_scale * img_points
#     image_coords_hom = np.append(image_coords, Z_scale)
#     cam_coords = np.linalg.inv(intrinsic) @ image_coords_hom
#     cam_coords_hom = np.append(cam_coords, 1)
#     world_coords = np.linalg.inv(T_wc) @ cam_coords_hom
#     print(world_coords[:3])

# def point_to_world(x, y):
#     Z_scale = 268
#     img_points = np.array([x, y])
#     image_coords = Z_scale * img_points
#     image_coords_hom = np.append(image_coords, Z_scale)
#     cam_coords = np.linalg.inv(intrinsic) @ image_coords_hom
#     cam_coords_hom = np.append(cam_coords, 1)
#     world_coords = np.linalg.inv(T_wc) @ cam_coords_hom
#     return world_coords[:3]
