import rospy
import message_filters
from message_filters import ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Define constants
# T_wdc = np.array([
#     [ 0.0210377485,  -0.474024047,   0.880260539,  137.9578],
#     [-0.999772393,   -0.0130974325,  0.0168409938,   2.79708711],
#     [ 0.00354611692, -0.880414482,  -0.474191696,   67.3985744],
#     [ 0.0,            0.0,           0.0,            1.0]])  # 오늘 낮까지


T_wdc = np.array([
    [ 0.000990766529, 0.0451835649,  0.99897821,    135.793327],
    [-0.999967349,   -0.00796694816, 0.00135209085,   4.06815260],
    [ 0.00801989989, -0.998946932,   0.0451741962,   68.0230942],
    [ 0.0,            0.0,           0.0,             1.0]]) # 오늘 저녁 기준

class PosePublisher:
    def __init__(self):
        self.pos_pub = rospy.Publisher("/reference_pos", PointStamped, queue_size=10)
        self.rgb_info = None
        self.depth_info = None

        # Subscribe to camera info topics once to get the camera parameters
        self.rgb_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback, "rgb")

        # Subscribe to image topics with ApproximateTimeSynchronizer
        self.rgb_raw_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        self.depth_raw_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

        self.ts = ApproximateTimeSynchronizer([self.rgb_raw_sub, self.depth_raw_sub], 10, 0.1)
        self.ts.registerCallback(self.image_callback)


    def generate_direct_backward_mapping(self, 
        world_x_min, world_x_max, world_x_interval,
        world_y_min, world_y_max, world_y_interval, extrinsic, intrinsic):
        
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

        extrinsic = np.linalg.inv(extrinsic)
        camera_coords = (extrinsic @ world_coords.T).T  # [N, 4]
        uv_coords_hom = (intrinsic @ camera_coords[:, :3].T).T  # Drop the homogeneous component for intrinsic multiplication
        uv_coords = uv_coords_hom[:, :2] / uv_coords_hom[:, 2:]

        # Reshape back to the original grid shape
        map_x = uv_coords[:, 0].reshape(world_x_grid.shape)
        map_y = uv_coords[:, 1].reshape(world_y_grid.shape)
        
        return map_x, map_y


    def bilinear_sampler(self, imgs, pix_coords):
        """
        Construct a new image by bilinear sampling from the input image.
        Args:
            imgs:               [H,W,C]
            pix_coords:         [h,w,2]
            
            :return:
                sampled image   [h,w,c]
        """
        img_h, img_w, img_c = imgs.shape
        pix_h, pix_w, _ = pix_coords.shape
        out_shape = (pix_h, pix_w, img_c)
        
        pix_x, pix_y = np.split(pix_coords, [1], axis=-1)
        pix_x = pix_x.astype(np.float32)
        pix_y = pix_y.astype(np.float32)
        
        # Rounding
        pix_x0 = np.floor(pix_x).astype(np.int32)
        pix_x1 = pix_x0 + 1
        pix_y0 = np.floor(pix_y).astype(np.int32)
        pix_y1 = pix_y0 + 1
        
        # Clip within image boundary
        y_max = (img_h - 1)
        x_max = (img_w - 1)
        zero = np.zeros([1], dtype=np.int32)

        pix_x0 = np.clip(pix_x0, zero, x_max)
        pix_y0 = np.clip(pix_y0, zero, y_max)
        pix_x1 = np.clip(pix_x1, zero, x_max)
        pix_y1 = np.clip(pix_y1, zero, y_max)

        # Weights [pix_h, pix_w, 1]
        wt_x0 = (pix_x1 - pix_x).astype(np.float32)
        wt_x1 = (pix_x - pix_x0).astype(np.float32)
        wt_y0 = (pix_y1 - pix_y).astype(np.float32)
        wt_y1 = (pix_y - pix_y0).astype(np.float32)

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


    def remap_bilinear(self, image, map_x, map_y):
        pix_coords = np.concatenate([np.expand_dims(map_x, -1), np.expand_dims(map_y, -1)], axis=-1)
        bilinear_output = self.bilinear_sampler(image, pix_coords)
        output = np.round(bilinear_output).astype(np.uint8)
        return output


    def camera_info_callback(self, data, camera_type):
        if camera_type == "rgb" and not self.rgb_info:
            self.rgb_info = data
            self.rgb_info_sub.unregister()  # Unsubscribe after receiving the info


    def image_callback(self, rgb_image, depth_image):
        # Load RGB and Depth image
        bridge = CvBridge()
        try:
            cv_rgb = bridge.imgmsg_to_cv2(rgb_image, "bgr8")
            cv_depth = bridge.imgmsg_to_cv2(depth_image, "16UC1")
        except CvBridgeError as e:
            print(e)
            return

        rgb_intrinsic = np.array(self.rgb_info.K).reshape(3, 3)


        cv2.imshow("Original Image", cv_rgb)
        #--------------------------------
        world_x_max = 600 # (cm)
        world_x_min = 400
        world_y_max = 200
        world_y_min = -200

        world_x_interval = 1
        world_y_interval = 1
        

        map_x, map_y = self.generate_direct_backward_mapping(world_x_min, world_x_max, world_x_interval, world_y_min,
                                                              world_y_max, world_y_interval, T_wdc, rgb_intrinsic)
        output_image_bilinear = self.remap_bilinear(cv_rgb, map_x, map_y)

        cv2.imshow("BEV Image", output_image_bilinear)
        cv2.waitKey(1)
        
        #--------------------------------

if __name__ == "__main__":
    rospy.init_node("pose_publisher")
    node = PosePublisher()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()