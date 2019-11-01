import cv2
import numpy as np
import pcl

from Utilities import Helpers as helpers
from Utilities.CameraPinhole import PinholeCamera
from Utilities.DepthVectors import InverseDepthVector
from Utilities.Grid3D import Grid3D


class ShapeDSI:

    def __init__(self, dim_x_, dim_y_, dim_z_, min_depth_, max_depth_, fov_):
        self.dim_x, self.dim_y, self.dim_z, self.min_depth, self.max_depth, self.fov = \
            dim_x_, dim_y_, dim_z_, min_depth_, max_depth_, fov_


class EMVS:

    def __init__(self, cam, dsi_shape):
        self.dvs_cam = cam
        self.dsi_shape = dsi_shape
        self.width = 0
        self.height = 0
        self.K = None
        self.dsi = None
        self.packet_size = 1024
        self.raw_depths_vec = None
        self.depths_vec = None
        self.virtual_cam = None
        self.precomputed_rectified_points = None

    def instantiate(self):
        full_resolution = self.dvs_cam.fullResolution()
        self.width = full_resolution[0]
        self.height = full_resolution[1]

        self.K = np.mat([[self.dvs_cam.fx(), 0.0, self.dvs_cam.cx()],
                         [0.0, self.dvs_cam.fy(), self.dvs_cam.cy()],
                         [0.0, 0.0, 1.0]])

        self.setup_dsi()
        self.precompute_rectified_points()

    def setup_dsi(self):
        self.depths_vec = InverseDepthVector(self.dsi_shape.min_depth,
                                             self.dsi_shape.max_depth,
                                             self.dsi_shape.dim_z)
        self.raw_depths_vec = self.depths_vec.get_depth_vector()

        self.dsi_shape.dim_x = self.dvs_cam.fullResolution()[0] if (self.dsi_shape.dim_x <= 0) else self.dsi_shape.dim_x
        self.dsi_shape.dim_y = self.dvs_cam.fullResolution()[1] if (self.dsi_shape.dim_y <= 0) else self.dsi_shape.dim_y

        if self.dsi_shape.fov < 10.0:
            f_virtual_cam = self.dvs_cam.fx()
        else:
            dsi_fov_rad = self.dsi_shape.fov * np.pi / 180.0
            f_virtual_cam = 0.5 * float(self.dsi_shape.dim_x) / np.tan(0.5 * dsi_fov_rad)

        self.virtual_cam = PinholeCamera(self.dsi_shape.dim_x,
                                         self.dsi_shape.dim_y,
                                         f_virtual_cam, f_virtual_cam,
                                         0.5 * float(self.dsi_shape.dim_x),
                                         0.5 * float(self.dsi_shape.dim_y))
        self.dsi = Grid3D(self.dsi_shape.dim_x, self.dsi_shape.dim_y, self.dsi_shape.dim_z)
        print "DSI setup done!!"

    def start(self, events, trajectory, world_to_ecam):
        self.instantiate()

        # Back-project events into the DSI
        check = self.event_back_projection(events, trajectory, world_to_ecam)

    def event_back_projection(self, events, trajectory, world_to_ecam):
        if len(events) < self.packet_size:
            print "number of events < packet size"
            return False

        # 2D coordinates of the events transferred to reference view using plane Z = z0.
        event_locations_z0_plane = []

        # list of camera centers
        camera_centers = []

        current_event = 0
        print "Evaluating dsi!"
        while current_event + self.packet_size < len(events):
            frame_ts = events[current_event + self.packet_size / 2].ts
            time_event_cam, event_cam_to_world, check = trajectory.get_pose_at(frame_ts)
            if not check:
                current_event += 1
                continue

            event_cam_to_vcam = world_to_ecam * event_cam_to_world

            vcam_to_event_cam = event_cam_to_vcam.inverse()

            R = vcam_to_event_cam.quat_to_3x3_rotation_matrix()
            t = vcam_to_event_cam.position

            # Store optical center of dvs camera in the frame of reference of virtual camera
            camera_centers.append(np.array(np.matmul(-R.T, t), dtype=np.float32))

            # Projecting points on z0
            plane_z0 = self.raw_depths_vec[0]

            # Eq. (8), Apply planar homography, map points into the virtual camera viewpoint
            H_z0_inv = np.asmatrix(R, dtype=np.float)

            H_z0_inv = plane_z0 * H_z0_inv

            H_z0_inv = H_z0_inv + helpers.to_3x3_translation(t)

            # Bring homography in pixel coordinates
            H_z0_inv_px = self.K * H_z0_inv * self.virtual_cam.k_inv

            H_z0_px = np.linalg.inv(H_z0_inv_px)

            # pre-computing the warped event locations according to Eq. (11)
            for i in range(self.packet_size):
                e = events[current_event]
                current_event += 1
                p = np.empty((3, 1), dtype=np.float)

                p[0], p[1] = self.precomputed_rectified_points[:, (e.y * self.width + e.x)]
                p[2] = 1.0

                p = np.matmul(H_z0_px, p)
                p /= p[2]
                event_locations_z0_plane.append(p)

        print "DSI evaluation done!"

        self.dsi.reset_grid()

        print "DSI reset!"

        self.fill_voxel_grid(event_locations_z0_plane, camera_centers)

        return True

    def fill_voxel_grid(self, event_locations_z0, camera_centers):
        # This function implements Step 2 of paper's algorithm.
        # It maps events from plane Z0 to all the planes Zi of the DSI using Eq. (15)
        # and then votes for the corresponding voxel.

        N = 256
        X = np.zeros((N, 1), dtype=np.float32)
        Y = np.zeros((N, 1), dtype=np.float32)

        z0 = self.raw_depths_vec[0]

        for depth_plane in range(len(self.raw_depths_vec)):
            batch_step = 0

            for packet in range(len(camera_centers)):

                # Eq. 15
                v_camera = camera_centers[packet]
                zi = float(self.raw_depths_vec[depth_plane])
                a = z0 * (zi - v_camera[2])
                bx = (z0 - zi) * (v_camera[0] * self.virtual_cam.fx + v_camera[2] * self.virtual_cam.cx)
                by = (z0 - zi) * (v_camera[1] * self.virtual_cam.fy + v_camera[2] * self.virtual_cam.cy)
                d = zi * (z0 - v_camera[2])

                for batch in range(self.packet_size / N):
                    for i in range(N):
                        X[i] = event_locations_z0[batch_step + i][0]
                        Y[i] = event_locations_z0[batch_step + i][1]

                    X = (X * a + bx) / d
                    Y = (Y * a + by) / d
                    for i in range(N):
                        self.dsi.vote_grid_value_at(X[i], Y[i], depth_plane)
                    if batch_step < len(event_locations_z0):
                        batch_step += N

        print "Voxel grid filled!!"

    def get_depth_map_from_dsi(self, adaptive_threshold_kernel_size, adaptive_threshold_c, median_filter_size):
        # maximum number of votes along optical ray
        confidence_map, depth_cell_indices = self.dsi.collect_local_maxima()

        # adapting thresholding on confidence map
        confidence_map_8bit = None
        confidence_map_8bit = cv2.normalize(confidence_map, confidence_map_8bit, 0.0, 255.0, cv2.NORM_MINMAX)
        confidence_map_8bit = np.uint8(confidence_map_8bit)

        mask = None
        mask = cv2.adaptiveThreshold(confidence_map_8bit, 1,
                                     cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY,
                                     adaptive_threshold_kernel_size,
                                     adaptive_threshold_c)  # 127

        # clean up depth map using median filter
        depth_cell_indices_filtered = cv2.medianBlur(depth_cell_indices, median_filter_size)

        # remove the outer border to suppress boundary effects
        border_size = max(adaptive_threshold_kernel_size / 2, 1)
        mask = self.remove_mask_boundary(mask, border_size)

        # Convert depth indices to depth values
        depth_map = self.convert_depth_indices_to_values(depth_cell_indices_filtered)

        return depth_map, confidence_map, mask

    def get_point_cloud(self, depth_map, mask, mean, std_threshold):
        pc = []

        for y in range(depth_map.shape[0]):
            for x in range(depth_map.shape[1]):
                if mask[y][x] > 0:
                    keypoint = np.array([x, y], dtype=np.float)
                    ray = self.virtual_cam.project_pixel_to_3d_ray(keypoint)
                    ray_vector_norm = np.linalg.norm(ray)
                    ray = ray / ray_vector_norm
                    xyz_vcam = (ray / ray[2] * np.float32(depth_map[y][x]))

                    p_vcam_tuple = (xyz_vcam[0], xyz_vcam[1], xyz_vcam[2], (1.0 / xyz_vcam[2]))
                    pc.append(p_vcam_tuple)

        p_vcam = pcl.PointCloud_PointXYZI(pc)
        cloud_filtered = pcl.PointCloud_PointXYZI()

        # Statistical Outlier Removal
        outlier_rm = pcl.StatisticalOutlierRemovalFilter_PointXYZI(p_vcam)
        outlier_rm.set_mean_k(mean)
        outlier_rm.set_std_dev_mul_thresh(std_threshold)
        cloud_filtered = outlier_rm.filter()

        pc = cloud_filtered

        return pc

    def precompute_rectified_points(self):
        # Create a lookup table that maps pixel coordinates to undistorted pixel coordinates
        self.precomputed_rectified_points = np.empty((2, self.height * self.width), dtype=np.float32)
        for y in range(0, self.height):
            for x in range(0, self.width):
                rectified_point = self.dvs_cam.rectifyPoint((x, y))
                point = np.array([[rectified_point[0]], [rectified_point[1]]],
                                 dtype=np.float32).flatten()
                self.precomputed_rectified_points[:, y * self.width + x] = point

    def convert_depth_indices_to_values(self, depth_cell_indices):
        # Convert depth indices to depth values, for all pixels
        depth_map = np.zeros((np.shape(depth_cell_indices)[0], np.shape(depth_cell_indices)[1]), dtype=np.float32)
        for y in range(0, np.shape(depth_cell_indices)[0]):
            for x in range(0, np.shape(depth_cell_indices)[1]):
                val = self.depths_vec.cell_index_to_depth(np.uint(depth_cell_indices[y][x]))
                depth_map[y][x] = val
        return depth_map

    def remove_mask_boundary(self, mask, border_size):
        for y in range(0, np.shape(mask)[0]):
            for x in range(0, np.shape(mask)[1]):
                if (x <= border_size or x >= np.shape(mask)[1] - border_size or
                        y <= border_size or y >= np.shape(mask)[0] - border_size):
                    mask[y, x] = 0
        return mask
