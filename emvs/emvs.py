import time

import cv2
import numpy as np
import pcl
from image_geometry import PinholeCameraModel
from rospy.rostime import Time

import Utilities.EMVS as EMVS
import Utilities.InputData
import Utilities.Trajectory as tu
from Utilities import Helpers as helpers

bag_filename, event_topic, pose_topic, camera_info_topic, start_time, stop_time, dim_x, dim_y, dim_z, fov_deg, min_depth, max_depth, \
adaptive_thresholding_kernel_size, threshold_c, median_filter_size, mean, std_threshold = helpers.read_conf_file()


def main():
    print "Parsing Bagfile... "
    # Load events, poses, and camera intrinsics from the rosbag
    events, timestamped_poses, camera_info_msg = Utilities.InputData.parse_rosbag(bag_filename, event_topic, camera_info_topic,
                                                                      pose_topic, start_time, stop_time)

    print "Done Parsing!"
    # Create a camera object from the loaded intrinsic parameters
    cam = PinholeCameraModel()
    cam.fromCameraInfo(camera_info_msg)

    # Use linear interpolation to compute the camera pose for each event
    trajectory = tu.LinearTrajectory(timestamped_poses)

    # Set the position of the reference view in the middle of the trajectory
    t0, pose0 = trajectory.get_first_pose()

    t1, pose1 = trajectory.get_last_pose()

    _, ecam_to_world, _ = trajectory.get_pose_at(Time.from_sec((0.5 * (t0.to_sec() + t1.to_sec()))))

    world_to_ecam = ecam_to_world.inverse()

    # Initialize the DSI
    if dim_z > 256:
        print "Please keep the number of depth planes <= 256"
    dsi_shape = Utilities.EMVS.ShapeDSI(cam.width, cam.height, dim_z, min_depth, max_depth, fov_deg)

    t_start = time.time()

    # Start of evaluation
    evaluator = EMVS.EMVS(cam, dsi_shape)
    evaluator.start(events=events, trajectory=trajectory, world_to_ecam=world_to_ecam)

    t_end = time.time()
    print "Time elapsed : ", (t_end - t_start)

    # Extract semi-dense depth map from DSI
    depth_map, confidence_map, semidense_mask = evaluator.get_depth_map_from_dsi(adaptive_thresholding_kernel_size,
                                                                              threshold_c, median_filter_size)
    post_evaluation_results(confidence_map, depth_map, dsi_shape, evaluator, semidense_mask)

    t_final = time.time()
    print "Time elapsed : ", (t_final - t_start)


def post_evaluation_results(confidence_map, depth_map, dsi_shape, evaluator, semidense_mask):
    cv2.imwrite("Outputs/semidense_mask.png", 255 * semidense_mask)
    confidence_map_255 = None
    confidence_map_255 = cv2.normalize(confidence_map, confidence_map_255, 0.0, 255.0, cv2.NORM_MINMAX, dtype=cv2.CV_32FC1)
    cv2.imwrite("Outputs/confidence_map.png", confidence_map_255)
    depth_map_255 = (depth_map - dsi_shape.min_depth) * (255.0 / (dsi_shape.max_depth - dsi_shape.min_depth))
    depth_map_8bit = np.uint8(depth_map_255)
    depthmap_color = None
    depthmap_color = cv2.applyColorMap(src=depth_map_8bit, colormap=cv2.COLORMAP_RAINBOW, dst=depthmap_color)
    depth_on_canvas = np.zeros((depth_map.shape[0], depth_map.shape[1], 3), dtype=np.uint8)
    depth_on_canvas[:] = (255, 255, 255)
    depth_on_canvas = helpers.copy_to(depth_on_canvas, depthmap_color, semidense_mask)
    cv2.imwrite("Outputs/depth_colored.png", depth_on_canvas)

    pc = evaluator.get_point_cloud(depth_map, semidense_mask, mean, std_threshold)

    pcl.save(pc, "Outputs/pointcloud.pcd", "pcd")
    print "Saved : ", pc.size, " points to Outputs/pointcloud.pcd"


if __name__ == '__main__':
    main()
