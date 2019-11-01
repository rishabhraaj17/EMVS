import os
from collections import OrderedDict

import numpy as np
import yaml
from rospy.rostime import Time


def get_key_index_bounds(dictionary, time):
    ordered_dict = OrderedDict(sorted(dictionary.items(), key=lambda t: t[0]))
    previous, next_ = 0, 0
    if time < ordered_dict.keys()[0]:
        return previous, next_
    elif time > ordered_dict.keys()[-1]:
        previous = len(ordered_dict) - 1
        return previous, next_
    else:
        for idx, key in enumerate(ordered_dict):
            if key <= time:
                continue
            else:
                next_ = idx
                previous = idx - 1
                break
        return previous, next_


def to_seconds(time):
    return float(time.secs) + float(time.nsecs) / 1e9


def stamp_to_seconds(stamp):
    ros_time = Time(stamp.secs, stamp.nsecs)
    return ros_time.to_sec()


def time_difference(stamp1, stamp2):
    ros_time1 = Time(stamp1.secs, stamp1.nsecs)
    ros_time2 = Time(stamp2.secs, stamp2.nsecs)
    return ros_time1 - ros_time2


def time_difference_in_secs(stamp1, stamp2):
    return time_difference(stamp1, stamp2).to_sec()


def to_3x3_translation(t):
    return np.array([
        [0, 0, t[0]],
        [0, 0, t[1]],
        [0, 0, t[2]]
    ])


def check_eq(a, b):
    return True if a == b else False


def copy_to(img, other_image, mask):
    locs = np.where(mask != 0)  # Get the non-zero mask locations

    if len(img.shape) == 3 and len(other_image.shape) != 3:
        img[locs[0], locs[1]] = other_image[locs[0], locs[1], None]
        return img
    elif (len(img.shape) == 3 and len(other_image.shape) == 3) or \
            (len(img.shape) == 1 and len(other_image.shape) == 1):
        img[locs[0], locs[1]] = other_image[locs[0], locs[1]]
        return img
    else:
        raise Exception("Incompatible input and output dimensions")


def read_conf_file():
    filename = os.getcwd() + "/conf.yaml"
    with open(filename, 'r') as stream:
        configuration = yaml.load(stream)
        bag_filename = configuration['bag_filename']
        event_topic = configuration['event_topic']
        pose_topic = configuration['pose_topic']
        camera_info_topic = configuration['camera_info_topic']
        start_time = configuration['start_time']
        stop_time = configuration['stop_time']
        dim_x = configuration['dim_x']
        dim_y = configuration['dim_y']
        dim_z = configuration['dim_z']
        fov_deg = configuration['fov_deg']
        min_depth = configuration['min_depth']
        max_depth = configuration['max_depth']
        adaptive_thresholding_kernel_size = configuration['adaptive_thresholding_kernel_size']
        threshold_c = configuration['threshold_c']
        median_filter_size = configuration['median_filter_size']
        mean = configuration['mean']
        std_threshold = configuration['std_threshold']

    return bag_filename, event_topic, pose_topic, camera_info_topic, start_time, stop_time, dim_x, dim_y, dim_z, fov_deg, min_depth, max_depth, \
        adaptive_thresholding_kernel_size, threshold_c, median_filter_size, mean, std_threshold
