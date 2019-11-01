from collections import OrderedDict

import Helpers as helpers
from Utilities.PoseTransforms import PoseTransformer


class Trajectory(object):

    def __init__(self, poses):
        self.poses = poses

    def get_pose_at(self, time):
        return self.get_pose_at(time)

    def get_first_pose(self):
        ordered_poses = OrderedDict(sorted(self.poses.items(), key=lambda t: t[0]))
        time = ordered_poses.keys()[0]
        pose = ordered_poses[time]
        return time, pose

    def get_last_pose(self):
        ordered_poses = OrderedDict(sorted(self.poses.items(), key=lambda t: t[0]))
        time = ordered_poses.keys()[-1]
        pose = ordered_poses[time]
        return time, pose

    def get_num_of_poses(self):
        return self.poses.size


class LinearTrajectory(Trajectory):

    def __init__(self, poses):
        super(LinearTrajectory, self).__init__(poses)
        if len(poses) <= 2:
            raise ValueError("At least two poses needed to proceed!")

    def get_pose_at(self, time):

        self.poses = OrderedDict(sorted(self.poses.items(), key=lambda t: t[0]))

        if self.poses.keys()[0] == time:
            print "There's no pose before the very first pose."
            return False
        elif self.poses.keys()[-1] == time:
            print "There's no pose after the last recorded pose."
            return False
        else:
            previous_pose_idx, next_pose_idx = helpers.get_key_index_bounds(self.poses, time)

            t0 = self.poses.keys()[previous_pose_idx]
            pose0 = self.poses[t0]
            t1 = self.poses.keys()[next_pose_idx]
            pose1 = self.poses[t1]

        pose = self.__estimate_pose(pose0, pose1, t0, t1, time)
        return time, pose, True

    def __estimate_pose(self, pose0, pose1, t0, t1, time):
        pose0_inv = pose0.inverse()
        pose_rel = pose0_inv * pose1
        del_t = helpers.to_seconds(time - t0) / helpers.to_seconds(t1 - t0)
        pose_temp = PoseTransformer.exp(pose_rel, del_t)
        estimated_pose = pose0 * pose_temp
        pose = estimated_pose
        return pose
