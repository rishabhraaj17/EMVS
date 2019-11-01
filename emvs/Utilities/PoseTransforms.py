import math
import sys

import numpy as np
import pyquaternion.quaternion as Quat
from geometry_msgs.msg import Point, Quaternion

epsilon = sys.float_info.epsilon


class PoseTransformer:

    def __init__(self, position, rotation_quaternion):
        self.position = position
        self.rotation = rotation_quaternion

    @staticmethod
    def build_from_pose_stamped(pose_stamped):
        position = Point()
        position.x = pose_stamped.pose.position.x
        position.y = pose_stamped.pose.position.y
        position.z = pose_stamped.pose.position.z

        position_vector = np.array([position.x, position.y, position.y])

        rotation_quat = Quaternion()
        rotation_quat.w = pose_stamped.pose.orientation.w
        rotation_quat.x = pose_stamped.pose.orientation.x
        rotation_quat.y = pose_stamped.pose.orientation.y
        rotation_quat.z = pose_stamped.pose.orientation.z

        quat_vector = np.array([rotation_quat.w, rotation_quat.x, rotation_quat.y, rotation_quat.z])

        return PoseTransformer(position_vector, quat_vector)

    def build_from_pose(self, pos, rotation):
        position = Point()
        position.x = pos.pose.position.x
        position.y = pos.pose.position.y
        position.z = pos.pose.position.z

        position_vector = np.array([position.x, position.y, position.y])

        rotation_quat = Quaternion()
        rotation_quat.w = rotation.pose.orientation.w
        rotation_quat.x = rotation.pose.orientation.x
        rotation_quat.y = rotation.pose.orientation.y
        rotation_quat.z = rotation.pose.orientation.z

        quat_vector = np.array([rotation_quat.w, rotation_quat.x, rotation_quat.y, rotation_quat.z])

        self.position = position_vector
        self.rotation = quat_vector

    def inverse(self):
        rot_mat_self = Quat.Quaternion(self.rotation)

        quat_conjugate = rot_mat_self.conjugate
        quat_inverse = rot_mat_self.inverse
        vector_rotated = -1 * (quat_inverse.rotate(self.position))
        return PoseTransformer(vector_rotated, quat_conjugate.elements)

    def __mul__(self, other):
        rot_mat_self = Quat.Quaternion(self.rotation)
        rot_mat_other = Quat.Quaternion(other.rotation)

        compose_quat = rot_mat_self * rot_mat_other
        compose_pos = self.position + rot_mat_self.rotate(other.position)
        return PoseTransformer(compose_pos, compose_quat.elements)

    def append_t_and_homogeneous(self, x):
        '''
        t here is the position i.e. translation
        :param x: 3x3 matrix, most probably a rotation matrix
        :return: 4x4 homogeneous matrix composed of [x t]
        '''
        t = self.position
        x_t = np.column_stack((x, t))
        m = np.vstack((x_t,
                       [0.0, 0.0, 0.0, 1.0]))
        return m

    def convert_back(self, x):
        '''
        :param x: 3x3 or 4x4 homogeneous matrix composed of [R t]
        :return: quaternion and/or position
        '''
        t = None
        if np.shape(x) == (4, 4):
            t = x[:3, [3]]
            x_ = np.delete(x, 3, 0)
            x = np.delete(x_, 3, 1)
        q_ = Quat.Quaternion._from_matrix(x)
        q = np.zeros((4,), dtype=float)
        q[0] = q_[0]
        q[1] = q_[1]
        q[2] = q_[2]
        q[3] = q_[3]
        if t is not None:
            return PoseTransformer(t, q)
        else:
            return PoseTransformer([0, 0, 0], q)

    @staticmethod
    def convert_back_np(x):
        '''
        :param x: 3x3 or 4x4 homogeneous matrix composed of [R t]
        :return: quaternion and/or position
        '''
        t = None
        if np.shape(x) == (4, 4):
            t = x[:3, [3]]
            x_ = np.delete(x, 3, 0)
            x = np.delete(x_, 3, 1)
        q_ = Quat.Quaternion._from_matrix(x)
        q = np.zeros((4,), dtype=float)
        q[0] = q_[0]
        q[1] = q_[1]
        q[2] = q_[2]
        q[3] = q_[3]
        if t is not None:
            return PoseTransformer(np.asarray(t).T.flatten(), np.asarray(q))
        else:
            return PoseTransformer(np.array([0, 0, 0]), np.asarray(q))

    def quat_to_3x3(self, q):
        '''

        :param q: quaternion
        :return: 3x3 rotation matrix
        '''
        quat_r = Quat.Quaternion(q)
        r = quat_r.rotation_matrix
        return r

    def quat_to_3x3_rotation_matrix(self):
        '''

        :param q: quaternion
        :return: 3x3 rotation matrix
        '''
        quat_r = Quat.Quaternion(self.rotation)
        r = quat_r.rotation_matrix
        return r

    def quat_to_3x3_inv(self, q):
        '''

        :param q: quaternion
        :return: 3x3 rotation matrix
        '''
        q_ = np.linalg.inv(self.quat_to_3x3(q))
        return q_

    def compose(self, t1, t2):
        '''

        :param t1: matrix
        :param t2: matrix
        :return: product
        '''

        assert np.shape(t1) == np.shape(t2), "Shape of the 2 matrices do not match"
        t = np.matmul(t1, t2)
        return t

    def make_extrinsic_matrix(self):
        r = self.quat_to_3x3(self.rotation)
        m = self.append_t_and_homogeneous(r)
        return m

    def __str__(self):
        m = self.make_extrinsic_matrix()
        return str(m)

    def log(self):
        l = Quat.Quaternion(self.rotation)
        log = Quat.Quaternion.log(l)
        print log
        l_ = log.rotation_matrix
        l = self.append_t_and_homogeneous(l_)
        return l

    def __log(self):
        '''
        :return: 3 x 1 vector
        '''
        a = self.rotation[1:]
        na = np.linalg.norm(self.rotation)
        eta = self.rotation[0]

        scale = 0

        if math.fabs(eta) < na:
            if eta >= 0:
                scale = np.arccos(eta) / na
            else:
                scale = -np.arccos(-eta) / na
        else:
            if eta >= 0:
                scale = arc_sinx_overx(na)
            else:
                scale = -arc_sinx_overx(na)

        return a * (float(2) * scale)

    def vector6_from_transformation(self):
        return PoseTransformer(self.position, self.__log())

    def __get_log(self):
        vec6 = self.vector6_from_transformation()
        exp = vec6.vec6_to_transformation()
        return exp

    @staticmethod
    def vec3quat_to_quat(dx):
        theta = np.linalg.norm(dx)

        na = 0.0

        if is_lessthen_epsilons4th_root(theta):
            one_over_48 = float(1.0 / 48.0)
            na = 0.5 + (theta * theta) * one_over_48
        else:
            na = np.sin(theta * 0.5) / theta

        ct = np.cos(theta * 0.5)

        return np.array([ct, dx[0] * na, dx[1] * na, dx[2] * na])

    def vec6_to_transformation(self):
        position = self.position
        rotation = PoseTransformer.vec3quat_to_quat(self.rotation)

        return PoseTransformer(position.flatten(), rotation)

    @staticmethod
    def exp(transformation, factor=1.0):
        vec6 = transformation.vector6_from_transformation()
        vec6 = factor * vec6
        exp = vec6.vec6_to_transformation()
        return exp

    def __rmul__(self, other):
        position = self.position * other
        rotation = self.rotation * other
        return PoseTransformer(position, rotation)


def normalize_vector(v):
    return v / np.linalg.norm(v)


def arc_sinx_overx(x):
    if is_lessthen_epsilons4th_root(math.fabs(x)):
        return 1.0 * x * x * float(1.0 / 6.0)
    return np.arcsin(x) / x


def is_lessthen_epsilons4th_root(x):
    epsilon4throot = math.pow(epsilon, float(1.0 / 4.0))
    return x < epsilon4throot


class AngleAxis:

    def __init__(self, angle, axis):
        self.angle = angle
        self.axis = axis

    def from_quaternion_3x3(self, mat):
        T = PoseTransformer.convert_back_np(mat)
        from_quaternion_to_angle_axis(T.rotation)

    def __str__(self):
        return str(np.insert(self.axis, 0, self.angle))

    def angle_axis_to_quaternion(self):
        s = np.sin(self.angle / 2)
        w = np.cos(self.angle / 2)
        x = self.axis[0] * s
        y = self.axis[1] * s
        z = self.axis[2] * s
        return np.array([w, x, y, z])


def from_quaternion_to_angle_axis(quat):
    if quat[0] > 1:
        quat = normalize_vector(quat)

    angle = 2 * np.arccos(quat[0])
    s = np.sqrt(1 - (quat[0] * quat[0]))

    if s < 0.001:
        x = quat[1]
        y = quat[2]
        z = quat[3]
    else:
        x = quat[1] / s
        y = quat[2] / s
        z = quat[3] / s
    axis = np.array([x, y, z])
    return angle, axis
