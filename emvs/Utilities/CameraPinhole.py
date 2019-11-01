import numpy as np

ray_vector = np.zeros((3, 1), dtype=np.float64)
key_point = np.zeros((2, 1), dtype=np.float64)


class PinholeCamera:

    def __init__(self, width, height, fx, fy, cx, cy):
        self.width, self.height, self.fx, self.fy, self.cx, self.cy = \
            width, height, fx, fy, cx, cy
        # calculating intrinsic
        self.k = [
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0]
        ]
        self.k_inv = np.linalg.inv(self.k)

    def project_3d_to_pixel(self, p):
        '''
        p is [x, y, z]. we are dividing x and y by z to bring back the 3d coordinates to pixel by removing depth
        :param p: point
        :return: keypoint
        '''
        global key_point
        key_point = [
            [self.fx * p[0] / p[2] + self.cx],
            [self.fy * p[1] / p[2] + self.cy]]
        return key_point

    def project_pixel_to_3d_ray(self, kp):
        '''

        :param kp: keypoint
        :return: ray vector
        '''
        global ray_vector
        ray_vector = [[(kp[0] - self.cx) / self.fx],
                      [(kp[1] - self.cy) / self.fy],
                      [1.0]]
        return ray_vector
