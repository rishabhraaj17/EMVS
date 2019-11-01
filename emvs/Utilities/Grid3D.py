import numpy as np
import multiprocessing


class Grid3D:

    def __init__(self, dim_x=0, dim_y=0, dim_z=0):
        self.size = []
        self.numCells = 0
        self.data_array = []
        if not self.size:
            self.deallocate()
        self.allocate(dim_x, dim_y, dim_z)

    def allocate(self, dim_x, dim_y, dim_z):
        x = dim_x
        y = dim_y
        z = dim_z
        self.size = [x, y, z]
        self.numCells = self.size[0] * self.size[1] * self.size[2]
        self.data_array = np.zeros(500, dtype=np.float32)
        self.data_array = np.resize(self.data_array, self.numCells)

    def deallocate(self):
        if not self.size:
            pass
        else:
            self.size[0] = 0
            self.size[1] = 0
            self.size[2] = 0
        self.numCells = 0
        self.data_array = []

    def get_grid_value_at(self, ix, iy, iz):
        i = int(ix + self.size[0] * (iy + self.size[1] * iz))
        return self.data_array[i]

    def vote_grid_value_at(self, x_f, y_f, layer):
        layer_index_jump_factor = layer * self.size[0] * self.size[1]
        if x_f >= float(0) and y_f >= float(0):
            x = int(x_f)
            y = int(y_f)
            if x + 1 < self.size[0] and y + 1 < self.size[1]:
                from_new_start_position_goto = x + y * self.size[0]
                g_pointer = layer_index_jump_factor + int(from_new_start_position_goto)
                fx = x_f - x
                fy = y_f - y
                fx1 = float(1) - fx
                fy1 = float(1) - fy

                self.data_array[g_pointer] += fx1 * fy1
                self.data_array[g_pointer + 1] += fx * fy1
                self.data_array[g_pointer + self.size[0]] += fx1 * fy
                self.data_array[g_pointer + self.size[0] + 1] += fx * fy

    def collect_local_maxima(self):
        u_size = np.uint(self.size[0])
        v_size = np.uint(self.size[1])
        w_size = np.uint(self.size[2])
        depth_cell_indices = np.empty((v_size, u_size), dtype=np.float32)  # check: size
        max_val = np.zeros((v_size, u_size, 1), dtype=np.float32)
        max_pos_idx = np.zeros((v_size, u_size), dtype=np.uint8)

        grid_vals_vec = np.empty(w_size, dtype=np.float32)

        for v in range(0, v_size):
            for u in range(0, u_size):
                for k in range(0, w_size):
                    grid_vals_vec[k] = self.get_grid_value_at(u, v, k)

                max = np.max(grid_vals_vec)
                max_ind = np.argmax(grid_vals_vec)
                max_val[v][u] = max
                max_pos_idx[v][u] = max_ind
        return max_val, max_pos_idx

    def reset_grid(self):
        for i in range(len(self.data_array)):
            self.data_array[i] = float(0)

    def write_grid_npy(self, sz_filename):
        data = np.reshape(self.data_array, (self.size[2], self.size[1], self.size[0]))
        np.save(sz_filename, data)

    def get_dimensions(self):
        dim_x = self.size[0]
        dim_y = self.size[1]
        dim_z = self.size[2]
        return dim_x, dim_y, dim_z

    def print_info(self):
        print("Voxel Grid Dimensions: " + "(" + self.size[0] + "," + self.size[1] + "," + self.size[2] + ")")
        print("Voxel Grid data_array size: ", len(self.data_array))


class Grid3DPooled:

    manager = multiprocessing.Manager()

    def __init__(self, dim_x=0, dim_y=0, dim_z=0):
        self.size = []
        self.numCells = 0
        self.data_array = Grid3DPooled.manager.list()
        if not self.size:
            self.deallocate()
        self.allocate(dim_x, dim_y, dim_z)

    def allocate(self, dim_x, dim_y, dim_z):
        x = dim_x
        y = dim_y
        z = dim_z
        self.size = [x, y, z]
        self.numCells = self.size[0] * self.size[1] * self.size[2]
        self.data_array = Grid3DPooled.manager.list([0] * self.numCells)

    def deallocate(self):
        if not self.size:
            pass
        else:
            self.size[0] = 0
            self.size[1] = 0
            self.size[2] = 0
        self.numCells = 0
        self.data_array = Grid3DPooled.manager.list()

    def get_grid_value_at(self, ix, iy, iz):
        i = int(ix + self.size[0] * (iy + self.size[1] * iz))
        return self.data_array[i]

    def vote_grid_value_at(self, x_f, y_f, layer):
        layer_index_jump_factor = layer * self.size[0] * self.size[1]
        if x_f >= float(0) and y_f >= float(0):
            x = int(x_f)
            y = int(y_f)
            if x + 1 < self.size[0] and y + 1 < self.size[1]:
                from_new_start_position_goto = x + y * self.size[0]
                g_pointer = layer_index_jump_factor + int(from_new_start_position_goto)
                fx = x_f - x
                fy = y_f - y
                fx1 = float(1) - fx
                fy1 = float(1) - fy

                self.data_array[g_pointer] += fx1 * fy1
                self.data_array[g_pointer + 1] += fx * fy1
                self.data_array[g_pointer + self.size[0]] += fx1 * fy
                self.data_array[g_pointer + self.size[0] + 1] += fx * fy

    def collect_local_maxima(self):
        u_size = np.uint(self.size[0])
        v_size = np.uint(self.size[1])
        w_size = np.uint(self.size[2])
        depth_cell_indices = np.empty((v_size, u_size), dtype=np.float32)
        max_val = np.zeros((v_size, u_size, 1), dtype=np.float32)
        max_pos_idx = np.zeros((v_size, u_size), dtype=np.uint8)

        grid_vals_vec = np.empty(w_size, dtype=np.float32)

        for v in range(0, v_size):
            for u in range(0, u_size):
                for k in range(0, w_size):
                    grid_vals_vec[k] = self.get_grid_value_at(u, v, k)

                max = np.max(grid_vals_vec)
                max_ind = np.argmax(grid_vals_vec)
                max_val[v][u] = max
                max_pos_idx[v][u] = (grid_vals_vec[0] - max_ind)
        return max_val, max_pos_idx

    def reset_grid(self):
        for i in range(len(self.data_array)):
            self.data_array[i] = float(0)

    def write_grid_npy(self, sz_filename):
        data = np.reshape(self.data_array, (self.size[2], self.size[1], self.size[0]))
        np.save(sz_filename, data)

    def get_dimensions(self):
        dim_x = self.size[0]
        dim_y = self.size[1]
        dim_z = self.size[2]
        return dim_x, dim_y, dim_z

    def print_info(self):
        print("Voxel Grid Dimensions: " + "(" + self.size[0] + "," + self.size[1] + "," + self.size[2] + ")")
        print("Voxel Grid data_array size: ", len(self.data_array))

