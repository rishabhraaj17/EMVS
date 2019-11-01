import numpy as np


class DepthVector(object):

    def __init__(self, min_depth, max_depth, num_depth_cells):
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.num_depth_cells = num_depth_cells

        if self.min_depth < 0.0:
            raise ValueError("Minimum Depth must be greater than 0.0")
        if self.max_depth < 0.0:
            raise ValueError("Maximum Depth must be greater than 0.0")
        if self.num_depth_cells < 0:
            raise ValueError("Number of depth cells must be greater than 0")

        if self.min_depth > self.max_depth:
            self.min_depth, self.max_depth = max_depth, min_depth
        self.vec = None
        self.depth_to_cell_idx_multiplier = None

    def size(self):
        return self.vec.size

    def depth_to_cell_index(self, depth):
        return self.depth_to_cell_index(depth)

    def cell_index_to_depth(self, i):
        return self.cell_index_to_depth(i)

    def depth_to_cell(self, depth):
        return self.depth_to_cell(depth)

    def get_depth_vector(self):
        out = []
        for i in range(self.num_depth_cells):
            out.append(self.cell_index_to_depth(i))
        return np.asarray(out)


class LinearDepthVector(DepthVector):

    def __init__(self, min_depth, max_depth, num_depth_cells):
        super(LinearDepthVector, self).__init__(min_depth, max_depth, num_depth_cells)
        self.init()

    def init(self):
        self.depth_to_cell_idx_multiplier = float((self.num_depth_cells / (self.max_depth - self.min_depth)))
        self.vec = np.resize(self.vec, self.num_depth_cells)

        for i in range(self.num_depth_cells):
            self.vec[i] = self.min_depth + float(i) / self.depth_to_cell_idx_multiplier

    def cell_index_to_depth(self, i):
        return self.vec[i]

    def depth_to_cell_index(self, depth):
        return int((depth - self.min_depth) * self.depth_to_cell_idx_multiplier + 0.5)

    def depth_to_cell(self, depth):
        return (depth - self.min_depth) * self.depth_to_cell_idx_multiplier


class InverseDepthVector(DepthVector):

    def __init__(self, min_depth, max_depth, num_depth_cells):
        super(InverseDepthVector, self).__init__(min_depth, max_depth, num_depth_cells)
        self.inv_min_depth = None
        self.inv_max_depth = None
        self.init()

    def init(self):

        self.inv_min_depth = float(1) / self.min_depth
        self.inv_max_depth = float(1) / self.max_depth

        self.depth_to_cell_idx_multiplier = float((self.num_depth_cells / (self.inv_min_depth - self.inv_max_depth)))
        self.vec = np.resize(self.vec, self.num_depth_cells)

        for i in range(self.num_depth_cells):
            self.vec[i] = self.inv_max_depth + float(i) / self.depth_to_cell_idx_multiplier

    def cell_index_to_depth(self, i):
        return float(1) / self.vec[i]

    def depth_to_cell_index(self, depth):
        return int((float(1) / (depth - self.inv_max_depth)) * self.depth_to_cell_idx_multiplier + 0.5)

    def depth_to_cell(self, depth):
        return (float(1) / (depth - self.inv_max_depth)) * self.depth_to_cell_idx_multiplier
