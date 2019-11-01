import threading
from multiprocessing import Process


class EMVSThread(threading.Thread):
    thread_lock = threading.Lock()

    def __init__(self, thread_id, name, counter, target):
        threading.Thread.__init__(self, target=target)
        self.threadID = thread_id
        self.name = name
        self.counter = counter

    def run(self):
        print "Starting " + self.name
        # Get lock to synchronize threads
        EMVSThread.thread_lock.acquire()
        # Free lock to release next thread
        EMVSThread.thread_lock.release()


class PlaneThread(threading.Thread):
    thread_lock = threading.Lock()

    def __init__(self, thread_id, name, packet_size, raw_depths_vec, fx, fy, cx, cy, dsi, N, X, Y, batch_step, camera_centers, depth_plane,
                 event_locations_z0, z0, end_plane=99):
        threading.Thread.__init__(self)
        self.threadID = thread_id
        self.name = name
        self.N = N
        self.X = X
        self.Y = Y
        self.batch_step = batch_step
        self.camera_centers = camera_centers
        self.depth_plane = depth_plane
        self.event_locations_z0 = event_locations_z0
        self.z0 = z0
        self.packet_size = packet_size
        self.raw_depths_vec = raw_depths_vec
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.dsi = dsi
        self.end_plane = end_plane

    def run(self):
        print "Starting " + self.name
        # Get lock to synchronize threads
        # EMVSThread.thread_lock.acquire()
        self.vote_events()
        # Free lock to release next thread
        # EMVSThread.thread_lock.release()

    def vote_events(self):
        for depth_plane in range(self.depth_plane, self.end_plane):
            self.batch_step = 0
            # time.sleep(0.01)
            for packet in range(len(self.camera_centers)):

                C = self.camera_centers[packet]
                zi = float(self.raw_depths_vec[depth_plane])
                a = self.z0 * (zi - C[2])
                bx = (self.z0 - zi) * (C[0] * self.fx + C[2] * self.cx)
                by = (self.z0 - zi) * (C[1] * self.fy + C[2] * self.cy)
                d = zi * (self.z0 - C[2])

                for batch in range(self.packet_size / self.N):
                    for i in range(self.N):
                        self.X[i] = self.event_locations_z0[self.batch_step + i][0]
                        self.Y[i] = self.event_locations_z0[self.batch_step + i][1]

                    self.X = (self.X * a + bx) / d
                    self.Y = (self.Y * a + by) / d
                    for i in range(self.N):
                        self.dsi.vote_grid_value_at(self.X[i], self.Y[i], depth_plane)
                    if self.batch_step < len(self.event_locations_z0):
                        self.batch_step += self.N


class PlaneProcess(Process):

    def __init__(self, thread_id, name, packet_size, raw_depths_vec, fx, fy, cx, cy, dsi, N, X, Y, batch_step, camera_centers, depth_plane,
                 event_locations_z0, z0, end_plane=99):
        super(PlaneProcess, self).__init__()
        self.threadID = thread_id
        self.name = name
        self.N = N
        self.X = X
        self.Y = Y
        self.batch_step = batch_step
        self.camera_centers = camera_centers
        self.depth_plane = depth_plane
        self.event_locations_z0 = event_locations_z0
        self.z0 = z0
        self.packet_size = packet_size
        self.raw_depths_vec = raw_depths_vec
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.dsi = dsi
        self.end_plane = end_plane

    def run(self):
        print "Starting " + self.name
        # Get lock to synchronize threads
        # EMVSThread.thread_lock.acquire()
        self.vote_events()
        # Free lock to release next thread
        # EMVSThread.thread_lock.release()

    def vote_events(self):
        for depth_plane in range(self.depth_plane, self.end_plane):
            self.batch_step = 0
            for packet in range(len(self.camera_centers)):

                C = self.camera_centers[packet]
                zi = float(self.raw_depths_vec[depth_plane])
                a = self.z0 * (zi - C[2])
                bx = (self.z0 - zi) * (C[0] * self.fx + C[2] * self.cx)
                by = (self.z0 - zi) * (C[1] * self.fy + C[2] * self.cy)
                d = zi * (self.z0 - C[2])

                for batch in range(self.packet_size / self.N):
                    for i in range(self.N):
                        self.X[i] = self.event_locations_z0[self.batch_step + i][0]
                        self.Y[i] = self.event_locations_z0[self.batch_step + i][1]

                    self.X = (self.X * a + bx) / d
                    self.Y = (self.Y * a + by) / d
                    for i in range(self.N):
                        self.dsi.vote_grid_value_at(self.X[i], self.Y[i], depth_plane)
                    if self.batch_step < len(self.event_locations_z0):
                        self.batch_step += self.N
