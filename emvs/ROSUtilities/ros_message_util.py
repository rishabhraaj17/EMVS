class RosMessages:
    def __init__(self):
        self.event = None
        self.pose = None
        self.camera_info = None

    def event_callback(self, msg):
        self.event = msg

    def pose_callback(self, msg):
        self.pose = msg

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def callback(self, event, pose, camera_info):
        self.event = event
        self.pose = pose
        self.camera_info = camera_info

