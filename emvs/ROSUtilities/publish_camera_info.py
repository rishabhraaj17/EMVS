import rospy
from sensor_msgs.msg import CameraInfo
import std_msgs.msg
import math

def publish_cinfo():
    pub = rospy.Publisher('/dvs/camera_info', CameraInfo, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    height = 128
    width = 128
    cx = (float(width) + 1.0) / 2.0
    cy = (float(height) + 1.0) / 2.0
    hov = 1.5707963267948966
    focal_length = (float(width)) / (2.0 * math.tan(hov/2.0))
    hack_baseline = 0.0
    while not rospy.is_shutdown():
        info = CameraInfo()
        info.header = std_msgs.msg.Header()
        info.height = height
        info.width = width
        info.distortion_model = "plumb_bob"
        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.K = [focal_length, 0.0, cx, 0.0, focal_length, cy, 0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [focal_length, 0.0, cx, -focal_length * hack_baseline, 0.0, focal_length, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        pub.publish(info)
        print "\nCamera Info Published"
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_cinfo()
    except rospy.ROSInterruptException:
        pass