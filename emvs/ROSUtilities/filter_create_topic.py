import rospy
from dvs_msgs.msg import EventArray
import sensor_msgs.msg._Image as img
import publish_camera_info as camera_info
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Header

events_header = Header()


def display_events(data, args):
    global events_header
    pub = args[0]
    pub.publish(data)
    events_header = data.header
    print "Event Published"


def display_image_raw(data, args):
    pub = args[0]
    pub.publish(data)
    print "Raw Image Published"


def publish_pose(data, args):
    pub = args[0]
    pose = PoseStamped()
    pose.header = events_header
    pose.header.frame_id = "/davis_linear_slider"
    pose.header.seq = 0  # Seems Irrelevant
    #print data.name.index('wl_snake')
    pose.pose = data.pose[data.name.index('wl_snake::module_1')]
    #print pose.pose
    pub.publish(pose)
    print "Pose Published"


def listener():
    dvs = rospy.init_node('dvs_camera', anonymous=True)

    pub_event = rospy.Publisher('/dvs/events', EventArray, queue_size=10)
    sub_event = rospy.Subscriber('head/dvs128/events', EventArray, display_events, (pub_event, None))

    pub_image_raw = rospy.Publisher('/dvs/image_raw', img.Image, queue_size=10)
    sub_image_raw = rospy.Subscriber('/snake/camera/image_raw', img.Image, display_image_raw, (pub_image_raw, None))

    pub_pose = rospy.Publisher('/dvs/pose', PoseStamped, queue_size=10)
    sub_pose = rospy.Subscriber('/gazebo/link_states', ModelStates, publish_pose, (pub_pose, None))

    camera_info.publish_cinfo()

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
