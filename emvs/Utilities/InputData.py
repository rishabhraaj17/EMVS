import rosbag
from dvs_msgs.msg import Event
from geometry_msgs.msg import PoseStamped
from rospy.rostime import Time
from sensor_msgs.msg import CameraInfo

from Utilities import PoseTransforms, Helpers as helpers


def events_sort_key(event):
    return event.ts


def parse_rosbag(input_rosbag, event_topic, camera_info_topic, pose_topic, tmin, tmax):
    topics = {'event_topic': event_topic, 'camera_info_topic': camera_info_topic, 'pose_topic': pose_topic}
    time_pose_dict = {}
    events = []
    initial_ts = None
    got_initial_ts = False
    continue_loop = True
    camera_info_msg = None
    i = 0

    with rosbag.Bag(input_rosbag, 'r') as current_bag:
        for topic, msg, t in current_bag.read_messages(topics=topics.values()):
            if continue_loop is False:
                break
            if topic == event_topic:
                if msg is not None:
                    if msg.events is None:
                        continue
                    ts = msg.events[0].ts
                    if got_initial_ts is False:
                        initial_ts = ts
                        got_initial_ts = True
                        print "Initial Stamp (events): " + str(initial_ts)

                    for i in range(len(msg.events)):
                        relative_stamp = helpers.time_difference_in_secs(msg.events[i].ts, initial_ts)
                        if relative_stamp < tmin:
                            continue
                        if relative_stamp > tmax:
                            continue_loop = False

                        ev_modified = Event(msg.events[i].x, msg.events[i].y, msg.events[i].ts, msg.events[i].polarity)
                        ev_modified.ts = Time(helpers.stamp_to_seconds(ev_modified.ts) - helpers.stamp_to_seconds(initial_ts))
                        events.append(ev_modified)

            if topic == camera_info_topic:
                camera_info_msg = CameraInfo(msg.header, msg.height, msg.width, msg.distortion_model, msg.D, msg.K, msg.R, msg.P, msg.binning_x,
                                             msg.binning_y, msg.roi)

            if topic == pose_topic:
                pose_msg = PoseStamped(msg.header, msg.pose)
                pose_msg_ts = pose_msg.header.stamp

                if got_initial_ts is False:
                    initial_ts = pose_msg_ts
                    got_initial_ts = True
                    print "Initial Stamp (pose): " + str(initial_ts)

                relative_stamp = helpers.time_difference_in_secs(msg.header.stamp, initial_ts)
                if relative_stamp < tmin:
                    continue
                if relative_stamp > tmax:
                    continue_loop = False

                transformation = PoseTransforms.PoseTransformer.build_from_pose_stamped(pose_msg)
                diff = float(helpers.time_difference_in_secs(pose_msg.header.stamp, initial_ts))
                transformation_time = Time.from_seconds(diff)
                time_pose_dict.update({transformation_time: transformation})

    events.sort(key=events_sort_key)
    return events, time_pose_dict, camera_info_msg
