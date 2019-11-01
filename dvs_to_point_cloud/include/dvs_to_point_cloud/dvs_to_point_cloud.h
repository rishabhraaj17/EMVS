#ifndef DVS_TO_POINT_CLOUD_H
#define DVS_TO_POINT_CLOUD_H

#include <dvs_to_point_cloud/voxel_grid.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <algorithm>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

typedef message_filters::sync_policies::ApproximateTime<dvs_msgs::EventArray, geometry_msgs::PoseStamped> ApproxPolicy;

#endif  // DVS_TO_POINT_CLOUD_H
