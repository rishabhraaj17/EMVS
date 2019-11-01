#include <dvs_to_point_cloud/dvs_to_point_cloud.h>

// number of voxel grids used to discretize the space
#define NUM_VGS 6

ros::Publisher cam_pose_pub;
ros::Publisher pc_pub;

// voxel grids and transformations from the fixed frame to VGs' frames
VoxelGrid *vg[NUM_VGS];
tf::Transform vg_tfs[NUM_VGS];


void eventsAndCamPoseCB(const dvs_msgs::EventArrayConstPtr& event_array,
                        const geometry_msgs::PoseStampedConstPtr& cam_pose)
{
    tf::Transform cam_to_map_tf;
    tf::poseMsgToTF(cam_pose->pose, cam_to_map_tf);

    tf::Point cam_origin(0.15, 0.0, 0.0); // camera offset in snake head's frame
    cam_origin = cam_to_map_tf * cam_origin; // camera position in fixed frame

    geometry_msgs::Point co_geo;
    co_geo.x = cam_origin.x();
    co_geo.y = cam_origin.y();
    co_geo.z = cam_origin.z();

    sensor_msgs::PointCloud pc;

    geometry_msgs::Point32 p32;

    float fl = 64.0;
    float x;
    float y;

    // back-projection
    for (const auto &e : event_array->events)
    {
        x = e.x - 64.0;
        y = e.y - 64.0;
        tf::Point p( 2.0,
                    -2.0 * x / fl,
                    -2.0 * y / fl);
        p = cam_to_map_tf * p;
        p32.x = p.x();
        p32.y = p.y();
        p32.z = p.z();
        pc.points.push_back(p32);
    }

    // raycasting
    for (int i = 0; i < NUM_VGS; i++)
    {
        vg[i]->castRays(pc, co_geo, vg_tfs[i]);
    }

    // point cloud reconstruction
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < NUM_VGS; i++)
    {
        vg[i]->voxelGrid2PointCloud(cloud_filtered, vg_tfs[i].inverse());
    }

    cloud_filtered->header.frame_id = "map";
    pc_pub.publish(*cloud_filtered);
}

void modelStatesCB(const gazebo_msgs::ModelStatesConstPtr& model_states)
{
    geometry_msgs::PoseStamped cam_pose;
    cam_pose.header.frame_id = "map";
    cam_pose.header.stamp = ros::Time::now();

    auto it = std::find(model_states->name.begin(), model_states->name.end(), "wl_snake");

    if (it == model_states->name.end())
    {
        ROS_ERROR("Could not find the snake state");
    }
    else
    {
        cam_pose.pose = model_states->pose[it - model_states->name.begin()];
        cam_pose_pub.publish(cam_pose);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dvs_to_point_cloud");
    ros::NodeHandle nh;

    message_filters::Subscriber<dvs_msgs::EventArray> event_array_sub(nh, "/head/dvs128/events", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> cam_pose_sub(nh, "/dvs/camera_pose", 1);
    message_filters::Synchronizer<ApproxPolicy> sync(ApproxPolicy(3), event_array_sub, cam_pose_sub);
    sync.registerCallback(boost::bind(&eventsAndCamPoseCB, _1, _2));

    ros::Subscriber ms_sub = nh.subscribe("/gazebo/model_states", 1, modelStatesCB);

    cam_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/dvs/camera_pose", 1);
    pc_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/dvs/point_cloud", 1);


    // initialize voxel grids

    geometry_msgs::Point bbx_min;
    geometry_msgs::Point bbx_max;
    bbx_min.x =  0.86;
    bbx_min.y = -1.0;
    bbx_min.z = -1.0;
    bbx_max.x =  4.0;
    bbx_max.y =  1.0;
    bbx_max.z =  1.5;

    for (int i = 0; i < NUM_VGS; i++)
    {
        vg[i] = new VoxelGrid(bbx_min, bbx_max, 0.01);
    }


    // initialize transformations from map (fixed frame) to each voxel grid's frame

    tf::Transform step(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), -CV_PI / NUM_VGS),
                       tf::Vector3(0, 0, 0));

    tf::Transform tf(tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), (NUM_VGS - 1) * CV_PI / (NUM_VGS * 2.0)),
                     tf::Vector3(0, 0, 0));

    for (int i = 0; i < NUM_VGS; i++)
    {
        vg_tfs[i] = tf;
        tf *= step;
    }

    
    ros::spin();
    
    return 0;
}
