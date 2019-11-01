#include <dvs_to_point_cloud/voxel_grid.h>

/**
  * Cast rays from sensor origin to each point from the point cloud using
  * the fast voxel traversal algorithm.
  * Before casting, the transformation is applied to the sensor origin and
  * the point cloud.
  */
bool VoxelGrid::castRays(const sensor_msgs::PointCloud& pc,
                         const geometry_msgs::Point& sensor_origin,
                         tf::Transform tf)
{
    // Using "A Fast Voxel Traversal Algorithm for Ray Tracing"
    // by Amanatides and Woo
    // http://www.cse.chalmers.se/edu/year/2010/course/TDA361/grid.pdf

    tf::Point t_sensor_origin(sensor_origin.x,
                              sensor_origin.y,
                              sensor_origin.z);

    t_sensor_origin = tf * t_sensor_origin;

    bool origin_in_bbx = isInBBX(t_sensor_origin.x(),
                                 t_sensor_origin.y(),
                                 t_sensor_origin.z());

    double grid_min[3];
    int idx[3];
    int step[3];
    int cur_dir;
    double t_max[3];
    double t_delta[3];
    double direction[3];
    double intrsct_p[3];
    double cur_vox_min_border[3];
    double cur_vox_max_border[3];
    double min_rest;
    double cur_p[3];

    grid_min[0] = bbx_min_.x;
    grid_min[1] = bbx_min_.y;
    grid_min[2] = bbx_min_.z;

    tf::Point intrsct_vec;

    int score_update = 20;

    for (const auto& p32 : pc.points)
    {
        tf::Point p(p32.x, p32.y, p32.z);
        p = tf * p;

        int p_idx[] = { -1, -1, -1 };

        metricToIndices(p.x(),    p.y(),    p.z(),
                        p_idx[0], p_idx[1], p_idx[2]);

        // compute the direction of the ray
        tf::Point dir_vec = p - t_sensor_origin;
        direction[0] = dir_vec.x();
        direction[1] = dir_vec.y();
        direction[2] = dir_vec.z();

        if (!origin_in_bbx)
        {
            // find where the ray enters the bounding box
            double q = (grid_min[0] + 0.5 * resolution_ - t_sensor_origin.x()) / (p.x() - t_sensor_origin.x());
            if (q > 1)
            {
                // ray ends before intersecting with the volume - continue
                // to the next point
                continue;
            }
            intrsct_vec = t_sensor_origin + (p - t_sensor_origin) * q;
        }
        else
        {
            intrsct_vec = t_sensor_origin;
        }

        intrsct_p[0] = intrsct_vec.x();
        intrsct_p[1] = intrsct_vec.y();
        intrsct_p[2] = intrsct_vec.z();

        // if the intersection point lies inside the bounding
        // box, cast ray
        if (metricToIndices(intrsct_p[0],
                            intrsct_p[1],
                            intrsct_p[2],
                            idx[0],
                            idx[1],
                            idx[2]))
        {
            // initialization phase of the voxel traversing algorithm
            for (int i = 0; i < 3; i++)
            {
                cur_vox_min_border[i] = grid_min[i] + resolution_ * idx[i];
                cur_vox_max_border[i] = cur_vox_min_border[i] + resolution_;

                step[i] = (direction[i] > 0) ?  1 :
                          (direction[i] < 0) ? -1 :
                                                0 ;
                t_max[i] = (step[i] >= 0)
                      ? (cur_vox_max_border[i] - intrsct_p[i]) / direction[i]
                      : (cur_vox_min_border[i] - intrsct_p[i]) / direction[i];

                t_delta[i] = std::fabs(resolution_ / direction[i]);
            }

            // incremental phase of the voxel traversing algorithm
            // (continue while the current voxel is inside the bounding box)
            while (      isInBBX(  idx[0],   idx[1],   idx[2]) &&
                   !indicesEqual(  idx[0],   idx[1],   idx[2],
                                 p_idx[0], p_idx[1], p_idx[2]))
            {
                // update score
                data_[dataIndexTrusted(idx[0], idx[1], idx[2])] =
                    (uint8_t)std::min(
                                (int)data_[dataIndexTrusted(idx[0], idx[1], idx[2])] + score_update,
                                (int)max_val_
                            );

                // next iteration
                min_rest = t_max[0];
                cur_dir = 0;
                for (int i = 1; i < 3; i++)
                {
                    if (t_max[i] < min_rest)
                    {
                        min_rest = t_max[i];
                        cur_dir = i;
                    }
                }
                idx[cur_dir] += step[cur_dir];
                t_max[cur_dir] += t_delta[cur_dir];
            }
        }
    }

}



/**
  * Cast rays from sensor origin to each point from the point cloud
  * (voxel grid is considered a projective voxel grid).
  * Before casting, the transformation is applied to the sensor origin and
  * the point cloud.
  */
bool VoxelGrid::castRaysProjective(const sensor_msgs::PointCloud& pc,
                                   const geometry_msgs::Point& sensor_origin,
                                   tf::Transform tf)
{
    tf::Point t_sensor_origin(sensor_origin.x,
                              sensor_origin.y,
                              sensor_origin.z);

    t_sensor_origin = tf * t_sensor_origin;

    int score_update = 20;
    int idx[3];

    for (const auto& p32 : pc.points)
    {
        tf::Point p(p32.x, p32.y, p32.z);
        p = tf * p;

        tf::Point dir = (p - t_sensor_origin);
        tf::Point dir_step = dir * (resolution_ / dir.x());

        double min_depth = bbx_min_.x + resolution_ * 0.5;
        double q = (min_depth - t_sensor_origin.x()) / (p.x() - t_sensor_origin.x());
        tf::Point cur_unproj = t_sensor_origin + dir * q;

        while (cur_unproj.x() < std::min(p.x(), bbx_max_.x))
        {
            if (metricToIndices((double)(cur_unproj.x()),
                                (double)(cur_unproj.y() * bbx_min_.x / cur_unproj.x()),
                                (double)(cur_unproj.z() * bbx_min_.x / cur_unproj.x()),
                                idx[0],
                                idx[1],
                                idx[2]))
            {
                data_[dataIndexTrusted(idx[0], idx[1], idx[2])] =
                    (uint8_t)std::min(
                                (int)data_[dataIndexTrusted(idx[0], idx[1], idx[2])] + score_update,
                                (int)max_val_
                            );
            }
            cur_unproj += dir_step;
        }
    }
}



/**
 * Create a point cloud from the data in the voxel grid, apply the transformation
 * and add the transformed points to the output point cloud cloud_filtered.
 */
void VoxelGrid::voxelGrid2PointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_filtered,
                                     tf::Transform tf)
{
    // find the first local maximum in each row of the voxel grid in direction of growing depth
    cv::Mat depths(z_size_, y_size_, CV_16UC1);
    for (int z_idx = 0; z_idx < z_size_; z_idx++)
    {
        for (int y_idx = 0; y_idx < y_size_; y_idx++)
        {
            uint8_t ray_max_val = 0;
            int ray_max_idx = 0;
            for (int x_idx = 0; x_idx < x_size_; x_idx++)
            {
                if (data_[dataIndexTrusted(x_idx, y_idx, z_idx)] > ray_max_val)
                {
                     ray_max_val = data_[dataIndexTrusted(x_idx, y_idx, z_idx)];
                     ray_max_idx = x_idx;
                }
                else if (data_[dataIndexTrusted(x_idx, y_idx, z_idx)] < ray_max_val)
                {
                    break;
                }
            }
            depths.at<uint16_t>(z_idx, y_idx) = (uint16_t)ray_max_idx;
        }
    }

    cv::Mat depths8;
    cv::normalize(depths, depths8, 0.0, 255.0, cv::NORM_MINMAX);
    depths8.convertTo(depths8, CV_8U);

    cv::Mat mask;
    cv::adaptiveThreshold(depths8,
                          mask,
                          1,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          CV_THRESH_BINARY,
                          5,
                          -10);

    cv::Mat depths_thresh;
    depths.copyTo(depths_thresh, mask);

    cv::Mat depths_filtered;
    cv::medianBlur(depths_thresh, depths_filtered, 5);

    geometry_msgs::Point32 p;
    for (int z_idx = 0; z_idx < z_size_; z_idx++)
    {
        for (int y_idx = 0; y_idx < y_size_; y_idx++)
        {
            if (depths_thresh.at<uint16_t>(z_idx, y_idx) > 0)
            {
                indicesToMetric(depths_thresh.at<uint16_t>(z_idx, y_idx),
                                y_idx,
                                z_idx,
                                p.x,
                                p.y,
                                p.z);
                tf::Point p_tf(p.x,
                               p.y,
                               p.z);
                p_tf = tf * p_tf;
                pcl::PointXYZRGB prgb;
                prgb.x = p_tf.x();
                prgb.y = p_tf.y();
                prgb.z = p_tf.z();
                prgb.r = 255;
                prgb.g = 255;
                prgb.b = 255;
                cloud_filtered->push_back(prgb);
            }
        }
    }
}
