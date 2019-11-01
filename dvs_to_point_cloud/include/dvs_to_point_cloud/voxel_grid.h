#ifndef DVS_TO_POINT_CLOUD_VOXEL_GRID_H
#define DVS_TO_POINT_CLOUD_VOXEL_GRID_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


class VoxelGrid
{
private:

/*
 * Private trusted functions perform no range check to increase speed.
 */

inline int dataIndexTrusted(int x_idx,
                            int y_idx,
                            int z_idx)
{
    return x_idx * (y_size_ * z_size_) + z_idx * y_size_ + y_idx;
}


public:

VoxelGrid(const geometry_msgs::Point& bbx_min,
          const geometry_msgs::Point& bbx_max,
          const double resolution)
    : resolution_(resolution),
      bbx_min_(bbx_min),
      bbx_max_(bbx_max),
      x_size_(std::ceil((bbx_max.x - bbx_min.x) / resolution)),
      y_size_(std::ceil((bbx_max.y - bbx_min.y) / resolution)),
      z_size_(std::ceil((bbx_max.z - bbx_min.z) / resolution)),
      max_val_(240), 
      min_val_(1)
{
    assert(resolution > 0);
    assert(bbx_max.x > bbx_min.x);
    assert(bbx_max.y > bbx_min.y);
    assert(bbx_max.z > bbx_min.z);

    data_ = new uint8_t[x_size_ * y_size_ * z_size_];
    std::fill_n(data_, x_size_ * y_size_ * z_size_, 0);
}

~VoxelGrid()
{
    delete [] data_;
}


inline bool indicesEqual(const int x_idx_1,
                         const int y_idx_1,
                         const int z_idx_1,
                         const int x_idx_2,
                         const int y_idx_2,
                         const int z_idx_2)
{
    return (x_idx_1 == x_idx_2 &&
            y_idx_1 == y_idx_2 &&
            z_idx_1 == z_idx_2);
}

inline bool isInBBX(const int x_idx,
                    const int y_idx,
                    const int z_idx)
{
    return (y_idx >= 0 && y_idx < y_size_ &&
            z_idx >= 0 && z_idx < z_size_ &&
            x_idx >= 0 && x_idx < x_size_);
}

inline bool metricToIndices(const double x_metric,
                            const double y_metric,
                            const double z_metric,
                            int& x_idx,
                            int& y_idx,
                            int& z_idx)
{
    int x_idx_calc = std::floor((x_metric - bbx_min_.x) / resolution_);
    int y_idx_calc = std::floor((y_metric - bbx_min_.y) / resolution_);
    int z_idx_calc = std::floor((z_metric - bbx_min_.z) / resolution_);

    if (!isInBBX(x_idx_calc, y_idx_calc, z_idx_calc)) { return false; }

    x_idx = x_idx_calc;
    y_idx = y_idx_calc;
    z_idx = z_idx_calc;
    return true;
}

inline bool isInBBX(const double x_metric,
                    const double y_metric,
                    const double z_metric)
{
    int x_idx, y_idx, z_idx;
    return (metricToIndices(x_metric,
                            y_metric,
                            z_metric,
                            x_idx,
                            y_idx,
                            z_idx));
}

inline bool indicesToMetric(const int x_idx,
                            const int y_idx,
                            const int z_idx,
                            double& x_metric,
                            double& y_metric,
                            double& z_metric)
{
    if (!isInBBX(x_idx, y_idx, z_idx)) { return false; }

    x_metric = bbx_min_.x + resolution_ * x_idx + resolution_ * 0.5;
    y_metric = bbx_min_.y + resolution_ * y_idx + resolution_ * 0.5;
    z_metric = bbx_min_.z + resolution_ * z_idx + resolution_ * 0.5;
    return true;
}

inline bool indicesToMetric(const int x_idx,
                            const int y_idx,
                            const int z_idx,
                            float& x_metric_f,
                            float& y_metric_f,
                            float& z_metric_f)
{
    double x_metric, y_metric, z_metric;
    if (!indicesToMetric(x_idx,
                         y_idx,
                         z_idx,
                         x_metric,
                         y_metric,
                         z_metric))
    {
        return false;
    }

    x_metric_f = static_cast<float>(x_metric);
    y_metric_f = static_cast<float>(y_metric);
    z_metric_f = static_cast<float>(z_metric);
    return true;
}

inline int dataIndex(int x_idx,
                     int y_idx,
                     int z_idx)
{
    if (!isInBBX(x_idx, y_idx, z_idx)) { return -1; }

    return x_idx * (y_size_ * z_size_) + z_idx * y_size_ + y_idx;
}



/**
  * Cast rays from sensor origin to each point from the point cloud using
  * the fast voxel traversal algorithm.
  * Before casting, the transformation is applied to the sensor origin and
  * the point cloud.
  */
bool castRays(const sensor_msgs::PointCloud& pc,
              const geometry_msgs::Point& sensor_origin,
              tf::Transform tf);

/**
  * Cast rays from sensor origin to each point from the point cloud
  * (voxel grid is considered a projective voxel grid).
  * Before casting, the transformation is applied to the sensor origin and
  * the point cloud.
  */
bool castRaysProjective(const sensor_msgs::PointCloud& pc,
                        const geometry_msgs::Point& sensor_origin,
                        tf::Transform tf);

/**
 * Create a point cloud from the data in the voxel grid, apply the transformation
 * and add the transformed points to the output point cloud cloud_filtered.
 */
void voxelGrid2PointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_filtered,
                          tf::Transform tf);


// occupancy data as 1D array
uint8_t *data_;

// voxel grid integer dimensions (number of voxels in each direction)
const size_t x_size_;
const size_t y_size_;
const size_t z_size_;

// resolution (metric length of the voxel side)
const double resolution_;

// bounding box minimum point absolute coordinates
const geometry_msgs::Point bbx_min_;
// bounding box maximum point absolute coordinates
const geometry_msgs::Point bbx_max_;

// max and min values
const uint8_t max_val_;
const uint8_t min_val_;

};

#endif
