
#ifndef SURFACE_DETECTION_SURFACE_DETECTION_IMPROVED_H
#define SURFACE_DETECTION_SURFACE_DETECTION_IMPROVED_H

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

// std headers
#include <random>

// PCL headers
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/range_image/range_image.h>

class SurfaceDetection
{
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_, robot_status_sub_;
    ros::Publisher coordinates_pub_;
    ros::ServiceClient clientScript;
    bool robotFlag, holeFlag;
public:
    SurfaceDetection();
    void pointcloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
    void robotstatusCb(const std_msgs::String& msg);
    bool point_is_in_cylinder(pcl::PointXYZ pt, pcl::PointXYZ pt_cyl1, pcl::PointXYZ pt_cyl2, double r);
    double degToRad(double degree);
    geometry_msgs::Point crossProduct(geometry_msgs::Point A, geometry_msgs::Point B);
};

pcl::visualization::PCLVisualizer* viewer;

#endif //SURFACE_DETECTION_SURFACE_DETECTION_IMPROVED_H
