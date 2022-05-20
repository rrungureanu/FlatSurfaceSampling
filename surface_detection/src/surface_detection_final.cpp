#include "../include/surface_detection_final.h"

SurfaceDetection::SurfaceDetection()
{
    // Create a ROS subscriber for the input point cloud and robot status
    pointcloud_sub_ = nh_.subscribe ("/camera/depth/color/points", 1, &SurfaceDetection::pointcloudCb, this);
    robot_status_sub_ = nh_.subscribe ("/surface_sampling/robot_status", 1, &SurfaceDetection::robotstatusCb, this);

    // Create a ROS publisher for the surface robot coordinates
    coordinates_pub_ = nh_.advertise<geometry_msgs::Point> ("/surface_sampling/surface_coordinates", 1);

    robotFlag = true;
}

bool SurfaceDetection::point_is_in_cylinder(pcl::PointXYZ pt, pcl::PointXYZ pt_cyl1, pcl::PointXYZ pt_cyl2, double r)
{
    pcl::PointXYZ vec; vec.x = pt_cyl2.x - pt_cyl1.x; vec.y = pt_cyl2.y - pt_cyl1.y; vec.z = pt_cyl2.z - pt_cyl1.z;
    pcl::PointXYZ vec2; vec2.x = pt.x - pt_cyl1.x; vec2.y = pt.y - pt_cyl1.y; vec2.z = pt.z - pt_cyl1.z;
    pcl::PointXYZ vec3; vec3.x = pt.x - pt_cyl2.x; vec3.y = pt.y - pt_cyl2.y; vec3.z = pt.z - pt_cyl2.z;
    pcl::PointXYZ cross; cross.x = vec2.y*vec.z - vec2.z*vec.y; cross.y = vec2.z*vec.x - vec2.x*vec.z; cross.z = vec2.x*vec.y - vec2.y*vec.x;
    double calc = r * sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
    if((vec.x*vec2.x + vec.y*vec2.y + vec.z*vec2.z) >= 0 &&
       (vec.x*vec3.x + vec.y*vec3.y + vec.z*vec3.z) <= 0 &&
       sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z) <= calc)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void SurfaceDetection::robotstatusCb(const std_msgs::String& msg)
{
    if(msg.data == "RobotReady")
        robotFlag = true;
}

void SurfaceDetection::pointcloudCb(const sensor_msgs::PointCloud2ConstPtr &input)
{
    if(robotFlag)
    {
        ROS_INFO("Cloud cb");
        pcl::ScopeTime scopeTimeAll("Time of pointcloud callback");
        {
            // Create a container for the data.
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obj(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>);

            //Convert ROS PointCloud2 to PCL PointCloud2
            pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2), cloud_filtered(new pcl::PCLPointCloud2);
            pcl_conversions::toPCL(*input, *pcl_pc2);

            // Create the filtering object: downsample the dataset using a leaf size of 1cm
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud(pcl_pc2);
            sor.setLeafSize(0.01f, 0.01f, 0.01f);
            sor.filter(*cloud_filtered);

            //Convert PCL PointCloud2 to PCL PointCloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(*cloud_filtered, *cloud);

            //Plane segmentation processing start
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            // Create the plane segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            // Optional
            seg.setOptimizeCoefficients(true);
            // Mandatory
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01);

            // Create the filtering object
            pcl::ExtractIndices<pcl::PointXYZ> extract;

            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0)
            {
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            }

            // Extract the inliers
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_p);
            extract.setNegative(true);
            extract.filter(*cloud_obj);

            // Random seed
            std::random_device rd;

            // Initialize Mersenne Twister pseudo-random number generator
            std::mt19937 gen(rd());

            // Generate pseudo-random numbers
            // uniformly distributed in range (1, 100)
            std::uniform_int_distribution<> dis(0, cloud_p->points.size());

            int bf_index = -1, bf_plane_count = -1, bf_obj_count = 9999999, plane_count = 0, obj_count = 0, CYLINDER_LENGTH = 5;
            double CYLINDER_RADIUS = 0.1;
            pcl::PointXYZ pt_orig, pt_cyl1, pt_cyl2, pt_approach;
            geometry_msgs::Point pt_robot;

            const auto plane_a = coefficients->values[0] / coefficients->values[3];
            const auto plane_b = coefficients->values[1] / coefficients->values[3];
            const auto plane_c = coefficients->values[2] / coefficients->values[3];

            for (int x = 0; x < 1000; x++)
            {
                plane_count = obj_count = 0;
                int randomPointIndex = dis(gen);
                std::cerr << "!" << randomPointIndex << "! ";
                pt_orig = cloud_p->points[randomPointIndex], pt_cyl1, pt_cyl2;
                //pcl::PointXYZ pt_orig = cloud_p->points[cloud->points.size()/2], pt_cyl1, pt_cyl2;

                pt_cyl1.x = pt_orig.x + CYLINDER_LENGTH * plane_a;
                pt_cyl1.y = pt_orig.y + CYLINDER_LENGTH * plane_b;
                pt_cyl1.z = pt_orig.z + CYLINDER_LENGTH * plane_c;
                pt_cyl2.x = pt_orig.x - CYLINDER_LENGTH * plane_a;
                pt_cyl2.y = pt_orig.y - CYLINDER_LENGTH * plane_b;
                pt_cyl2.z = pt_orig.z - CYLINDER_LENGTH * plane_c;

                for (int i = 0; i <= cloud_p->points.size(); i++)
                {
                    if (point_is_in_cylinder(cloud_p->points[i], pt_cyl1, pt_cyl2, CYLINDER_RADIUS))
                    {
                        plane_count++;
                    }
                }
                for (int i = 0; i <= cloud_obj->points.size(); i++)
                {
                    if (point_is_in_cylinder(cloud_obj->points[i], pt_cyl1, pt_cyl2, CYLINDER_RADIUS))
                    {
                        obj_count++;
                    }
                }
                //Best fit check
                if (obj_count < 5 && plane_count > bf_plane_count)
                {
                    bf_index = randomPointIndex;
                    bf_plane_count = plane_count;
                    bf_obj_count = obj_count;
                }
            }

            //Lift point higher
            pt_approach.x = cloud_p->points[bf_index].x + 0.1 * plane_a;
            pt_approach.y = cloud_p->points[bf_index].y + 0.1 * plane_b;
            pt_approach.z = cloud_p->points[bf_index].z + 0.1 * plane_c;

            //Convert point to robot coordinates
            pt_robot.x = pt_approach.y * 1000;
            pt_robot.y = (-pt_approach.x - 0.075) * 1000;
            pt_robot.z = (pt_approach.z - 0.096) * 1000;

            coordinates_pub_.publish(pt_robot);
            robotFlag = false;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "surface_detection_final");
    SurfaceDetection surfacedetection;
    while(1){ros::spinOnce();}
    return 0;
}