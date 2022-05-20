#include "../include/surface_detection_improved.h"

//int true_hole = 0, false_hole = 0, true_full = 0, false_full = 0, frame_count = 0;

//TESTING HOLE PARAMS
#define CYL_PER_CIRCLE 18
#define CIRCLE_RADIUS 0.02
#define RATIO_MIN 0.1
#define RATIO_MAX 1.0
#define RATIO_CHANGE 0.1
#define HOLE_THRESHOLD 5

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
    pcl::PointXYZ vec2; vec2.x = pt.x - pt_cyl1.x; vec2.y = pt.y - pt_cyl1.y; vec2 .z = pt.z - pt_cyl1.z;
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

double SurfaceDetection::degToRad(double degree)
{
    double pi = 3.14159265359;
    return (degree * (pi / 180));
}

geometry_msgs::Point SurfaceDetection::crossProduct(geometry_msgs::Point A, geometry_msgs::Point B)
{
    geometry_msgs::Point C;
    C.x = A.y*B.z-A.z*B.y;
    C.y = A.z*B.x-A.x*B.z;
    C.z = A.x*B.y-A.y*B.x;
    return C;
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
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_circ(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ring(new pcl::PointCloud<pcl::PointXYZ>);

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
            geometry_msgs::Point pt_robot, V_Norm;

            V_Norm.x = coefficients->values[0] / coefficients->values[3];
            V_Norm.y = coefficients->values[1] / coefficients->values[3];
            V_Norm.z = coefficients->values[2] / coefficients->values[3];

            for (int x = 0; x < 2000; x++)
            {
                plane_count = obj_count = 0;
                int randomPointIndex = dis(gen);
                pt_orig = cloud_p->points[randomPointIndex], pt_cyl1, pt_cyl2;

                pt_cyl1.x = pt_orig.x + CYLINDER_LENGTH * V_Norm.x;
                pt_cyl1.y = pt_orig.y + CYLINDER_LENGTH * V_Norm.y;
                pt_cyl1.z = pt_orig.z + CYLINDER_LENGTH * V_Norm.z;
                pt_cyl2.x = pt_orig.x - CYLINDER_LENGTH * V_Norm.x;
                pt_cyl2.y = pt_orig.y - CYLINDER_LENGTH * V_Norm.y;
                pt_cyl2.z = pt_orig.z - CYLINDER_LENGTH * V_Norm.z;

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

            //Calculating orthogonal vectors for the plane
            //Cross product of normal vector with a standard basis vector to produce vector perpendicular to normal
            geometry_msgs::Point V_NormN, V_B, V_C, V_NormB, V_NormC;
            V_NormN.x = abs(V_Norm.x), V_NormN.y = abs(V_Norm.y), V_NormN.z = abs(V_Norm.z);
            if(V_NormN.x < V_NormN.y)
            {
               if(V_NormN.x < V_NormN.z)
               {
                   V_B.x = 0; V_B.y = -V_Norm.z; V_B.z = V_Norm.y;
               }
               else
               {
                   V_B.x = -V_Norm.y; V_B.y = V_Norm.x; V_B.z = 0;
               }
            }
            else
            {
                if(V_NormN.y < V_NormN.z)
                {
                    V_B.x = V_Norm.z; V_B.y = 0; V_B.z = -V_Norm.x;
                }
                else
                {
                    V_B.x = -V_Norm.y; V_B.y = V_Norm.x; V_B.z = 0;
                }
            }
            //Compute cross-product of the new vector with the normal in order to produce another vector V_C ortogonal to V_B
            V_C = crossProduct(V_Norm, V_B);
            //Normalize new vectors
            float mod1 = V_B.x*V_B.x + V_B.y*V_B.y + V_B.z*V_B.z, mod2 = V_C.x*V_C.x + V_C.y*V_C.y + V_C.z*V_C.z;
            float mag1 = std::sqrt(mod1), mag2 = std::sqrt(mod2);
            if (mag1 == 0 || mag2 == 0) {
                throw std::logic_error("The input vector is a zero vector");
            }
            V_NormB.x = V_B.x / mag1; V_NormB.y = V_B.y / mag1; V_NormB.z = V_B.z / mag1;
            V_NormC.x = V_C.x / mag2; V_NormC.y = V_C.y / mag2; V_NormC.z = V_C.z / mag2;

            //Analysis of hole point density
            std::vector<unsigned int> local_plane_count_analysis;
            unsigned int smallest_plane_count = 9999, sum_plane_count = 0;
            //Switch hole flag off
            holeFlag = false;
            //New additions of checking density for inside cylinders
            pcl::PointXYZ pts_circ_orig, pts_circ_cyl1[CYL_PER_CIRCLE], pts_circ_cyl2[CYL_PER_CIRCLE];
            unsigned int local_plane_count;
            for(float r_ratio = RATIO_MAX; r_ratio > RATIO_MIN; r_ratio -= RATIO_CHANGE)
            {
                for (int i = 0; i < CYL_PER_CIRCLE; i++)
                {
                    local_plane_count = 0;
                    double angle = 36.0 / CYL_PER_CIRCLE * i * 10.0;
                    //Calculate coordinates of point lying on circumference of circle with radius r and angle
                    pts_circ_orig.x = cloud_p->points[bf_index].x + CYLINDER_RADIUS * r_ratio *
                                                                    (cos(degToRad(angle)) * V_NormB.x +
                                                                     sin(degToRad(angle)) * V_NormC.x);
                    pts_circ_orig.y = cloud_p->points[bf_index].y + CYLINDER_RADIUS * r_ratio *
                                                                    (cos(degToRad(angle)) * V_NormB.y +
                                                                     sin(degToRad(angle)) * V_NormC.y);
                    pts_circ_orig.z = cloud_p->points[bf_index].z + CYLINDER_RADIUS * r_ratio *
                                                                    (cos(degToRad(angle)) * V_NormB.z +
                                                                     sin(degToRad(angle)) * V_NormC.z);

                    //New cylinder points
                    pts_circ_cyl1[i].x = pts_circ_orig.x + CYLINDER_LENGTH * V_Norm.x;
                    pts_circ_cyl1[i].y = pts_circ_orig.y + CYLINDER_LENGTH * V_Norm.y;
                    pts_circ_cyl1[i].z = pts_circ_orig.z + CYLINDER_LENGTH * V_Norm.z;
                    pts_circ_cyl2[i].x = pts_circ_orig.x - CYLINDER_LENGTH * V_Norm.x;
                    pts_circ_cyl2[i].y = pts_circ_orig.y - CYLINDER_LENGTH * V_Norm.y;
                    pts_circ_cyl2[i].z = pts_circ_orig.z - CYLINDER_LENGTH * V_Norm.z;

                    for (int j = 0; j <= cloud_p->points.size(); j++)
                    {
                        if (point_is_in_cylinder(cloud_p->points[j], pts_circ_cyl1[i], pts_circ_cyl2[i], CIRCLE_RADIUS))
                        {
                            //cloud_circ->push_back(cloud_p->points[j]);
                            local_plane_count++;
                        }
                    }
                    std::cerr<<"PC: "<< local_plane_count << std::endl;
                    //Plane count analysis
                    local_plane_count_analysis.push_back(local_plane_count);
                    if(local_plane_count < smallest_plane_count)
                        smallest_plane_count = local_plane_count;
                    if (local_plane_count< HOLE_THRESHOLD)
                        holeFlag = true;
                }
            }

            for(int i = 0; i<local_plane_count_analysis.size(); i++)
            {
                sum_plane_count+=local_plane_count_analysis[i];
            }
            std:cerr<<"Plane count avg: "<< sum_plane_count/local_plane_count_analysis.size() << "  Smallest plane count: " << smallest_plane_count << std::endl;
            //Ring display for analysis
            for (int i = 0; i < 36; i++){
                    double angle = i * 10;
                    //Calculate coordinates of point lying on circumference of circle with radius r and angle
                    pts_circ_orig.x = cloud_p->points[bf_index].x + CYLINDER_RADIUS*
                                                                    (cos(degToRad(angle)) * V_NormB.x +
                                                                     sin(degToRad(angle)) * V_NormC.x);
                    pts_circ_orig.y = cloud_p->points[bf_index].y + CYLINDER_RADIUS*
                                                                    (cos(degToRad(angle)) * V_NormB.y +
                                                                     sin(degToRad(angle)) * V_NormC.y);
                    pts_circ_orig.z = cloud_p->points[bf_index].z + CYLINDER_RADIUS*
                                                                    (cos(degToRad(angle)) * V_NormB.z +
                                                                     sin(degToRad(angle)) * V_NormC.z);
                    cloud_ring->push_back(pts_circ_orig);
                }
            //Display pointcloud
            unsigned int h_r, h_g, h_b; // colors
            viewer->removePointCloud("cloud_p");
            //viewer->removePointCloud("cloud_cylinder");
            viewer->removePointCloud("cloud_object");
            viewer->removePointCloud("cloud_ring");
            //viewer->removePointCloud("cloud_circ");
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_p(cloud_p, 0, 255, 0);
            //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cylinder(cloud_cylinder, 0, 0, 255);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_obj(cloud_obj, 255, 0, 0);
            if(holeFlag)
            {
                std::cerr<<"Hole\n";
                h_r = 0;
                h_g = 255;
                h_b = 255;
            }
            else
            {
                std::cerr<<"Full\n";
                h_r = 255;
                h_g = 255;
                h_b = 0;
            }

            /*
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
            */

            //Visualization of pointlouds
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_ring(cloud_ring, h_r, h_g, h_b);
            viewer->addPointCloud<pcl::PointXYZ>(cloud_p, color_p, "cloud_p");
            viewer->addPointCloud<pcl::PointXYZ>(cloud_obj, color_obj, "cloud_object");
            viewer->addPointCloud<pcl::PointXYZ>(cloud_ring, color_ring, "cloud_ring");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_p");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_object");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_ring");

            /*
            frame_count++;
            //Analysis keyboard reading
            if(frame_count > 20)
            {
                char read;
                cin >> read;
                switch (read)
                {
                    case 'a':
                        true_hole++;
                        break;
                    case 's':
                        false_hole++;
                        break;
                    case 'd':
                        true_full++;
                        break;
                    case 'f':
                        false_full++;
                        break;
                    case 'p':
                    {
                        std::cerr << "True hole: " << true_hole << "\nFalse hole: " << false_hole << "\n True full: "
                                  << true_full << "\n False full: " << false_full << std::endl;
                        exit(1);
                    }
                    default:
                        break;
                }
            }
             */
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "surface_detection_final");
    SurfaceDetection surfacedetection;

    // Create PCL visualizer
    viewer = new pcl::visualization::PCLVisualizer("3d viewer");
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    // Spin
    while(!viewer->wasStopped()) {
        ros::spinOnce();
        viewer->spinOnce();
    }

    return 0;
}