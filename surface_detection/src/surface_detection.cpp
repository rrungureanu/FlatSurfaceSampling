#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
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
#include <random>

ros::Publisher pub;
pcl::visualization::PCLVisualizer* viewer;

std::ofstream file;

int prev_planePoints, prev_objPoints;

bool point_is_in_cylinder(pcl::PointXYZ pt, pcl::PointXYZ pt_cyl1, pcl::PointXYZ pt_cyl2, double r)
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

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    ROS_INFO("Cloud cb");
    pcl::ScopeTime scopeTimeAll("Time of callback");
    {
        // Create a container for the data.
        sensor_msgs::PointCloud2 output;
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
        /*
        //Circle segmentation processing start

        pcl::ModelCoefficients::Ptr coefficients_c(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_c(new pcl::PointIndices);
        // Create the circle segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg_c;
        // Optional
        seg_c.setOptimizeCoefficients(true);
        // Mandatory
        seg_c.setModelType(pcl::SACMODEL_CIRCLE2D);
        seg_c.setMethodType(pcl::SAC_RANSAC);
        seg_c.setDistanceThreshold(0.01);
        seg_c.setRadiusLimits(0.1, 0.15);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract_c;

        // Segment the largest planar component from the remaining cloud
        seg_c.setInputCloud(cloud_p);
            seg_c.segment(*inliers_c, *coefficients_c);
        if (inliers_c->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_p);
        extract.setIndices(inliers_c);
        extract.setNegative(false);
        extract.filter(*cloud_c);
        */
        /*
        //PERPENDICULAR TO PLANE EXPERIMENTATION
        pcl::PointXYZ pnt_on_line, initial_pnt;
        initial_pnt.x = coefficients->values[0];
        initial_pnt.y = coefficients->values[1];
        initial_pnt.z = coefficients->values[2];
        const auto plane_a = coefficients->values[0]/coefficients->values[3];
        const auto plane_b = coefficients->values[1]/coefficients->values[3];
        const auto plane_c = coefficients->values[2]/coefficients->values[3];

        double LINE_LENGTH = 100, DISTANCE_INCREMENT = 0.01;

        for(double distfromstart=0.0;distfromstart<LINE_LENGTH;distfromstart+=DISTANCE_INCREMENT){
            pnt_on_line.x = initial_pnt.x + distfromstart*plane_a;
            pnt_on_line.y = initial_pnt.y + distfromstart*plane_b;
            pnt_on_line.z = initial_pnt.z + distfromstart*plane_c;
            cloud_p->points.push_back(pnt_on_line);
        }

        LINE_LENGTH = -100;

        for(double distfromstart=0.0;distfromstart>LINE_LENGTH;distfromstart-=DISTANCE_INCREMENT){
            pnt_on_line.x = initial_pnt.x + distfromstart*plane_a;
            pnt_on_line.y = initial_pnt.y + distfromstart*plane_b;
            pnt_on_line.z = initial_pnt.z + distfromstart*plane_c;
            cloud_p->points.push_back(pnt_on_line);
        }
        */

    /*

            for (int j = 0; j < 100; j++)
            {
                cloud_cylinder->points.clear();
                //Cylinder experimentation, checking if all points somewhere lie in cylinder
                srand(time(NULL));
                int randomPointIndex = rand() % cloud_p->points.size(), CYLINDER_LENGTH = 20;
                double CYLINDER_RADIUS = 0.1;
                pcl::PointXYZ pt_orig = cloud_p->points[randomPointIndex], pt_cyl1, pt_cyl2;

                const auto plane_a = coefficients->values[0] / coefficients->values[3];
                const auto plane_b = coefficients->values[1] / coefficients->values[3];
                const auto plane_c = coefficients->values[2] / coefficients->values[3];

                pt_cyl1.x = pt_orig.x + CYLINDER_LENGTH * plane_a;
                pt_cyl1.y = pt_orig.y + CYLINDER_LENGTH * plane_b;
                pt_cyl1.z = pt_orig.z + CYLINDER_LENGTH * plane_c;
                pt_cyl2.x = pt_orig.x - CYLINDER_LENGTH * plane_a;
                pt_cyl2.y = pt_orig.y - CYLINDER_LENGTH * plane_b;
                pt_cyl2.z = pt_orig.z - CYLINDER_LENGTH * plane_c;

                for (int i = 0; i <= cloud->points.size(); i++)
                {
                    if (point_is_in_cylinder(cloud->points[i], pt_cyl1, pt_cyl2, CYLINDER_RADIUS))
                        cloud_cylinder->push_back(cloud->points[i]);
                }
            }

        */
        /*
        //Selective cylinder selection for determining thresholds
        srand(time(NULL));
        int randomPointIndex = rand() % cloud_p->points.size(), CYLINDER_LENGTH = 20, plane_count = 0, obj_count = 0;
        double CYLINDER_RADIUS = 0.1;
        pcl::PointXYZ pt_orig = cloud_p->points[randomPointIndex], pt_cyl1, pt_cyl2;
        //pcl::PointXYZ pt_orig = cloud_p->points[cloud->points.size()/2], pt_cyl1, pt_cyl2;

        const auto plane_a = coefficients->values[0] / coefficients->values[3];
        const auto plane_b = coefficients->values[1] / coefficients->values[3];
        const auto plane_c = coefficients->values[2] / coefficients->values[3];

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
                cloud_cylinder->push_back(cloud_p->points[i]);
            }
        }
        for (int i = 0; i <= cloud_obj->points.size(); i++)
        {
            if (point_is_in_cylinder(cloud_obj->points[i], pt_cyl1, pt_cyl2, CYLINDER_RADIUS))
            {
                obj_count++;
                cloud_cylinder->push_back(cloud_obj->points[i]);
            }
        }


        std::cerr << "Plane points: " << prev_planePoints << " Obj points: " << prev_objPoints << std::endl;
        prev_planePoints = plane_count; prev_objPoints = obj_count;
    */
        //NEWEST APPROACH choose a random cylinder X  times, end up with the best fit (most plane points, least object points) and display that
        // Random seed
        std::random_device rd;

        // Initialize Mersenne Twister pseudo-random number generator
        std::mt19937 gen(rd());

        // Generate pseudo-random numbers
        // uniformly distributed in range (1, 100)
        std::uniform_int_distribution<> dis(0, cloud_p->points.size());

        int bf_index = -1, bf_plane_count = -1, bf_obj_count = 9999999, plane_count = 0, obj_count = 0, CYLINDER_LENGTH = 20;
        double CYLINDER_RADIUS = 0.1;
        pcl::PointXYZ pt_orig, pt_cyl1, pt_cyl2, pt_approach, pt_robot;
        for(int x = 0; x < 1000; x++)
        {
            plane_count = obj_count = 0;
            int randomPointIndex = dis(gen);
            std::cerr << "!" << randomPointIndex << "! ";
            pt_orig = cloud_p->points[randomPointIndex], pt_cyl1, pt_cyl2;
            //pcl::PointXYZ pt_orig = cloud_p->points[cloud->points.size()/2], pt_cyl1, pt_cyl2;

            const auto plane_a = coefficients->values[0] / coefficients->values[3];
            const auto plane_b = coefficients->values[1] / coefficients->values[3];
            const auto plane_c = coefficients->values[2] / coefficients->values[3];

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
            if( obj_count < 10 && plane_count > bf_plane_count)
            {
                bf_index = randomPointIndex;
                bf_plane_count = plane_count;
                bf_obj_count = obj_count;
            }
        }


        plane_count = obj_count = 0;
        pt_orig = cloud_p->points[bf_index];
        const auto plane_a = coefficients->values[0] / coefficients->values[3];
        const auto plane_b = coefficients->values[1] / coefficients->values[3];
        const auto plane_c = coefficients->values[2] / coefficients->values[3];

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
                cloud_cylinder->push_back(cloud_p->points[i]);
            }
        }
        for (int i = 0; i <= cloud_obj->points.size(); i++)
        {
            if (point_is_in_cylinder(cloud_obj->points[i], pt_cyl1, pt_cyl2, CYLINDER_RADIUS))
            {
                obj_count++;
                cloud_cylinder->push_back(cloud_obj->points[i]);
            }
        }

        //Lift point higher
        pt_approach.x = cloud_p->points[bf_index].x + 0.1 * plane_a;
        pt_approach.y = cloud_p->points[bf_index].y + 0.1 * plane_b;
        pt_approach.z = cloud_p->points[bf_index].z + 0.1 * plane_c;

        pt_robot.x = pt_approach.y;
        pt_robot.y = -pt_approach.x - 0.075;
        pt_robot.z = pt_approach.z - 0.096;

        std::cerr << std::endl << pt_robot.x * 1000<< " " << pt_robot.y * 1000 << " " << pt_robot.z * 1000<< " " << std::endl;
        std::cerr << std::endl << "Plane points: " << bf_plane_count << " Obj points: " << bf_obj_count << std::endl;
        prev_planePoints = plane_count; prev_objPoints = obj_count;
        /*
       //Estimate normals
       // Create the normal estimation class, and pass the input dataset to it
       pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
       ne.setInputCloud (cloud_p);

       // Create an empty kdtree representation, and pass it to the normal estimation object.
       // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
       pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
       ne.setSearchMethod (tree);

       // Output datasets
       pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

       // Use all neighbors in a sphere of radius 3cm
       ne.setRadiusSearch (0.05);

       // Compute the features
       ne.compute (*cloud_normals);

       for(int i = 0; i < cloud_normals->points.size(); i++)
       {
           if(!isnan(cloud_normals->points[i].curvature))
           std::cerr<<cloud_normals->points[i].curvature << std::endl;
       }

        */
        //Display pointcloud
        viewer->removePointCloud("cloud_p");
        viewer->removePointCloud("cloud_cylinder");
        viewer->removePointCloud("cloud_object");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_p(cloud_p, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cylinder(cloud_cylinder, 0, 0, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_obj(cloud_obj, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_p, color_p, "cloud_p");
        viewer->addPointCloud<pcl::PointXYZ>(cloud_cylinder, color_cylinder, "cloud_cylinder");
        viewer->addPointCloud<pcl::PointXYZ>(cloud_obj, color_obj, "cloud_object");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_p");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_cylinder");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_object");

        int input = 2;
        //cin>>input;
        if(input == 1)
            file << "1," << prev_planePoints << ',' << prev_objPoints << '\n';
        else if(input == 0)
            file << "0," << prev_planePoints << ',' << prev_objPoints << '\n';
    }
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "surface_detection");
    ros::NodeHandle nh;

    // Initialize csv file
    file.open("/home/silverback/PointsData.csv");

    // Create PCL visualizer
    viewer = new pcl::visualization::PCLVisualizer("3d viewer");
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    while(!viewer->wasStopped()) {
        ros::spinOnce();
        viewer->spinOnce();
    }
    delete(viewer);
    file.close();
}
