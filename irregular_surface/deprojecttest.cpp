// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <iostream>
#include <fstream>
#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV AP

int main(int argc, char * argv[]) try
{
    //File for writing coordinates to simulation
    std::ofstream file;
    file.open("/home/silverback/realsensepath/SamplingPath.txt");
    int it = 0;
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();

    //Acquire intrinsics
    auto intrinsics = p.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    rs2_intrinsics* intrin = &intrinsics;

    //OpenCV setup
    using namespace cv;
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        //Align color to depth
        rs2::align align(RS2_STREAM_DEPTH);
        auto aligned_frames = align.process(frames);
        rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);

        // Get the color frame's dimensions
        float width = color_frame.get_width();
        float height = color_frame.get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(width, height), CV_8UC3, (void *) color_frame.get_data(), Mat::AUTO_STEP);

        //Creating parallel lines in a circle
        int radius = 40;
        int num_lines = 10;
        int angle = 0;
        float delta_y = (2.0 * radius) / (num_lines);
        float cos_a = cos(angle * M_PI / 180.0);
        float sin_a = sin(angle * M_PI / 180.0);

        //Create containers for lines and points
        std::vector<std::vector<cv::Point3f>> samplingPoints;

        for (int i = 0; i < num_lines; i++)
        {
            // applying pythagoras
            float y = delta_y * i - radius;
            float x = sqrt(radius * radius - y * y);

            // rotating the displacement vector
            float left_x = y * sin_a + x * cos_a;
            float right_x = y * sin_a - x * cos_a;
            float leftright_y = y * cos_a - x * sin_a;

            std::vector<cv::Point3f> samplingPoints_line;
            bool firstPoint = true;
            for(float j = width /2 + right_x;j < width /2 + left_x; j+=5){
                float dist_to_point = depth.get_distance(j, height / 2 +leftright_y);
                float point3d[3];
                float point2d[2];
                point2d[0] = j;
                point2d[1] = height / 2 +leftright_y;
                rs2_deproject_pixel_to_point(point3d, intrin, point2d, dist_to_point);
                // Print the distance
                //std::cout << "X: " << point3d[0] << " Y: " << point3d[1] << " Z: " << point3d[2] << "    1st\n";
                cv::Point3f samplingPoints_point; samplingPoints_point.x = point3d[0]; samplingPoints_point.y = point3d[1]; samplingPoints_point.z = point3d[2];
                // Add start approach point
                if(firstPoint){
                    cv::Point3f samplingPoints_pointApp; samplingPoints_pointApp.x = point3d[0]; samplingPoints_pointApp.y = point3d[1]; samplingPoints_pointApp.z = point3d[2]-0.08;
                    samplingPoints_line.push_back(samplingPoints_pointApp);
                    firstPoint = false;
                }
                samplingPoints_line.push_back(samplingPoints_point);
                //file << point3d[0] * 1000 << ", " << point3d[1] * 1000 << ", " << point3d[2] * 1000 << ", -90, 0, 0, " << it << '\n';
            }
            //Add end approach point
            if(!samplingPoints_line.empty()){
                const int linesize = samplingPoints_line.size() - 1;
                cv::Point3f samplingPoints_pointApp; samplingPoints_pointApp.x = samplingPoints_line[linesize].x; samplingPoints_pointApp.y = samplingPoints_line[linesize].y; samplingPoints_pointApp.z = samplingPoints_line[linesize].z - 0.08;
                samplingPoints_line.push_back(samplingPoints_pointApp);
            }
            //Add line of points to vector of lines
            samplingPoints.push_back(samplingPoints_line);

            line(image, Point(width / 2 + left_x, height / 2 + leftright_y),
                 Point(width / 2 + right_x, height / 2 + leftright_y), Scalar(0, 255, 0), 2);

        }

        //Creating perpendicular parallel lines
        angle = 90;
        delta_y = (2.0 * radius) / (num_lines);
        cos_a = cos(angle * M_PI / 180.0);
        sin_a = sin(angle * M_PI / 180.0);

        for (int i = 0; i < num_lines; i++)
        {
            // applying pythagoras
            float y = delta_y * i - radius;
            float x = sqrt(radius * radius - y * y);

            // rotating the displacement vector
            float leftright_x = y * sin_a + x * cos_a;
            float left_y = y * cos_a - x * sin_a;
            float right_y = y * cos_a + x * sin_a;

            std::vector<cv::Point3f> samplingPoints_line;
            bool firstPoint = true;
            for(float j = height /2 + left_y;j < height /2 + right_y; j+=5){
                float dist_to_point = depth.get_distance(width / 2 + leftright_x, j);
                float point3d[3];
                float point2d[2];
                point2d[0] = width / 2 + leftright_x;
                point2d[1] = j;
                rs2_deproject_pixel_to_point(point3d, intrin, point2d, dist_to_point);
                // Print the distance
                //std::cout << "X: " << point3d[0] << " Y: " << point3d[1] << " Z: " << point3d[2] << "    2nd\n";
                cv::Point3f samplingPoints_point; samplingPoints_point.x = point3d[0]; samplingPoints_point.y = point3d[1]; samplingPoints_point.z = point3d[2];
                // Add start approach point
                if(firstPoint){
                    cv::Point3f samplingPoints_pointApp; samplingPoints_pointApp.x = point3d[0]; samplingPoints_pointApp.y = point3d[1]; samplingPoints_pointApp.z = point3d[2]-0.08;
                    samplingPoints_line.push_back(samplingPoints_pointApp);
                    firstPoint = false;
                }
                samplingPoints_line.push_back(samplingPoints_point);
                //file << point3d[0] * 1000 << ", " << point3d[1] * 1000 << ", " << point3d[2] * 1000 << ", -90, 0, 0, " << it << '\n';
            }
            //Add end approach point
            if(!samplingPoints_line.empty()){
                const int linesize = samplingPoints_line.size() - 1;
                cv::Point3f samplingPoints_pointApp; samplingPoints_pointApp.x = samplingPoints_line[linesize].x; samplingPoints_pointApp.y = samplingPoints_line[linesize].y; samplingPoints_pointApp.z = samplingPoints_line[linesize].z - 0.08;
                samplingPoints_line.push_back(samplingPoints_pointApp);
            }
            //Add line of points to vector of lines
            samplingPoints.push_back(samplingPoints_line);

            line(image, Point(width / 2 + leftright_x, height / 2 + left_y),
                 Point(width / 2 + leftright_x, height / 2 + right_y), Scalar(0, 0, 255), 2);
        }
        //Writing vector to file
        it = 0;
        for(int i = 0; i < samplingPoints.size(); i++){
            std::cout<<"Line: "<<i<<std::endl;
            if(i%2==0){
                for(int j = 0; j < samplingPoints[i].size(); j++){
                    file << samplingPoints[i][j].x * 1000 << ", " << samplingPoints[i][j].y * 1000 << ", " << samplingPoints[i][j].z * 1000 << ", 0, 0, 0, " << it << '\n';
                    std::cout<<samplingPoints[i][j].x * 1000 << ", " << samplingPoints[i][j].y * 1000 << ", " << samplingPoints[i][j].z * 1000 << ", 0, 0, 0, " << it << '\n';
                    it++;
                }
            }
            else{
                for(int j = samplingPoints[i].size()-1; j >= 0; j--){
                    file << samplingPoints[i][j].x * 1000 << ", " << samplingPoints[i][j].y * 1000 << ", " << samplingPoints[i][j].z * 1000 << ", 0, 0, 0, " << it << '\n';
                    std::cout<<samplingPoints[i][j].x * 1000 << ", " << samplingPoints[i][j].y * 1000 << ", " << samplingPoints[i][j].z * 1000 << ", 0, 0, 0, " << it << '\n';
                    it++;
                }
            }
        }
        // Update the window with new data
        break;
        imshow(window_name, image);
    }
    file.close();
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}