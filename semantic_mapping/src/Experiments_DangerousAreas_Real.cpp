/**
 * Experiments.cpp
 *
 * This class implements methods for evaluation of the implemented algorithms.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 12.11.2017
 */

#include <iostream>
#include <dirent.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>

#include "semantic_mapping/DangerousAreaRecognition.h"

void initParams(DangerousAreaRecognition d){

}


int testDangerousAreaRecognitionOnDB_Real_All(std::string path, DangerousAreaRecognition d){
}

int testDangerousAreaRecognitionOnDB_Debug_Yes(std::string path, DangerousAreaRecognition d){
    ROS_INFO("Starting dangerous area detection on real database - DEBUG");
    d.setDebugFlag();
    d.setKinectTilt(0.50);
    sensor_msgs::PointCloud2 input_cloud;
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    if (pcl::io::loadPCDFile<PointXYZ> (path + "yes_0.5//pcd_stair_32.pcd", *cloud) == -1){
       PCL_ERROR ("Couldn't read file\n");
       return (-1);
    }
    toROSMsg(*cloud, input_cloud);
    sensor_msgs::PointCloud2ConstPtr input_cloud_ptr(new sensor_msgs::PointCloud2(input_cloud));
    d.cloudCallback(input_cloud_ptr);
}

int testDangerousAreaRecognitionOnDB_Real_Yes(std::string path, DangerousAreaRecognition d){
    ROS_INFO("Starting dangerous area detection on real database - YES");
    std::vector<std::string> tilt_list = {"0", "0.25", "0.5", "0.75", "1"};
    for (std::string tilt : tilt_list){
        ROS_ERROR("TEST: Tilt = %f", std::atof(tilt.c_str()));
        std::clock_t begin = clock();
        d.setKinectTilt(std::atof(tilt.c_str()));
        for (int i = 1; i < 7; i++){
            sensor_msgs::PointCloud2 input_cloud;
            PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
            std::string filename = path + "yes_" + tilt + "//yes_" + tilt + "_" + std::to_string(i) + ".pcd";
            if (pcl::io::loadPCDFile<PointXYZ> (filename, *cloud) == -1){
               ROS_ERROR("Couldn't read file %s", filename.c_str());
               return (-1);
            }
            ROS_INFO("Processing %s", filename.c_str());
            toROSMsg(*cloud, input_cloud);
            sensor_msgs::PointCloud2ConstPtr input_cloud_ptr(new sensor_msgs::PointCloud2(input_cloud));
            d.cloudCallback(input_cloud_ptr);
        }
        std::clock_t end = clock();
        double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
        ROS_WARN("Elapsed time in ms: %f", elapsed_ms);
        ROS_WARN("---");
    }
}

int testDangerousAreaRecognitionOnDB_Real_No(std::string path, DangerousAreaRecognition d){
    ROS_INFO("Starting dangerous area detection on real database - NO");
    std::vector<std::string> tilt_list = {"0", "0.25", "0.5", "0.75", "1"};
    for (std::string tilt : tilt_list){
        ROS_ERROR("TEST: Tilt = %f", std::atof(tilt.c_str()));
        std::clock_t begin = clock();
        d.setKinectTilt(std::atof(tilt.c_str()));
        for (int i = 1; i < 7; i++){
            sensor_msgs::PointCloud2 input_cloud;
            PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
            std::string filename = path + "no_" + tilt + "//no_" + tilt + "_" + std::to_string(i) + ".pcd";
            if (pcl::io::loadPCDFile<PointXYZ> (filename, *cloud) == -1){
               ROS_ERROR("Couldn't read file %s", filename.c_str());
               return (-1);
            }
            ROS_INFO("Processing %s", filename.c_str());
            toROSMsg(*cloud, input_cloud);
            sensor_msgs::PointCloud2ConstPtr input_cloud_ptr(new sensor_msgs::PointCloud2(input_cloud));
            d.cloudCallback(input_cloud_ptr);
        }
        std::clock_t end = clock();
        double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
        ROS_WARN("Elapsed time in ms: %f", elapsed_ms);
        ROS_WARN("---");
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "experiments_dangerous_areas_real");
    std::string stairspath = "//home//stanic//master_thesis//catkin_workspace//src//semantic_auto_expl//semantic_mapping//pcd//stairs_real//";
    DangerousAreaRecognition d;
    if (argc > 1 && argv[1] == "debug"){
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
        testDangerousAreaRecognitionOnDB_Debug_Yes(stairspath, d);
    } else {
        //ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
        //testDangerousAreaRecognitionOnDB_Debug_Yes(stairspath, d);
        testDangerousAreaRecognitionOnDB_Real_No(stairspath, d);
        testDangerousAreaRecognitionOnDB_Real_Yes(stairspath, d);
        //testOpenDoorRecognitionOnDB_Sim_All(stairspath, d);
    }
}

