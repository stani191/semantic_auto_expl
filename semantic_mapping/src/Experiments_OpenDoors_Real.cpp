/**
 * Experiments.cpp
 *
 * This class implements methods for evaluation of the implemented algorithms.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 22.06.2017
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

#include "semantic_mapping/OpenDoorRecognitionApp.h"

void initParams(OpenDoorRecognitionApp o){
    o.setRansacEpsAngle(0.2);
    o.setRansacDistanceThreshold(0.04);
    o.setRansacMaxIterations(30);
    o.setAlpha(0.10);
    o.setDoorWidthMin(0.65);
    o.setDoorWidthMax(1.3);
    o.setDoorHeight(-1.95);
    o.setKinectHeight(-0.36);
    o.setStripeHeight(0);
    o.setStripeNoise(0.007);
    o.setCubeDistance(0.05);
    o.setMaxNoise(1000);
}

int testOpenDoorRecognitionOnDB_Debug_Yes(std::string path, OpenDoorRecognitionApp o){
    ROS_INFO("Starting door recognition on real database - DEBUG");
    o.setDebugFlag();
    sensor_msgs::PointCloud2 input_cloud;
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    if (pcl::io::loadPCDFile<PointXYZ> (path + "pcd_open_door_2.pcd", *cloud) == -1){
       PCL_ERROR ("Couldn't read file\n");
       return (-1);
    }
    toROSMsg(*cloud, input_cloud);
    sensor_msgs::PointCloud2ConstPtr input_cloud_ptr(new sensor_msgs::PointCloud2(input_cloud));
    o.cloudCallback(input_cloud_ptr);
}

int testOpenDoorRecognitionOnDB_Real_Yes(std::string path, OpenDoorRecognitionApp o){
    ROS_INFO("Starting door recognition on real database - positives");

    sensor_msgs::PointCloud2 input_cloud;
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

    std::vector<std::string> doorfiles;
    for (int i = 0; i < 26; i++){
        std::string filename = path + "pcd_open_door_" + std::to_string(i) + ".pcd";
        doorfiles.push_back(filename);
    }
    int correct = 0;
    int i = 0;
    for(std::string file : doorfiles){
        if (pcl::io::loadPCDFile<PointXYZ> (file, *cloud) == -1){
           PCL_ERROR ("Couldn't read file\n");
           return (-1);
        }
        toROSMsg(*cloud, input_cloud);
        sensor_msgs::PointCloud2ConstPtr input_cloud_ptr(new sensor_msgs::PointCloud2(input_cloud));
        ROS_INFO("Processing %s", file.c_str());
        o.cloudCallback(input_cloud_ptr);
        if(o.getLastCorrect()){
            correct++;
        }
        i++;
    }
    ROS_WARN("[Positives] Found %d of %d open doors correctly", correct, i);
}

int testOpenDoorRecognitionOnDB_Real_No(std::string path, OpenDoorRecognitionApp o){
    ROS_INFO("Starting door recognition on real database - negatives");
    sensor_msgs::PointCloud2 input_cloud;
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

    std::vector<std::string> doorfiles;
    for (int i = 0; i < 26; i++){
        std::string filename = path + "pcd_no_open_door_" + std::to_string(i) + ".pcd";
        doorfiles.push_back(filename);
    }
    int correct = 0;
    int i = 0;
    for(std::string file : doorfiles){
        if (pcl::io::loadPCDFile<PointXYZ> (file, *cloud) == -1){
           PCL_ERROR ("Couldn't read file\n");
           return (-1);
        }
        toROSMsg(*cloud, input_cloud);
        sensor_msgs::PointCloud2ConstPtr input_cloud_ptr(new sensor_msgs::PointCloud2(input_cloud));
        ROS_INFO("Processing %s", file.c_str());
        o.cloudCallback(input_cloud_ptr);
        if(o.getLastCorrect()){
            correct++;
        }
        i++;
    }
    ROS_WARN("[Negatives] Found %d of %d open doors correctly", i-correct, i);
}

int testOpenDoorRecognitionOnDB_Real_RansacEpsAngle(std::string doorspath_real_yes,std::string doorspath_real_no, OpenDoorRecognitionApp o){
    std::vector<double> ransacEpsAngles{0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45};
    initParams(o);
    for (double angle : ransacEpsAngles){
        ROS_ERROR("TEST: Ransac EPS Angle = %f", angle);
        o.setRansacEpsAngle(angle);
        std::clock_t begin = clock();
        testOpenDoorRecognitionOnDB_Real_Yes(doorspath_real_yes, o);
        testOpenDoorRecognitionOnDB_Real_No(doorspath_real_no, o);
        std::clock_t end = clock();
        double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
        ROS_WARN("Elapsed time in ms: %f", elapsed_ms);
        ROS_WARN("---");
    }
}

int testOpenDoorRecognitionOnDB_Real_RansacDistanceThreshold(std::string doorspath_real_yes,std::string doorspath_real_no, OpenDoorRecognitionApp o){
    std::vector<double> ransacDistanceThresholds{0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06};
    initParams(o);
    for (double thresh : ransacDistanceThresholds){
        o.setRansacDistanceThreshold(thresh);
        ROS_ERROR("TEST: Ransac Distance Threshold = %f", thresh);
        std::clock_t begin = clock();
        testOpenDoorRecognitionOnDB_Real_Yes(doorspath_real_yes, o);
        testOpenDoorRecognitionOnDB_Real_No(doorspath_real_no, o);
        std::clock_t end = clock();
        double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
        ROS_WARN("Elapsed time in ms: %f", elapsed_ms);
        ROS_WARN("---");
    }
}

int testOpenDoorRecognitionOnDB_Real_RansacMaxIterations(std::string doorspath_real_yes,std::string doorspath_real_no, OpenDoorRecognitionApp o){
    std::vector<double> ransacMaxIterationsList{1, 10, 31, 100, 316, 1000, 3162};
    initParams(o);
    for (double maxIter : ransacMaxIterationsList){
        o.setRansacMaxIterations(maxIter);
        ROS_ERROR("TEST: Ransac Max Iterations = %f", maxIter);
        std::clock_t begin = clock();
        testOpenDoorRecognitionOnDB_Real_Yes(doorspath_real_yes, o);
        testOpenDoorRecognitionOnDB_Real_No(doorspath_real_no, o);
        std::clock_t end = clock();
        double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
        ROS_WARN("Elapsed time in ms: %f", elapsed_ms);
        ROS_WARN("---");
    }
}

int testOpenDoorRecognitionOnDB_Real_Alpha(std::string doorspath_real_yes,std::string doorspath_real_no, OpenDoorRecognitionApp o){
    std::vector<double> alphaList{0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40};
    initParams(o);
    for (double alpha : alphaList){
        o.setAlpha(alpha);
        ROS_ERROR("TEST: Alpha = %f", alpha);
        std::clock_t begin = clock();
        testOpenDoorRecognitionOnDB_Real_Yes(doorspath_real_yes, o);
        testOpenDoorRecognitionOnDB_Real_No(doorspath_real_no, o);
        std::clock_t end = clock();
        double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
        ROS_WARN("Elapsed time in ms: %f", elapsed_ms);
        ROS_WARN("---");
    }
}

int testOpenDoorRecognitionOnDB_Real_StripeNoise(std::string doorspath_real_yes,std::string doorspath_real_no, OpenDoorRecognitionApp o){
    std::vector<double> stripeNoiseList{0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007, 0.008, 0.009, 0.01};
    initParams(o);
    for (double stripeNoise : stripeNoiseList){
        o.setStripeNoise(stripeNoise);
        std::clock_t begin = clock();
        ROS_ERROR("TEST: Stripe Noise = %f", stripeNoise);
        testOpenDoorRecognitionOnDB_Real_Yes(doorspath_real_yes, o);
        testOpenDoorRecognitionOnDB_Real_No(doorspath_real_no, o);
        std::clock_t end = clock();
        double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
        ROS_WARN("Elapsed time in ms: %f", elapsed_ms);
        ROS_WARN("---");
    }
}

int testOpenDoorRecognitionOnDB_Real_CubeDistance(std::string doorspath_real_yes,std::string doorspath_real_no, OpenDoorRecognitionApp o){
    std::vector<double> cubeDistanceList{0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07};
    initParams(o);
    for (double cubeDistance : cubeDistanceList){
        o.setCubeDistance(cubeDistance);
        ROS_ERROR("TEST: Cube Distance = %f", cubeDistance);
        std::clock_t begin = clock();
        testOpenDoorRecognitionOnDB_Real_Yes(doorspath_real_yes, o);
        testOpenDoorRecognitionOnDB_Real_No(doorspath_real_no, o);
        std::clock_t end = clock();
        double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
        ROS_WARN("Elapsed time in ms: %f", elapsed_ms);
        ROS_WARN("---");
    }
}

int testOpenDoorRecognitionOnDB_Real_MaxNoise(std::string doorspath_real_yes,std::string doorspath_real_no, OpenDoorRecognitionApp o){
    std::vector<int> maxNoiseList{0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000};
    initParams(o);
    for (int maxNoise : maxNoiseList){
        o.setMaxNoise(maxNoise);
        ROS_ERROR("TEST: Max Noise = %d", maxNoise);
        std::clock_t begin = clock();
        testOpenDoorRecognitionOnDB_Real_Yes(doorspath_real_yes, o);
        testOpenDoorRecognitionOnDB_Real_No(doorspath_real_no, o);
        std::clock_t end = clock();
        double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
        ROS_WARN("Elapsed time in ms: %f", elapsed_ms);
        ROS_WARN("---");
    }
}

int testOpenDoorRecognitionOnDB_Real_All(std::string doorspath_real_yes,std::string doorspath_real_no, OpenDoorRecognitionApp o){
    testOpenDoorRecognitionOnDB_Real_RansacEpsAngle(doorspath_real_yes, doorspath_real_no, o);
    testOpenDoorRecognitionOnDB_Real_RansacDistanceThreshold(doorspath_real_yes, doorspath_real_no, o);
    testOpenDoorRecognitionOnDB_Real_RansacMaxIterations(doorspath_real_yes, doorspath_real_no, o);
    testOpenDoorRecognitionOnDB_Real_Alpha(doorspath_real_yes, doorspath_real_no, o);
    testOpenDoorRecognitionOnDB_Real_StripeNoise(doorspath_real_yes, doorspath_real_no, o);
    testOpenDoorRecognitionOnDB_Real_CubeDistance(doorspath_real_yes, doorspath_real_no, o);
    testOpenDoorRecognitionOnDB_Real_MaxNoise(doorspath_real_yes, doorspath_real_no, o);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "experiments_open_door_real");
    std::string doorspath_real_yes = "//home//stanic//master_thesis//catkin_workspace//src//semantic_auto_expl//semantic_mapping//pcd//doors_real//open_doors//";
    std::string doorspath_real_no = "//home//stanic//master_thesis//catkin_workspace//src//semantic_auto_expl//semantic_mapping//pcd//doors_real//no_open_doors//";
    //std::string stairspath = ;
    OpenDoorRecognitionApp o;
    if (argc > 1 && argv[1] == "debug"){
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
        testOpenDoorRecognitionOnDB_Debug_Yes(doorspath_real_yes, o);
    } else {
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
        testOpenDoorRecognitionOnDB_Real_All(doorspath_real_yes, doorspath_real_no, o);
    }
    //testOpenDoorRecognitionOnDB_Real_Yes(doorspath_real_yes, o);
    //testOpenDoorRecognitionOnDB_Real_No(doorspath_real_no, o);
    //testDangerousAreaRecognitionOnDB(path);
}

