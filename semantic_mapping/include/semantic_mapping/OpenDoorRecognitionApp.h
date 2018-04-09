#ifndef OPENDOORRECOGNITIONAPP_H
#define OPENDOORRECOGNITIONAPP_H

/**
 * OpenDoorRecognitionApp.h
 * Starts the open door recognition process.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include <iostream>
#include <ctime>

#include <sys/time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/crop_box.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <boost/thread/thread.hpp>

#include "semantic_mapping_msgs/DoorMessage.h"

using namespace pcl;

class OpenDoorRecognitionApp{

public:
    OpenDoorRecognitionApp();
    ~OpenDoorRecognitionApp();

    double euclideanDistance(const PointXYZ& p1, const PointXYZ& p2);
    void detectOpenDoors(PointCloud<PointXYZ>::Ptr cloud);
    void spin();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (PointCloud<PointXYZ>::ConstPtr cloud1, PointCloud<PointXYZ>::ConstPtr cloud2, PointCloud<PointXYZ>::ConstPtr cloud3, PointCloud<PointXYZ>::ConstPtr cloud4);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

    bool getLastCorrect();
    void setDebugFlag();
    void setRansacEpsAngle(double d);
    void setRansacDistanceThreshold(double d);
    void setRansacMaxIterations(double d);
    void setAlpha(double d);
    void setDoorWidthMin(double d);
    void setDoorWidthMax(double d);
    void setDoorHeight(double d);
    void setKinectHeight(double d);
    void setStripeHeight(double d);
    void setStripeNoise(double d);
    void setCubeDistance(double d);
    void setMaxNoise(int i);

private:
    // RANSAC Parameters
    //the maximum allowed difference between the model normal and the given axis in radians
    double ransac_eps_angle = 0.2;
    double ransac_distance_threshold = 0.04;
    double ransac_max_iterations = 30;

    // Other open door recognition algorithm parameters
    // Repeat open door recognition algorithm while percentage of unobserved cloud points > alpha
    double alpha = 0.10; // 0.04
    double door_width_min = 0.65;
    double door_width_max = 1.3;
    double door_height = -1.90;
    double kinect_height = -0.72; //-0.36 sim
    double stripe_height = 0;
    double stripe_noise = 0.007;
    double cube_distance = 0.05;
    int max_noise = 100; // 1000

    struct timeval tp;
    int door_id_counter = 0;

    bool finished = true;
    bool last_correct = false;
    std::string node_name;
    ros::NodeHandle n;
    ros::Subscriber input_cloud_sub;
    ros::Publisher map_pub;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    bool debug = false;
};

#endif // OPENDOORRECOGNITIONAPP_H
