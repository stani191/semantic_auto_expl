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

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
    double alpha = 0.10; // 0.04
    float radius = 0.005;
    double door_width_min = 0.65;
    double door_width_max = 1.1;
    double door_height = 1.95;
    double kinect_height = 0.36;
    double stripe_height = 0.5;
    double stripe_noise = 0.007;
    int max_noise = 1000;

    double ransac_eps_angle = 0.1;
    double ransac_distance_threshold = 0.04;
    double ransac_max_iterations = 30;

    struct timeval tp;
    int door_id_counter = 0;

    bool finished = true;
    std::string node_name;
    ros::NodeHandle n;
    ros::Subscriber input_cloud_sub;
    ros::Publisher map_pub;

};

#endif // OPENDOORRECOGNITIONAPP_H
