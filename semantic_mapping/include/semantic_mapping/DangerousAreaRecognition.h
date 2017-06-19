#ifndef DANGEROUSAREARECOGNITION_H
#define DANGEROUSAREARECOGNITION_H

/**
 * DangerousAreaRecognition.h
 * Starts the dangerous area recognition process.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 12.06.2017
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
#include <pcl/filters/passthrough.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <boost/thread/thread.hpp>

using namespace pcl;

class DangerousAreaRecognition{

public:
    DangerousAreaRecognition();
    ~DangerousAreaRecognition();

    void spin();

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
    struct timeval tp;

    bool finished = true;
    double kinect_height = -1;
    double kinect_height_offset = 0.05;
    std::string node_name;
    ros::NodeHandle n;
    ros::Subscriber input_cloud_sub;
    ros::Publisher dangerous_pub;
    tf::TransformListener listener;
};

#endif // DANGEROUSAREARECOGNITION_H
