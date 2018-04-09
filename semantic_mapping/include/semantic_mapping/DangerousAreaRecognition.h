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

    void setDebugFlag();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (PointCloud<PointXYZ>::ConstPtr cloud0, PointCloud<PointXYZ>::ConstPtr cloud1, PointCloud<PointXYZ>::ConstPtr cloud2, PointCloud<PointXYZ>::ConstPtr cloud3, PointCloud<PointXYZ>::ConstPtr cloud4);
    void setKinectHeight(double d);
    void setKinectHeightOffset(double d);
    void setKinectTilt(double d);

private:
    struct timeval tp;

    bool finished = true;
    double kinect_height = -0.72; //-0.36 for sim
    double kinect_height_offset = -0.03; // -0.06 for sim
    double kinect_tilt = 0.0;
    std::string node_name;
    ros::NodeHandle n;
    ros::Subscriber input_cloud_sub;
    ros::Publisher dangerous_pub;

    bool debug = true;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

#endif // DANGEROUSAREARECOGNITION_H
