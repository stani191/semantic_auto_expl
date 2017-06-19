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
#include <ctime>

using namespace pcl;

//ros::Publisher pub;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
bool finished = false;

/// Open 3D viewer and add point cloud
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (PointCloud<PointXYZ>::ConstPtr cloud){

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointXYZ> (cloud, "robotino cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0,0, "robotino cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "robotino cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg){
    std::clock_t begin = clock();
    finished = true;

    /// Convert to PCL data type
    //PCLPointCloud2* cloud = new PCLPointCloud2;
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    fromROSMsg(*msg, *cloud);

    // Perform the actual filtering
    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
    /*for (size_t i = 0; i < cloud->points.size (); ++i){
        std::cerr << "    " << cloud->points[i].x << " "
                            << cloud->points[i].y << " "
                            << cloud->points[i].z << std::endl;
    }*/

    // Convert to ROS data type
    // sensor_msgs::PointCloud2 output;

    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr inliers (new PointIndices);

    /// Create the segmentation object
    SACSegmentation<PointXYZ> seg;

    /// Optional
    seg.setOptimizeCoefficients (true);

    /// Mandatory
    seg.setModelType(SACMODEL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    std::clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Elapsed time (sec): " << elapsed_secs << std::endl;

    if (inliers->indices.size() == 0){
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " "
                                        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    /*for (size_t i = 0; i < inliers->indices.size (); ++i){
        std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                   << cloud->points[inliers->indices[i]].y << " "
                                                   << cloud->points[inliers->indices[i]].z << std::endl;
    }*/
    // Publish the data
    //pub.publish (output);

    // Find points which lie under the plane to decide whether it is safe for the robot
    int negativePoints = 0;
    int positivePoints = 0;

    for(PointXYZ point : *cloud){
        if(coefficients->values[0] * point.x + coefficients->values[1] * point.y
                + coefficients->values[2] * point.z + coefficients->values[3] < -0.01){
            negativePoints++;
        } else {
            positivePoints++;
        }
    }

    std::cerr << "Negative Points: " << negativePoints << std::endl;
    std::cerr << "Positive Points: " << positivePoints << std::endl;



    viewer = simpleVis(cloud);
    viewer->addPlane(*coefficients, "plane");
    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    ros::shutdown();
}


int main(int argc, char **argv){
    ros::init(argc, argv, "point_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("kinect/depth/points", 1, pointCloud2Callback);
    //pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud_test", 1);


    ros::spin();

}


