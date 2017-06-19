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
#include <pcl/io/pcd_io.h>
#include <ctime>

using namespace pcl;
int filenameCounter = 0;

void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg){

    /// Convert to PCL data type
    //PCLPointCloud2* cloud = new PCLPointCloud2;
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    fromROSMsg(*msg, *cloud);

    // Perform the actual filtering
    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
    std::string filename = "pcd/pcd_open_door_" + std::to_string(filenameCounter) + ".pcd";
    filenameCounter++;
    pcl::io::savePCDFileASCII (filename, *cloud);
    std::cerr << "Saved data points to " << filename << std::endl;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "point_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("kinect/depth/points", 1, pointCloud2Callback);
    while(ros::ok()){
        std::cerr << "Press enter to save current cloud" << std::endl;
        getchar();
        ros::spinOnce();
    }

}



