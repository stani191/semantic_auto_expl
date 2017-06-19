#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"

using namespace cv;

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg){
    try {
        imshow("Depth Image", cv_bridge::toCvShare(msg, "32FC1")->image);
        waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to '32FC1'", msg->encoding.c_str());
    }
}

void depthRectImageCallback(const sensor_msgs::ImageConstPtr& msg){
    try {
        imshow("Depth Rect Image", cv_bridge::toCvShare(msg, "32FC1")->image);
        waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to '32FC1'", msg->encoding.c_str());
    }
}

void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg){
    try {
        imshow("RGB Image", cv_bridge::toCvShare(msg, "rgb8")->image);
        waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'", msg->encoding.c_str());
    }
}

void irImageCallback(const sensor_msgs::ImageConstPtr& msg){
    try {
        imshow("IR Image", cv_bridge::toCvShare(msg, "rgb8")->image);
        waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'", msg->encoding.c_str());
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    namedWindow("Depth Image");
    namedWindow("Depth Rect Image");
    namedWindow("RGB Image");
    namedWindow("IR Image");
    startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub1 = it.subscribe("kinect/depth/image", 1, depthImageCallback);
    image_transport::Subscriber sub2 = it.subscribe("kinect/rgb/image_raw", 1, rgbImageCallback);
    image_transport::Subscriber sub3 = it.subscribe("kinect/ir/image", 1, irImageCallback);
    ros::spin();
    destroyWindow("Depth Image");
    destroyWindow("Depth Rect Image");
    destroyWindow("RGB Image");
    destroyWindow("IR Image");
}

