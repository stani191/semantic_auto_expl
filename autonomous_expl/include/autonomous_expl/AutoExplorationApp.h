#ifndef AUTOEXPLORATIONAPP_H
#define AUTOEXPLORATIONAPP_H

/**
 * AutoExplorationApp.h
 * Header file for autonomous target selection and navigation.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

#include <actionlib_msgs/GoalStatusArray.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>

#include "autonomous_expl/DistanceTransformer.h"
#include "autonomous_expl/FrontierExtractor.h"
#include "autonomous_expl/TargetSelector.h"

using namespace cv;

class AutoExplorationApp{

public:
    AutoExplorationApp();
    ~AutoExplorationApp();

    bool navigate(geometry_msgs::PointStamped target);
    bool selectNavTarget(Mat map, float resolution);
    bool navigate();
    void setCurrentRobotPos();
    bool updateMap();

    void resetTimeoutTimer();

    geometry_msgs::PointStamped mapToPixelCoords(geometry_msgs::PointStamped robotPosMap);
    geometry_msgs::PointStamped pixelToMapCoords(geometry_msgs::PointStamped robotPosPixel);
    double euclideanDistance(const geometry_msgs::Point p1, const geometry_msgs::Point p2);

    void rotate();
    void step();
    void stop();

    void spin();
    void mapCallback(const nav_msgs::OccupancyGrid msg);
    void mapUpdateCallback(const map_msgs::OccupancyGridUpdate msg);
    void velCallback(const geometry_msgs::Twist msg);
    void odomCallback(const nav_msgs::Odometry msg);
    void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray msg);

private:
    DistanceTransformer df;
    FrontierExtractor fe;
    TargetSelector ts;

    geometry_msgs::PointStamped robotPosBaseLink;
    geometry_msgs::PointStamped robotPosMap;
    geometry_msgs::PointStamped robotPosWorld;
    geometry_msgs::PointStamped targetMap;
    geometry_msgs::PointStamped targetWorld;
    std::string node_name;

    Point2f robotPositionMap;
    double robotOrientationMap;
    Mat dyn_map;
    Mat vis_map;
    geometry_msgs::Point mapOrigin;
    double mapResolution;
    double mapWidth;
    double mapHeight;

    bool isMoving = false;
    bool initFinished = false;
    bool navigationActive = true;
    bool mapAvailable = false;
    bool isTimeout = false;

    struct timeval tp;

    ros::NodeHandle n;
    ros::Publisher vel_pub;
    ros::Publisher target_pub;
    ros::Publisher tilt_pub;
    ros::Subscriber map_sub;
    ros::Subscriber mapUpdate_sub;
    ros::Subscriber vel_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber move_base_status_sub;
    ros::ServiceClient mapClient;

    tf::TransformListener tf_listener;

    double time_begin;
    double time_current;
    double timeout_begin;
    double timeout_current;
    double distance_travelled = 0.0;
    const double timeout_threshold = 10.0;
    int num_targets = 0;

    const double PI = 3.1415926535897;
    const double KINECT_TILT_90DEG = -0.12;
    const int WAIT_INIT = 30;
    const int WAIT_TILT = 5;
};

#endif // AUTOEXPLORATIONAPP_H
