#ifndef DOORSTATECHECKER_H
#define DOORSTATECHECKER_H

/**
 * DoorStateChecker.h
 * Checks states of doors (open/closed) using map information
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 05.06.2017
 */

#include <ros/ros.h>
#include <iostream>
#include <sys/time.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/opencv.hpp>

#include "semantic_mapping_msgs/DoorMessage.h"


class DoorStateChecker
{

public:
    DoorStateChecker(ros::NodeHandle&);
    ~DoorStateChecker();
    void checkDoorStates(std::vector<semantic_mapping_msgs::DoorMessage> &doors);
    void checkDoorState(semantic_mapping_msgs::DoorMessage &door);

    geometry_msgs::PointStamped midPoint(const geometry_msgs::PointStamped p1, const geometry_msgs::PointStamped p2);
    double euclideanDistance(const geometry_msgs::Point p1, const geometry_msgs::Point p2);
    double euclideanDistance(const cv::Point2f p1, const geometry_msgs::PointStamped p2);
    geometry_msgs::PointStamped mapToPixelCoords(geometry_msgs::PointStamped point);

    void mapCallback(const nav_msgs::OccupancyGrid msg);

private:
    ros::Subscriber map_sub;
    tf::TransformListener tf_listener;

    int occupiedThresh = 7;

    cv::Mat dyn_map;
    geometry_msgs::Point mapOrigin;
    double mapResolution;
    double mapWidth;
    double mapHeight;
};

#endif // DOORSTATECHECKER_H
