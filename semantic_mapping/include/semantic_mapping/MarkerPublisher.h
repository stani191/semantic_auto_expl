#ifndef MARKERPUBLISHER_H
#define MARKERPUBLISHER_H

/**
 * MarkerPublisher.h
 * Starts the rviz semantic marker publisher
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include <ros/ros.h>
#include <iostream>
#include <sys/time.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "semantic_mapping_msgs/DoorMessage.h"

class MarkerPublisher
{

public:
    MarkerPublisher();
    ~MarkerPublisher();

    void spin();
    bool isDuplicate(const semantic_mapping_msgs::DoorMessage msg);
    void openDoorCallback(semantic_mapping_msgs::DoorMessage msg);
    //void closedDoorCallback(robotino_utils_msgs::DoorMessage msg);
    void navTargetCallback(geometry_msgs::PoseStamped msg);
    double euclideanDistance(const geometry_msgs::Point p1, const geometry_msgs::Point p2);
    geometry_msgs::Point midPoint(const geometry_msgs::Point p1, const geometry_msgs::Point p2);
    void replaceDoor(semantic_mapping_msgs::DoorMessage oldDoor, semantic_mapping_msgs::DoorMessage newDoor);
    void addDoorMarker(const geometry_msgs::Point pose, std::string doorState, double scale, int id);
    void deleteDoorMarker(int id);
    void updateNavTargetMarker(const geometry_msgs::Point pose);
    void updateDoorMarkers();

    // TODO: callback for dangerous areas msg

private:
    std::string node_name;
    ros::NodeHandle n;
    ros::Subscriber input_open_door_sub;
    //ros::Subscriber input_closed_door_sub;
    ros::Subscriber nav_target_sub;
    tf::TransformListener listener;

    // TODO: subscriber for dangerous msg
    ros::Publisher marker_pub;

    // lists of stored semantic information
    std::vector<semantic_mapping_msgs::DoorMessage> doors;
    geometry_msgs::PoseStamped navTargetMap;
    // TODO: vector of dangerous areas
};

#endif // MARKERPUBLISHER_H
