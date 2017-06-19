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

#include <iostream>
#include <sys/time.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "semantic_mapping_msgs/DoorMessage.h"
#include "semantic_mapping/DoorStateChecker.h"

class MarkerPublisher
{

public:
    MarkerPublisher();
    ~MarkerPublisher();

    void spin();
    bool isDuplicate(const semantic_mapping_msgs::DoorMessage msg);
    void openDoorCallback(semantic_mapping_msgs::DoorMessage msg);
    void mapCallback(const nav_msgs::OccupancyGrid msg);
    void navTargetCallback(geometry_msgs::PoseStamped msg);
    void dangerousAreasCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    double euclideanDistance(const geometry_msgs::Point p1, const geometry_msgs::Point p2);
    geometry_msgs::Point midPoint(const geometry_msgs::Point p1, const geometry_msgs::Point p2);
    void replaceDoor(semantic_mapping_msgs::DoorMessage oldDoor, semantic_mapping_msgs::DoorMessage newDoor);
    void addDoorMarker(const geometry_msgs::Point pose, std::string doorState, double scale, int id);
    void deleteDoorMarker(int id);
    void updateNavTargetMarker(const geometry_msgs::Point pose);
    void updateDoorMarkers();


private:
    std::string node_name;
    ros::NodeHandle n;
    ros::Subscriber input_open_door_sub;
    ros::Subscriber nav_target_sub;
    ros::Subscriber update_sub;
    ros::Subscriber dangerous_sub;
    tf::TransformListener listener;

    // TODO: subscriber for dangerous msg
    ros::Publisher marker_pub;

    // lists of stored semantic information
    std::vector<semantic_mapping_msgs::DoorMessage> doors;
    geometry_msgs::PoseStamped navTargetMap;
    // TODO: vector of dangerous areas

    DoorStateChecker d = {n};
};

#endif // MARKERPUBLISHER_H
