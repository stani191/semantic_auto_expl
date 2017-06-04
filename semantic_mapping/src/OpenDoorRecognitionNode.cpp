/**
 * OpenDoorRecognitionNode.cpp
 * Starts the ROS Node for the recognition of open doors.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include "semantic_mapping/OpenDoorRecognitionApp.h"
#include <ros/ros.h>

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "open_door_detector");
  OpenDoorRecognitionApp o;
  o.spin();
  return 0;
}

