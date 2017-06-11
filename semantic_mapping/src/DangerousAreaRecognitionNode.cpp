/**
 * DangerousAreaRecognitionNode.cpp
 * Starts the ROS Node for the recognition of dangerous areas.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include "semantic_mapping/DangerousAreaRecognition.h"
#include <ros/ros.h>

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "dangerous_area_detector");
  DangerousAreaRecognition d;
  d.spin();
  return 0;
}

