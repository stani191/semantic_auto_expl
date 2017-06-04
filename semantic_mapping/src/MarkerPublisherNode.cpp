/**
 * MarkerPublisherNode.cpp
 * Starts the ROS Node for the rviz semantic marker publisher.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include "semantic_mapping/MarkerPublisher.h"
#include <ros/ros.h>

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "marker_publisher");
  MarkerPublisher m;
  m.spin();
  return 0;
}
