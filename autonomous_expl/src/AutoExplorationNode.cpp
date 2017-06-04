/**
 * AutoExplorationNode.cpp
 * Starts the AutoExploration ROS Node.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include "autonomous_expl/AutoExplorationApp.h"

int main( int argc, char *argv[] )
{
  ros::init(argc, argv, "robotino_aut_expl_node");
  AutoExplorationApp a;
  a.spin();
  return 0;
}

