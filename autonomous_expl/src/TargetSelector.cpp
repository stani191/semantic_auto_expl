/**
 * TargetSelector.cpp
 * Selects the goal target out of all targets using a cost function.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include "autonomous_expl/TargetSelector.h"

using namespace cv;

/**
 * @brief TargetSelector::TargetSelector
 * Constructor.
 */
TargetSelector::TargetSelector(void){
    srand(time(0));
}

/**
 * @brief TargetSelector::~TargetSelector
 * Destructor.
 */
TargetSelector::~TargetSelector(){
}

/**
 * @brief AutoExplorationApp::pixelToMapCoords
 * Converts given robot map matrix position (pixels) to map position (map frame)
 * @param robotPosPixel - robot position in map matrix (in pixels)
 * @returns robot position in map frame
 */
Point2f pixelToMapCoords(Point2f robotPosPixel){
    Point2f result;
    result.x = robotPosPixel.y * 0.05 - 20.0;
    result.y = robotPosPixel.x * 0.05 - 20.0;

    return result;
}

/**
 * @brief TargetSelector::selectTarget
 * Selects the best target from the given target list.
 * @param targets - target list, ()-pair
 * @param robotPos - current robot position in map matrix (pixels)
 * @param filtered_frontiers - Mat of filtered frontiers
 * @param frontiers - thresholded binary image of frontiers
 * @returns best target point
 */
geometry_msgs::Point TargetSelector::selectTarget(vector<std::pair<Point2f, Point2f>> targets, geometry_msgs::Point robotPos, Mat filtered_frontiers, Mat frontiers){
    /// Calculate quality value from distance and frontier value
    vector<double> cost;
    for(std::pair<Point2f,Point2f> &target : targets){
        // Manhattan distance
        double dist = abs(target.second.x-robotPos.x) + abs(target.second.y-robotPos.y);
        double value = frontiers.at<uchar>(target.first.y,target.first.x*3);
        if(value == 0){
            value = 0.1;
        }
        cost.push_back(dist+255*(value));
    }
    Scalar color = Scalar(0);
    circle(filtered_frontiers, targets[std::distance(cost.begin(), std::min_element(cost.begin(), cost.end()))].second, 3, color, -1, 8, 0 );

    /// Selected Target - 9target.pgm
    std::string imgName = "/home/stanic/master_thesis/catkin_workspace/" + std::to_string(imgCounter) + "_9target.pgm";
    imgCounter++;
    ROS_ASSERT(imwrite(imgName, filtered_frontiers));

    Point2f target_ = targets[std::distance(cost.begin(), std::min_element(cost.begin(), cost.end()))].second;
    geometry_msgs::Point target;
    target.x = target_.x;
    target.y = target_.y;
    target.z = 0;
    return target;
}
