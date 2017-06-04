#ifndef TARGETSELECTOR_H
#define TARGETSELECTOR_H

/**
 * TargetSelector.h
 * Selects the goal target out of all targets using a cost function.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include <stdlib.h>
#include <stdio.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

using namespace cv;

class TargetSelector{

public:
    TargetSelector(void);
    ~TargetSelector();
    geometry_msgs::Point selectTarget(vector<std::pair<Point2f,Point2f>> targets, geometry_msgs::Point robotPos, Mat dist_transform, Mat frontiers);

private:
    int imgCounter = 0;

};

#endif // TARGETSELECTOR_H
