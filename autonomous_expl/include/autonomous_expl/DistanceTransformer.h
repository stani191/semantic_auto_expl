#ifndef DISTANCETRANSFORMER_H
#define DISTANCETRANSFORMER_H

/**
 * DistanceTransformer.h
 * Implements the distance transform of the map considering the radius of the robot.
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

#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

using namespace cv;

class DistanceTransformer{

public:
    DistanceTransformer(void);
    ~DistanceTransformer();
    Mat distance_transform(geometry_msgs::Point robotPos, Mat map, float resolution);

private:
    Mat src, dst, bw, tr;

    // robotino params
    float robot_radius = 0.22;
    float robot_radius_tolerance = 0.05;
    int imgCounter = 0;

};

# endif // DISTANCETRANSFORMER_H
