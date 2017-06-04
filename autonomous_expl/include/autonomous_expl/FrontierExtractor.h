#ifndef FRONTIEREXTRACTOR_H
#define FRONTIEREXTRACTOR_H

/**
 * FrontierExtractor.h
 * Implements the extraction of frontiers, which are the target points for navigation.
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

class FrontierExtractor{

public:
    FrontierExtractor(void);
    ~FrontierExtractor();
    Mat extractFrontiers(geometry_msgs::Point robotPos, Mat map, Mat dist_transform);
    vector<std::pair<Point2f,Point2f>> extractTargets(Mat frontiers, geometry_msgs::Point robotPos, Mat dist_transform, double mapWidth, double mapHeight);

private:
    Mat src, src_gray;
    Mat dst, detected_edges;
    Mat occupied;
    int imgCounter = 0;

    // Edge detector parameters
    static int const canny_threshold = 100;
    static int const canny_ratio = 3;
    static int const canny_kernel_size = 3;
    RNG rng;
};

#endif // FRONTIEREXTRACTOR_H
