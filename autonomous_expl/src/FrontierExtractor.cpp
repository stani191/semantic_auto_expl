/**
 * FrontierExtractor.cpp
 * Implements the extraction of frontiers, which are the target points for navigation.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include "autonomous_expl/FrontierExtractor.h"

using namespace cv;

/**
 * @brief FrontierExtractor::FrontierExtractor
 * Constructor.
 */
FrontierExtractor::FrontierExtractor(void){
    rng(time(0));
}

/**
 * @brief FrontierExtractor::~FrontierExtractor
 * Desrtuctor.
 */
FrontierExtractor::~FrontierExtractor(){
}

/**
 * @brief FrontierExtractor::extractFrontiers
 * Extracts borders between observed and unobserved map cells.
 * @param robotPos - robot position in map matrix (pixels)
 * @param map - map matrix
 * @param dist_transform - distance transform matrix
 * @returns Mat with extracted frontiers (areas of interest)
 */
Mat FrontierExtractor::extractFrontiers(geometry_msgs::Point robotPos, Mat map, Mat dist_transform){
    Mat empty;

    /// Load the map
    src = Mat::zeros( map.size(), map.type());
    map.copyTo(src);
    if( !src.data )
    { return empty; }

    /// Create a matrix of the same type and size as src (for dst)
    dst = Mat::zeros( src.size(), src.type() );
    occupied = Mat::zeros( src.size(), src.type() );

    /// Extract occupied cells, which are filtered later in the process
    for(int i = 0; i < src.rows; i++){
        for(int j = 0; j < src.cols; j++){
            if(src.at<uchar>(i,j) < 10){
                occupied.at<uchar>(i,j) = 255;
            } else{
                occupied.at<uchar>(i,j) = 0;
            }
        }
    }

    /// Reduce noise with a 3x3 kernel
    blur( src, detected_edges, Size(3,3) );

    /// Apply Canny Edge Detector
    Canny( detected_edges, detected_edges, canny_threshold, canny_threshold*canny_ratio, canny_kernel_size );

    /// Filter occupied edges
    dst = Scalar::all(0);
    src.copyTo( dst, detected_edges);
    int c = 6;
    for(int i = c; i < src.rows-c; i++){
        for(int j = c; j < src.cols-c; j++){
            if(occupied.at<uchar>(i,j) == 255){
                for (int k=i-c; k<=i+c; k++){
                    for (int n=j-c; n<=j+c; n++){
                        dst.at<uchar>(k,n) = 0;
                    }
                }
            }
        }
    }

    /// Save extracted frontier map - 4frontiers.pgm
    std::string imgName = "/home/stanic/master_thesis/catkin_workspace/" + std::to_string(imgCounter) + "_4frontiers.pgm";
    //ROS_ASSERT(imwrite(imgName, dst));

    /// Filter frontiers with low-pass kernel - 5filtered_frontiers.pgm
    float kernel_data[25] = { 0, 0.2, 0.5, 0.2, 0, 0.2, 1, 1, 1, 0.2, 0.5, 1, 1, 1,
                              0.5, 0.2, 1, 1, 1, 0.2, 0, 0.2, 0.5, 0.2, 0 };
    Mat kernel = Mat(5, 5, CV_32F, kernel_data);
    filter2D(dst, dst, -1 , kernel, Point(-1, -1), 0, BORDER_DEFAULT );
    dilate( dst, dst, Mat(), Point(-1, -1), 2, 1, 1);
    blur(dst, dst, Size(20,20) );
    //dst.convertTo(dst, CV_8UC1, 255);
    Mat filtered_frontiers = Mat::zeros(dst.size(), dst.type());
    dst.copyTo(filtered_frontiers);

    imgName = "/home/stanic/master_thesis/catkin_workspace/" + std::to_string(imgCounter) + "_5filtered_frontiers.pgm";
    //ROS_ASSERT(imwrite(imgName, filtered_frontiers));

    return filtered_frontiers;
}

/**
 * @brief FrontierExtractor::extractTargets
 * Extracts suitable targets using the frontier and distance transform matrices
 * @param frontiers - frontier Mat
 * @param robotPos - robot position in map matrix (pixels)
 * @param dist_transform - distance transform Mat
 * @param mapWidth - map width
 * @param mapHeight - map height
 * @returns list of (-)-pairs
 */
vector<std::pair<Point2f,Point2f>> FrontierExtractor::extractTargets(Mat frontiers, geometry_msgs::Point robotPos, Mat dist_transform, double mapWidth, double mapHeight){
    /// Obtain thresholded binary image of frontiers
    for(int i = 0; i < frontiers.rows; i++){
          for(int j = 0; j < frontiers.cols; j++){
              if(frontiers.at<uchar>(i,j) < 100){
                  frontiers.at<uchar>(i,j) = 0;
            } else{
                  frontiers.at<uchar>(i,j) = 255;
            }
        }
    }

    /// Obtain target points - 6frontiers_target.pgm
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Detect edges using canny
    Canny( frontiers, canny_output, canny_threshold, canny_threshold*canny_ratio, canny_kernel_size );
    // Find contours
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    // Get the moments
    vector<Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ ){
         mu[i] = moments( contours[i], false );
    }

    /// Get the mass centers:
    vector<Point2f> mc( contours.size() );
    for( int i = 0; i < contours.size(); i++ ){
          mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    }

    /// Draw centers of mass
    //Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ ){
          Scalar color = Scalar(20);
        circle( frontiers, mc[i], 2, color, -1, 8, 0 );
    }

    /// Draw robot position
    std::string imgName = "/home/stanic/master_thesis/catkin_workspace/" + std::to_string(imgCounter) + "_6frontiers_target.pgm";
    ROS_ASSERT(imwrite(imgName, frontiers));

    /// Move targets to the closest point that can be reached by the robot - 7targets_moved.pgm
    vector<Point2f> ptvec;
    vector<std::pair<Point2f, Point2f>> mc_pair;
    for(int i = 0; i < dist_transform.rows; i++){
        for(int j = 0; j < dist_transform.cols; j++){
            if(dist_transform.at<uchar>(i,j) > 0){
                Point2f tmp(j,i);
                ptvec.push_back(tmp);
            }
       }
    }

    // if no place can be reached by the robot (early stage map)
    if(ptvec.empty()){
        return mc_pair;
    }

    // Nearest neighbor search
    flann::KDTreeIndexParams indexParams;
    flann::Index kdtree(Mat(ptvec).reshape(1), indexParams);
    for(Point2f &pnt : mc){
        if(pnt.x >= 0 || pnt.x <= mapWidth || pnt.y >= 0 || pnt.y <= mapHeight){
            vector<float> query;
            query.push_back(pnt.x);
            query.push_back(pnt.y);
            vector<int> indices;
            vector<float> dists;
            kdtree.radiusSearch(query, indices, dists, 10000, 1);
            for(int &ind : indices){
                mc_pair.push_back(std::make_pair(pnt, ptvec[ind]));
                Scalar color = Scalar(70);
                circle( frontiers, ptvec[ind], 2, color, -1, 8, 0 );
            }
        }
    }

    imgName = "/home/stanic/master_thesis/catkin_workspace/" + std::to_string(imgCounter) + "_7targets_moved.pgm";
    //ROS_ASSERT(imwrite(imgName, frontiers));

    /// Combine frontier and distance transform images - 8combined.pgm
    Mat dst_combined = Mat::zeros(dist_transform.size(), dist_transform.type());
    dst_combined = frontiers + dist_transform;

    imgName = "/home/stanic/master_thesis/catkin_workspace/" + std::to_string(imgCounter) + "_8combined.pgm";
    imgCounter++;
    //ROS_ASSERT(imwrite(imgName, dst_combined));
    return mc_pair;
 }

