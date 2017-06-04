/**
 * DistanceTransformer.cpp
 * Implements the distance transform of the map considering the radius of the robot.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include "autonomous_expl/DistanceTransformer.h"

using namespace cv;

/**
 * @brief DistanceTransformer::DistanceTransformer
 * Constructor.
 */
DistanceTransformer::DistanceTransformer(void){
}

/**
 * @brief DistanceTransformer::~DistanceTransformer
 * Destructor.
 */
DistanceTransformer::~DistanceTransformer(){
}

/**
 * @brief DistanceTransformer::distance_transform
 * Creates and returns a distance transform of the map matrix considering the radius of the robot in
 * order to retrieve the accessible area.
 * @param robotPos - current robot position (pixel)
 * @param map - map matrix
 * @param resolution - map resolution
 */
Mat DistanceTransformer::distance_transform(geometry_msgs::Point robotPos, Mat map, float resolution){
      Mat empty;

      /// Load the map
      src = Mat::zeros( map.size(), map.type());
      map.copyTo(src);
      if( !src.data )
      { return empty; }

      /// Set all undiscovered cells to 0
      for(int i = 0; i < src.rows; i++){
          for(int j = 0; j < src.cols; j++){
              if(src.at<uchar>(i,j) < 250){
                  src.at<uchar>(i,j) = 0;
              }
          }
      }

      /// Create matrices of the same type and size as src
      dst = Mat::zeros( src.size(), src.type() );
      tr = Mat::zeros( src.size(), src.type() );

      /// Peform distance transform
      // Create binary image from source image
      threshold(src, src, 40, 255, CV_THRESH_BINARY);
      std::string imgName = "/home/stanic/master_thesis/catkin_workspace/" + std::to_string(imgCounter) + "_0dist_trans.pgm";
      //ROS_ASSERT(imwrite(imgName, src));
      distanceTransform(src, tr, CV_DIST_L2, 3);
      imgName = "/home/stanic/master_thesis/catkin_workspace/" + std::to_string(imgCounter) + "_0dist_trans2.pgm";
      //ROS_ASSERT(imwrite(imgName, tr));

      /// Remove all distances within the robot's radius - 1dist_trans.pgm
      float radius_threshold = robot_radius/resolution + robot_radius_tolerance/resolution;
      for(int i = 0; i < tr.rows; i++){
          for(int j = 0; j < tr.cols; j++){
              if(tr.at<float>(i,j) < radius_threshold){
                 tr.at<float>(i,j) = 0.0;
              }
         }
      }
      imgName = "/home/stanic/master_thesis/catkin_workspace/" + std::to_string(imgCounter) + "_1dist_trans.pgm";
      //ROS_ASSERT(imwrite(imgName, tr));

      /// Normalize distance transform - 2dist_trans.pgm
      normalize(tr, tr, 0, 1., NORM_MINMAX);
      tr.convertTo(tr, CV_8UC1, 255);

      imgName = "/home/stanic/master_thesis/catkin_workspace/" + std::to_string(imgCounter) + "_2dist_trans.pgm";
      //ROS_ASSERT(imwrite(imgName, tr));

      /// Create accessible map - 3accessible.pgm
      for(int i = 0; i < tr.rows; i++){
          for(int j = 0; j < tr.cols; j++){
              if(tr.at<uchar>(i,j) > radius_threshold){
                  tr.at<uchar>(i,j) = 50;
              }
              else {
                  tr.at<uchar>(i,j) = 0;
              }
         }
      }
      imgName = "/home/stanic/master_thesis/catkin_workspace/" + std::to_string(imgCounter) + "_3accessible.pgm";
      imgCounter++;
      //ROS_ASSERT(imwrite(imgName, tr));
      //std::cout << "DistanceTransformer finished." << std::endl;
      return tr;
}
