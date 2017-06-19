#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <ctime>

using namespace pcl;


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
bool finished = false;

/// Open 3D viewer and add point cloud
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (PointCloud<PointXYZ>::ConstPtr cloud, PointCloud<PointXYZ>::ConstPtr floor_cloud,
                                                                PointCloud<PointXYZ>::ConstPtr wall_cloud){

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointXYZ> (cloud, "robotino cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0,0, "robotino cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "robotino cloud");
  viewer->addPointCloud<PointXYZ> (floor_cloud, "floor cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1.0,0, "floor cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "floor cloud");
  viewer->addPointCloud<PointXYZ> (wall_cloud, "wall cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1.0,0, "wall cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "wall cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


int main(int argc, char **argv){
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

  /// Load the point cloud file specified at argv[1]
  if (pcl::io::loadPCDFile<PointXYZ> (argv[1], *cloud) == -1){
     PCL_ERROR ("Couldn't read file\n");
     return (-1);
  }

  /// Print cloud size (width*height)
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << std::endl;

  std::clock_t begin = clock();
  ModelCoefficients::Ptr coefficients (new ModelCoefficients);
  PointIndices::Ptr inliers (new PointIndices);

  /// Create the segmentation object
  SACSegmentation<PointXYZ> seg;

  /// Optional
  seg.setOptimizeCoefficients (true);

  /// Apply RANSAC to extract floor plane
  /*if(std::string(argv[2]) == "parallel"){*/
      /// Plane has to be parallel to x-Axis of robot
      seg.setModelType(SACMODEL_PARALLEL_PLANE);
      seg.setMethodType(SAC_RANSAC);
      seg.setDistanceThreshold(0.01);
      Eigen::Vector3f x(1,0,0);
      seg.setAxis(x);
      seg.setEpsAngle(0.1);
      std::cout << "PARALLEL" << std::endl;
  /*} else {
      /// No constraints on plane
      seg.setModelType(SACMODEL_PLANE);
      seg.setMethodType(SAC_RANSAC);
      seg.setDistanceThreshold(0.01);
      std::cout << "NORMAL" << std::endl;
  }*/
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  /// If there is no plane print error msg
  if (inliers->indices.size() == 0){
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      std::clock_t end = clock();
      double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
      std::cout << "Elapsed time (ms): " << elapsed_ms << std::endl;
      return -1;
  }

  //int ransacCounter = 0;

  /// for non-parallel case:
  /*if(argv[3]){
      std::cout << "N ITERATIONS" << std::endl;
      while(coefficients->values[3] > 0.6 || coefficients->values[3] < 0.5){
          std::cout << "RANSAC plane not useful - searching new plane: " << ransacCounter << std::endl;
          PointCloud<PointXYZ>::Ptr outliers_cloud (new PointCloud<PointXYZ>);
          ExtractIndices<PointXYZ> extract;
          extract.setInputCloud(cloud);
          extract.setIndices(inliers);
          extract.setNegative(true);
          extract.filter(*outliers_cloud);
          *cloud = *outliers_cloud;
          seg.setInputCloud(cloud);
          seg.segment(*inliers, *coefficients);
          ++ransacCounter;
      }
  } else {
        std::cout << "1 ITERATION" << std::endl;
  }*/

  /// Print elapsed time, model coefficients and number of inliers
  std::clock_t end = clock();
  std::cout << " --- FLOOR CLOUD --- ";
  double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
  std::cout << "Elapsed time (ms): " << elapsed_ms << std::endl;
  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  /// New cloud which only incorporates inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < inliers->indices.size (); ++i){
      inliers_cloud->points.push_back(cloud->points[inliers->indices[i]]);
  }

  /// Calculate angle between extracted plane and z-axis
  double alpha = asin(coefficients->values[2]/ (sqrt(pow(coefficients->values[0],2) + pow(coefficients->values[1],2) + pow(coefficients->values[2],2))));
  std::cerr << "ANGLE: " << alpha << std::endl;
  // Publish the data
  //pub.publish (output);

  /// Find points which lie under the plane to decide if it is safe for the robot
  int negativePoints = 0;
  int positivePoints = 0;

  for(PointXYZ point : *cloud){
      if(coefficients->values[0] * point.x + coefficients->values[1] * point.y
              + coefficients->values[2] * point.z + coefficients->values[3] < -0.05){
          negativePoints++;
      } else {
          positivePoints++;
      }
  }

  std::cerr << "Negative Points: " << negativePoints << std::endl;
  std::cerr << "Positive Points: " << positivePoints << std::endl;

  begin = clock();
  /// Extract Wall Plane with RANSAC
  ModelCoefficients::Ptr coefficients2 (new ModelCoefficients);
  PointIndices::Ptr inliers2 (new PointIndices);

  /// Create the segmentation object
  SACSegmentation<PointXYZ> seg2;

  /// Optional
  seg2.setOptimizeCoefficients (true);

  /// Apply RANSAC to extract floor plane
  /*if(std::string(argv[2]) == "parallel"){*/
      /// Plane has to be parallel to x-Axis of robot
      seg2.setModelType(SACMODEL_PERPENDICULAR_PLANE);
      seg2.setMethodType(SAC_RANSAC);
      seg2.setDistanceThreshold(0.01);
      Eigen::Vector3f z(0,0,1);
      seg2.setAxis(z);
      seg2.setEpsAngle(0.1);
      std::cout << "PARALLEL" << std::endl;
  /*} else {
      /// No constraints on plane
      seg.setModelType(SACMODEL_PLANE);
      seg.setMethodType(SAC_RANSAC);
      seg.setDistanceThreshold(0.01);
      std::cout << "NORMAL" << std::endl;
  }*/
  seg2.setInputCloud(cloud);
  seg2.segment(*inliers2, *coefficients2);

  /// If there is no plane print error msg
  if (inliers->indices.size() == 0){
      PCL_ERROR ("Could not estimate a wall planar model for the given dataset.");
      end = clock();
      double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
      std::cout << "Elapsed time (ms): " << elapsed_ms << std::endl;
      return -1;
  }

  /// New cloud which only incorporates inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < inliers2->indices.size (); ++i){
      wall_cloud->points.push_back(cloud->points[inliers2->indices[i]]);
  }

  std::cout << " --- WALL CLOUD --- ";
  std::cerr << "Model coefficients: " << coefficients2->values[0] << " "
                                      << coefficients2->values[1] << " "
                                      << coefficients2->values[2] << " "
                                      << coefficients2->values[3] << std::endl;
  std::cerr << "Model inliers: " << inliers2->indices.size () << std::endl;

  /// Open visualization
  viewer = simpleVis(cloud, inliers_cloud, wall_cloud);
  viewer->addPlane(*coefficients, "plane");
  while (!viewer->wasStopped()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}



