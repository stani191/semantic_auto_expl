#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <ctime>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/crop_box.h>

using namespace pcl;

boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));
bool finished = false;

bool comparePoints(const PointXYZ& p1, const PointXYZ& p2) {
    return (p1.x<p2.x);
}

double eucludeanDistance(const PointXYZ& p1, const PointXYZ& p2) {
    return (sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.y-p2.y,2)));
}

/**
 * Detects stairs and obstacles which cannot be seen by the 2D laser.
 * @param cloud The Kinect point cloud delivered by the robot
 * @returns The point cloud without the floor
 */
PointCloud<PointXYZ>::Ptr detectStairs(PointCloud<PointXYZ>::Ptr cloud){
    PointCloud<PointXYZ>::Ptr cloud_p (new PointCloud<PointXYZ>), cloud_f (new PointCloud<PointXYZ>);
    PCDWriter writer;

    std::cout << std::endl << "Detecting Stairs & Obstacles..." << std::endl;
    std::clock_t begin = clock();

    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr inliers (new PointIndices);
    SACSegmentation<PointXYZ> seg;
    ExtractIndices<PointXYZ> extract;

    /// Apply RANSAC to extract floor plane
    /// Plane has to be parallel to x-Axis of robot
    seg.setOptimizeCoefficients (true);
    seg.setModelType(SACMODEL_PARALLEL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    Eigen::Vector3f x(1,0,0);
    seg.setAxis(x);
    seg.setEpsAngle(0.1);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);


    /// If there is no plane print error msg
    if (inliers->indices.size() == 0){
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        std::clock_t end = clock();
        double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
        std::cout << "Elapsed time (ms): " << elapsed_ms << std::endl;
        return cloud;
    }

    /// TODO: Additional verification, if plane is really floor

    /// Print elapsed time, model coefficients and number of inliers
    std::clock_t end = clock();
    double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
    std::cout << "Elapsed time for floor plane fitting (ms): " << elapsed_ms << std::endl;
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

    /// TODO: Add those points to new semantic map!

    /// Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the wall planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    /// Save plane
    std::stringstream ss;
    ss << "test/floor.pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    viewer->addPointCloud<PointXYZ> (cloud_p, "floor cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1.0,0, "floor cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "floor cloud");

    /// Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);

    return cloud_f;
}


/**
 * Detects open and closed doors.
 * @param cloud The Kinect cloud (with or without the floor)
 */
void detectDoors(PointCloud<PointXYZ>::Ptr cloud){
    PointCloud<PointXYZ>::Ptr cloud_p (new PointCloud<PointXYZ>), cloud_f (new PointCloud<PointXYZ>);
    PCDWriter writer;

    std::clock_t begin = clock();

    ModelCoefficients::Ptr coefficients(new ModelCoefficients ());
    PointIndices::Ptr inliers(new PointIndices ());
    ExtractIndices<PointXYZ> extract;
    SACSegmentation<PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_PARALLEL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    Eigen::Vector3f y(0,1,0);
    seg.setAxis(y);
    seg.setEpsAngle(0.1);
    seg.setDistanceThreshold (0.04);
    seg.setMaxIterations(30);

    /// Use RANSAC to find the dominant wall
    /// Dominant wall has to be parallel to y-Axis (ANGLE=0!)
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0){
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      return;
    }

    /// Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the wall planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                          << coefficients->values[1] << " "
                                          << coefficients->values[2] << " "
                                          << coefficients->values[3] << std::endl;

    /// Save plane
    std::stringstream ss;
    ss << "test/wall.pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    /// Visualize wall
    viewer->addPointCloud<PointXYZ> (cloud_p, "wall cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,1.0, "wall cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "wall cloud");

    /// Extract stripe constrained on y=0
    PointCloud<PointXYZ>::Ptr stripe_cloud (new PointCloud<PointXYZ>);
    for(PointXYZ point : *cloud_p){
        if(point.y > -0.005 && point.y < 0.005){
            stripe_cloud->points.push_back(point);
        }
    }

    /// Visualize stripe
    viewer->addPointCloud<PointXYZ> (stripe_cloud, "stripe");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,1.0,1.0, "stripe");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "stripe");

    /// Sort point cloud stripe by x values
    std::sort(stripe_cloud->points.begin(), stripe_cloud->points.end(), comparePoints);

    /// Perform nearest neighbor search on the stripe to find gaps which are possible open doors
    /// Radius = 5cm
    /// Algorithm lines 10-28
    int n_prev = 1;
    int n_curr;
    float radius = 0.02;
    float door_width_min = 0.7;
    float door_width_max = 1.6;
    float door_height = 1.95;
    float kinect_height = 0.36;
    PointXYZ p_curr = stripe_cloud->points.front();
    PointXYZ p_end = stripe_cloud->points.back();
    PointXYZ gap_start;
    PointXYZ gap_end;
    // KDTree structure for KNN search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    int i = 0;
    while(eucludeanDistance(p_curr,p_end) > 0.05){ // line 10
        /// determine number of current neighbors n_curr
        kdtree.setInputCloud(stripe_cloud);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        /*std::cout << "Neighbors within radius search at (" << p_curr.x
                  << " " << p_curr.y
                  << " " << p_curr.z
                  << ") with radius=" << radius << std::endl;*/
        if ( kdtree.radiusSearch (p_curr, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
            /*for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
              std::cout << "    "  <<   stripe_cloud->points[ pointIdxRadiusSearch[i] ].x
                        << " " << stripe_cloud->points[ pointIdxRadiusSearch[i] ].y
                        << " " << stripe_cloud->points[ pointIdxRadiusSearch[i] ].z
                        << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;*/
            n_curr = pointIdxRadiusSearch.size();
            std::cout << "Neighbors:" << pointIdxRadiusSearch.size() << std::endl;
        } else {
            n_curr = 0;
            std::cout << "Neighbors: 0" << std::endl;
        }

        /// case analysis: determine start and end of the gap
        if(n_curr == 0 && n_prev > 0){ // line 13
            gap_start = p_curr;
        } else if(n_curr > 0 && n_prev == 0){ // line 15
            gap_end = p_curr;

            /// A complete gap is found - now door detection takes place
            std::cout << "Gap start:" << std::endl;
            std::cout << " "<< gap_start.x
                              << " " << gap_start.y
                              << " " << gap_start.z << std::endl;
            std::cout << "Gap end:" << std::endl;
            std::cout << " "<< gap_end.x
                              << " " << gap_end.y
                              << " " << gap_end.z << std::endl;

            /// Visualize Gap
            PointCloud<PointXYZ>::Ptr gap_cloud (new PointCloud<PointXYZ>);
            std::string gapstring = "gap" + std::to_string(i);
            gap_cloud->points.push_back(gap_start);
            gap_cloud->points.push_back(gap_end);
            viewer->addPointCloud<PointXYZ> (gap_cloud, gapstring);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 1.0,1.0,0, gapstring);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 5, gapstring);

            /// A door candidate is detected - width check
            //if(eucludeanDistance(gap_start,gap_end) > door_width_min &&
            //        eucludeanDistance(gap_start,gap_end) < door_width_max){ // line 17

            /// Make a cube using the gap points
            float x_min = std::min(gap_start.x, gap_end.x);
            float x_max = std::max(gap_start.x, gap_end.x);
            float y_min = std::min(kinect_height-door_height, kinect_height);
            float y_max = std::max(kinect_height-door_height, kinect_height);
            float z_min = std::min(gap_start.z, gap_end.z);
            float z_max = std::max(gap_start.z, gap_end.z);

            Eigen::Vector4f minPoint;
            minPoint[0] = x_min;
            minPoint[1] = y_min;
            minPoint[2] = z_min;
            Eigen::Vector4f maxPoint;
            maxPoint[0] = x_max;
            maxPoint[1] = y_max;
            maxPoint[2] = z_max;

            /// Filter points that are in the cube
            PointCloud<PointXYZ>::Ptr door_cloud (new PointCloud<PointXYZ>);
            CropBox<PointXYZ> door_filter;
            door_filter.setInputCloud(cloud);
            door_filter.setMin(minPoint);
            door_filter.setMax(maxPoint);
            door_filter.filter (*door_cloud);

            viewer->addCube(x_min, x_max, y_min, y_max, z_min, z_max, 0.0, 1.0, 1.0, "AABB");

            /// Visualize door points
            std::string doorstring = "door" + std::to_string(i);
            viewer->addPointCloud<PointXYZ> (door_cloud, doorstring);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 1.0,0,1.0, doorstring);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 3, doorstring);
           // }
        }

        /// Move further on the stripe and repeat
        /// Visualize Temp
        /*PointCloud<PointXYZ>::Ptr temp_cloud (new PointCloud<PointXYZ>);
        std::string tempstring = "temp" + std::to_string(i);
        temp_cloud->points.push_back(p_curr);
        viewer->addPointCloud<PointXYZ> (temp_cloud, tempstring);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 0,1.0,1.0, tempstring);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 5, tempstring);*/

        p_curr.x += ((coefficients->values[0]/coefficients->values[0])*0.01);
        p_curr.z -= ((coefficients->values[2]/coefficients->values[0])*0.01);
        i++;
        n_prev = n_curr;
    }

    std::clock_t end = clock();
    double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
    std::cout << "Elapsed time wall fitting (ms): " << elapsed_ms << std::endl;
}

int main (int argc, char** argv){
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    std::cout << "Start..." << std::endl;
    /// Load the point cloud file specified at argv[1]
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1){
       PCL_ERROR ("Couldn't read file\n");
       return (-1);
    }

    /// Print cloud size (width*height)
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << std::endl;

    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointXYZ> (cloud, "original cloud");
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 1.0,0,0, "original cloud");
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original cloud");

    std::clock_t begin = clock();
    detectDoors(detectStairs(cloud));
    std::clock_t end = clock();
    double elapsed_ms = (double(end - begin) / CLOCKS_PER_SEC)*1000;
    std::cout << "Elapsed time overall (ms): " << elapsed_ms << std::endl;

    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return (0);
}


