/**
 * DangerousAreaRecognition.cpp
 * Dangerous area recognition module. Is subscribed to the kinect point cloud topic "kinect/depth/points"
 * which is used to recognize dangerous areas (stairs, things which are not in the field of view
 * of SLAM (not at the laser height). The recognized point cloud is published via "/dangerous_areas".
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include "semantic_mapping/DangerousAreaRecognition.h"

using namespace pcl;

/**
 * @brief DangerousAreaRecognition::DangerousAreaRecognition
 * Constructor. Registers subscriber for depth points and publisher for dangerous areas
 */
DangerousAreaRecognition::DangerousAreaRecognition(){
    input_cloud_sub = n.subscribe("kinect/depth/points", 1, &DangerousAreaRecognition::cloudCallback, this);
    gettimeofday(&tp, NULL);
    console::setVerbosityLevel(console::L_ALWAYS);
    ROS_INFO("Dangerous area recognition node started.");
    dangerous_pub = n.advertise<sensor_msgs::PointCloud2>("/dangerous_areas",5);

    // get robot height
    geometry_msgs::PointStamped kinectOriginWorld;
    geometry_msgs::PointStamped kinectOrigin;
    kinectOriginWorld.header.stamp = ros::Time::now();
    kinectOriginWorld.header.frame_id = "world";
    kinectOriginWorld.point.x = kinectOriginWorld.point.y = kinectOriginWorld.point.z = 0;

    tf::TransformListener listener;
    if(listener.waitForTransform("kinect_depth_optical_frame","world", ros::Time::now(), ros::Duration(2.0))){
        listener.transformPoint("kinect_depth_optical_frame", kinectOriginWorld, kinectOrigin);

        kinect_height = kinectOrigin.point.y;
    } else {
        kinect_height = -0.5 - 0.05 - 0.03;
    }
    ROS_INFO("Kinect sensor height = %f", kinect_height);
}

/**
 * @brief DangerousAreaRecognition::~DangerousAreaRecognition
 * Destructor.
 */
DangerousAreaRecognition::~DangerousAreaRecognition(){
    ROS_INFO("Open door recognition node terminated.");
}

void DangerousAreaRecognition::setDebugFlag(){
    debug = true;
}

void DangerousAreaRecognition::setKinectHeight(double d){
    kinect_height = d;
}

void DangerousAreaRecognition::setKinectHeightOffset(double d){
    kinect_height_offset = d;
}

void DangerousAreaRecognition::setKinectTilt(double d){
    kinect_tilt = d;
}


/**
 * @brief OpenDoorRecognitionApp::simpleVis
 * @param cloud
 * @param tilt_floor_cloud
 * @param wall_cloud
 * @return
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> DangerousAreaRecognition::simpleVis (PointCloud<PointXYZ>::ConstPtr cloud0, PointCloud<PointXYZ>::ConstPtr cloud1, PointCloud<PointXYZ>::ConstPtr cloud2, PointCloud<PointXYZ>::ConstPtr cloud3, PointCloud<PointXYZ>::ConstPtr cloud4){
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointXYZ> (cloud0, "cloud 0");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5,0.5,0.5, "cloud 0");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud 0");
  viewer->addPointCloud<PointXYZ> (cloud1, "cloud 1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,1.0,1.0, "cloud 1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud 1");
  viewer->addPointCloud<PointXYZ> (cloud2, "cloud 2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0,0, "cloud 2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud 2");
  viewer->addPointCloud<PointXYZ> (cloud3, "cloud 3");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1.0,0, "cloud 3");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud 3");
  viewer->addPointCloud<PointXYZ> (cloud4, "cloud 4");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,1.0, "cloud 4");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud 4");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

/**
 * @brief DangerousAreaRecognition::cloudCallback
 * Callback for incoming kinect point clouds. Every time a point cloud is received the dangerous area
 * algorithm by X () is executed. Also publishes every dangerous area found.
 */
void DangerousAreaRecognition::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    finished = false;

    // Obtain input point cloud from topic message
    PointCloud<PointXYZ>::Ptr input_cloud(new PointCloud<PointXYZ>), tilt_dangerous_areas(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr tilt_floor_cloud(new PointCloud<PointXYZ>), tilt_obstacle_cloud(new PointCloud<PointXYZ>);
    fromROSMsg(*msg, *input_cloud);

    // Nullpointer check
    if(input_cloud == nullptr){
        ROS_WARN("input cloud is a nullptr.");
        finished = true;
        return;
    }

    // Transfrorm to tilt angle
    Eigen::Affine3f tilt_transform = Eigen::Affine3f::Identity();
    //tilt_transform.rotate (Eigen::AngleAxisf (-0.785398, Eigen::Vector3f::UnitY()));
    tilt_transform.rotate (Eigen::AngleAxisf (-kinect_tilt, Eigen::Vector3f::UnitX()));
    //tilt_transform.rotate (Eigen::AngleAxisf (+kinect_tilt, Eigen::Vector3f::UnitX()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr tilt_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*input_cloud, *tilt_cloud, tilt_transform);

    // filter points under robot
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(tilt_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-kinect_height - kinect_height_offset, 100.0);
    pass.filter(*tilt_dangerous_areas);

    // filter floor points
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-kinect_height + kinect_height_offset, -kinect_height - kinect_height_offset);
    pass.filter(*tilt_floor_cloud);

    // filter points between floor and robot height
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0, -kinect_height + kinect_height_offset);
    pass.filter(*tilt_obstacle_cloud);

    // Transform back tilt angle send point cloud to marker publisher
    sensor_msgs::PointCloud2 output_msg;
    Eigen::Affine3f tilt_back_transform = Eigen::Affine3f::Identity();
    //tilt_transform.rotate (Eigen::AngleAxisf (0.785398, Eigen::Vector3f::UnitY()));
    tilt_back_transform.rotate (Eigen::AngleAxisf (kinect_tilt, Eigen::Vector3f::UnitX()));
    //tilt_back_transform.rotate (Eigen::AngleAxisf (-kinect_tilt, Eigen::Vector3f::UnitX()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr dangerous_areas (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*tilt_dangerous_areas, *dangerous_areas, tilt_back_transform);
    toROSMsg(*dangerous_areas, output_msg);
    dangerous_pub.publish(output_msg);

    if ((int) dangerous_areas->points.size() > 0){
        ROS_WARN("Dangerous area with %d points found.", (int) dangerous_areas->points.size());
    } else {
        ROS_WARN("No dangerous area found");
    }

    if (debug){
        viewer = simpleVis(input_cloud, tilt_cloud, tilt_dangerous_areas, tilt_floor_cloud, tilt_obstacle_cloud);
        //viewer->addPlane(*coefficients, "plane");
        while (!viewer->wasStopped()){
            viewer->spinOnce(100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }

    finished = true;
}

/**
 * @brief DangerousAreaRecognition::spin
 * ROS node controller.
 */
void DangerousAreaRecognition::spin(){
    ros::Rate lr(5);
    while(ros::ok()){
      try {
        if(finished){
            ros::spinOnce();
        }
        lr.sleep();
      } catch(std::runtime_error& err){
        ROS_ERROR("%s: %s", node_name.c_str(), err.what());
        std::exit(1);
      }
    }
    ROS_ERROR("Ros not ok, dangerous_area_detector finished.");
}

