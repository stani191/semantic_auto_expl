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

#include "semantic_mapping/OpenDoorRecognitionApp.h"

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
}

/**
 * @brief DangerousAreaRecognition::~DangerousAreaRecognition
 * Destructor.
 */
DangerousAreaRecognition::~DangerousAreaRecognition(){
    ROS_INFO("Open door recognition node terminated.");
}


/**
 * @brief DangerousAreaRecognition::cloudCallback
 * Callback for incoming kinect point clouds. Every time a point cloud is received the dangerous area
 * algorithm by X () is executed. Also publishes every dangerous area found.
 */
void DangerousAreaRecognition::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    finished = false;

    // Obtain input point cloud from topic message
    ros::Time cloudtime = ros::Time::now();
    PointCloud<PointXYZ>::Ptr input_cloud(new PointCloud<PointXYZ>), filtered_plane (new PointCloud<PointXYZ>);
    fromROSMsg(*msg, *input_cloud);

    // Nullpointer check
    if(input_cloud == nullptr){
        ROS_WARN("input cloud is a nullptr.");
        finished = true;
        return;
    }
    // filter points under robot
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-10.0, 0.0);

    // send point cloud to marker publisher
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
        } else {
            ROS_INFO("Dagerous area recognition node not finished.");
        }
        lr.sleep();
      } catch(std::runtime_error& err){
        ROS_ERROR("%s: %s", node_name.c_str(), err.what());
        std::exit(1);
      }
    }
    ROS_ERROR("Ros not ok, dangerous_area_detector finished.");
}

