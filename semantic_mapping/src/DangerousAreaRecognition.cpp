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

    listener.waitForTransform("kinect_depth_optical_frame","world", ros::Time::now(), ros::Duration(2.0));
    listener.transformPoint("kinect_depth_optical_frame", kinectOriginWorld, kinectOrigin);

    kinect_height = kinectOrigin.point.y;
    ROS_INFO("Kinect sensor height = %f", kinect_height);
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
    PointCloud<PointXYZ>::Ptr input_cloud(new PointCloud<PointXYZ>), cloud_filtered(new PointCloud<PointXYZ>);
    fromROSMsg(*msg, *input_cloud);

    // Nullpointer check
    if(input_cloud == nullptr){
        ROS_WARN("input cloud is a nullptr.");
        finished = true;
        return;
    }
    // filter points under robot
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-100.0, kinect_height - kinect_height_offset);
    pass.filter(*cloud_filtered);

    // send point cloud to marker publisher
    sensor_msgs::PointCloud2 output_msg;
    toROSMsg(*cloud_filtered, output_msg);
    dangerous_pub.publish(output_msg);
    //ROS_WARN("Dangerous area published.");
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

