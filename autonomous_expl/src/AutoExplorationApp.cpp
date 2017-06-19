/**
 * AutoExplorationApp.cpp
 * Implements autonomous target selection and navigation.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include "autonomous_expl/AutoExplorationApp.h"

using namespace cv;

/**
 * @brief AutoExplorationApp::AutoExplorationApp
 * Constructor. Subscribes to topics needed for autonomous exploration and registers the publisher
 * of new targets. Initializes robot position and kinect tilt. Performs a rotation on startup to
 * obtain a useful map and starts the autonomous exploration process.
 *
 */
AutoExplorationApp::AutoExplorationApp(){
    map_sub = n.subscribe("map", 50, &AutoExplorationApp::mapCallback, this);
    mapUpdate_sub = n.subscribe("map_updates", 50, &AutoExplorationApp::mapUpdateCallback, this);
    odom_sub = n.subscribe("odom", 1000, &AutoExplorationApp::odomCallback, this);
    move_base_status_sub = n.subscribe("move_base/status", 1, &AutoExplorationApp::moveBaseStatusCallback, this);
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    target_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);
    tilt_pub = n.advertise<std_msgs::Float64>("/tilt_controller/command", 100);
    mapClient = n.serviceClient<nav_msgs::GetMap>("/dynamic_map");
    node_name = ros::this_node::getName();

    isMoving = false;
    gettimeofday(&tp, NULL);

    // init base_link starting position of the robot
    robotPosBaseLink.header.seq = 0;
    robotPosBaseLink.header.stamp = ros::Time::now();
    robotPosBaseLink.header.frame_id = "base_link";
    robotPosBaseLink.point.x = 0.0;
    robotPosBaseLink.point.y = 0.0;
    robotPosBaseLink.point.z = 0.0;

    // wait for robot simulation to initialize
    sleep(WAIT_INIT);
    setCurrentRobotPos();
    targetWorld = robotPosWorld;
    targetMap = robotPosMap;

    // Set kinect tilt to 90°

    ROS_INFO("Setting kinect tilt...");
    std_msgs::Float64 tilt;
    tilt.data = KINECT_TILT_90DEG;
    tilt_pub.publish(tilt);
    sleep(WAIT_TILT);
    tilt_pub.publish(tilt);
    sleep(WAIT_TILT);

    ROS_INFO("AutoExploration initializazion done.");

    // make step forward and rotate 2*360° to obtain useful map
    step();
    rotate();
    sleep(3);

    // Set bool flags to start aut. exploration
    navigationActive = false;
    initFinished = true;
}

/**
 * @brief MarkerPublisher::~MarkerPublisher
 * Destructor.
 */
AutoExplorationApp::~AutoExplorationApp(){
    ROS_INFO("AutoExploration node terminated.");
}

/**
 * @brief AutoExplorationApp::mapToPixelCoords
 * Converts given robot map position (map frame) to position in map matrix (pixels)
 * @param robotPosMap - robot position in map frame
 * @return robot position in map matrix (in pixels)
 */
geometry_msgs::PointStamped AutoExplorationApp::mapToPixelCoords(geometry_msgs::PointStamped robotPosMap){
    geometry_msgs::PointStamped result;
    result.header = robotPosMap.header;
    result.point.x = (robotPosMap.point.x - mapOrigin.x) / mapResolution;
    result.point.y = (robotPosMap.point.y - mapOrigin.y) / mapResolution;
    result.point.z = (robotPosMap.point.z - mapOrigin.z) / mapResolution;

    return result;
}

/**
 * @brief AutoExplorationApp::pixelToMapCoords
 * Converts given robot map matrix position (pixels) to map position (map frame)
 * @param robotPosPixel - robot position in map matrix (in pixels)
 * @return robot position in map frame
 */
geometry_msgs::PointStamped AutoExplorationApp::pixelToMapCoords(geometry_msgs::PointStamped robotPosPixel){
    geometry_msgs::PointStamped result;
    result.header = robotPosPixel.header;
    result.point.x = robotPosPixel.point.y * mapResolution + mapOrigin.y;
    result.point.y = robotPosPixel.point.x * mapResolution + mapOrigin.x; // y and z are switched!
    result.point.z = robotPosPixel.point.z * mapResolution + mapOrigin.z;

    return result;
}

/**
 * @brief AutoExplorationApp::euclideanDistance
 * Calculates the Euclidean distance between two points.
 */
double AutoExplorationApp::euclideanDistance(const geometry_msgs::Point p1, const geometry_msgs::Point p2) {
    return (sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}

/**
 * @brief AutoExplorationApp::odomCallback
 * Odometry listener. Determines whether the robot is moving or not and sets the according boolean flag.
 */
void AutoExplorationApp::odomCallback(const nav_msgs::Odometry msg){
    if(mapAvailable && initFinished && !navigationActive){
        geometry_msgs::Vector3 angular = msg.twist.twist.angular;
        geometry_msgs::Vector3 linear = msg.twist.twist.linear;

        if(abs(linear.x) < 0.01 && abs(linear.y) < 0.01 && abs(angular.z) < 0.01 && isMoving){
            isMoving = false;
            ROS_DEBUG("Robot does not move anymore");
        } else if ((abs(linear.x) >= 0.01 || abs(linear.y) >= 0.01 || abs(angular.z) >= 0.01) && !isMoving){
            isMoving = true;
            ROS_DEBUG("Robot started to move.");
        }

        setCurrentRobotPos();

        if (!isMoving && euclideanDistance(robotPosWorld.point, targetWorld.point) < 0.5){
            //if(!selectNavTarget(dyn_map,mapResolution)){
                ROS_INFO("No active target");
                //ros::shutdown;
            //}
                navigate();
        } else if (euclideanDistance(robotPosWorld.point, targetWorld.point) < 0.5){
            ROS_INFO("Target == robot position, nothing to do.");
            ros::shutdown;
        }
    }
}

/**
 * @brief AutoExplorationApp::mapCallback
 * Map listener. Whenever a new map is generated by gmapping it is saved in OpenCV Mat type.
 * The map is then used as input for the target selection algorithm.
 */
void AutoExplorationApp::mapCallback(const nav_msgs::OccupancyGrid msg){
    mapOrigin = msg.info.origin.position;
    mapResolution = msg.info.resolution;
    mapWidth = msg.info.width;
    mapHeight = msg.info.height;
    ROS_INFO("New map obtained: Origin [%f, %f, %f] | Orientation [%f, %f, %f, %f]", mapOrigin.x, mapOrigin.y,
             mapOrigin.z, msg.info.origin.orientation.x, msg.info.origin.orientation.y,
             msg.info.origin.orientation.z, msg.info.origin.orientation.w);

    // Convert the map into a mat which is used by OpenCV for target selection
    dyn_map = Mat::zeros( msg.info.width, msg.info.height, CV_8UC1);
    for(int i = 0; i < dyn_map.rows; i++){
        for(int j = 0; j < dyn_map.cols; j++){
            if((int) msg.data[i+msg.info.width*j] == -1){
               dyn_map.at<uchar>(i,j) = 100;
            } else if((int) msg.data[i+msg.info.width*j] == 100){
               dyn_map.at<uchar>(i,j) = 0;
            } else if((int) msg.data[i+msg.info.width*j] == 0){
               dyn_map.at<uchar>(i,j) = 255;
            } else {
               ROS_ERROR("map conversion error...");
            }
       }
    }

    // Select new target of updated map
    /*if(!selectNavTarget(dyn_map,mapResolution)){
        ROS_INFO("No targets available. Finished.");
        ros::shutdown;
    }*/
    mapAvailable = true;
    //navigate();
}

/**
 * @brief AutoExplorationApp::mapUpdateCallback
 * Map update listener. Whenever the map is updated by gmapping the update is saved in OpenCV Mat type.
 * The updated map is then used as input for the exploration algorithm.
 */
void AutoExplorationApp::mapUpdateCallback(const map_msgs::OccupancyGridUpdate msg){
    ROS_INFO("New map update obtained: ROI [%d, %d, %d, %d]", msg.x, msg.y, msg.width, msg.height);

    // Convert the map update into a mat which is used by OpenCV for target selection
   for(int i = msg.y; i < msg.y + msg.height; i++){
        for(int j = msg.x; j < msg.x + msg.width; j++){
            if((int) msg.data[(i-msg.y)+msg.width*(j-msg.x)] == -1){
               dyn_map.at<uchar>(i,j) = 100;
            } else if((int) msg.data[(i-msg.y)+msg.width*(j-msg.x)] == 100){
               dyn_map.at<uchar>(i,j) = 0;
            } else if((int) msg.data[(i-msg.y)+msg.width*(j-msg.x)] == 0){
               dyn_map.at<uchar>(i,j) = 255;
            } else {
               ROS_ERROR("map conversion error...");
            }
       }
    }
    mapAvailable = true;
}

/**
 * @brief AutoExplorationApp::moveBaseStatusCallback
 * move_base status listener. Determines whether the target navigation is active or not. If no
 * target is active, navigation is started.
 */
void AutoExplorationApp::moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray msg){
    if(initFinished){
        int arraySize = 0;
        for(actionlib_msgs::GoalStatus s : msg.status_list){
            arraySize++;
        }

        ROS_DEBUG("Status list size: %d", arraySize);
        if(arraySize > 0){
            bool status1 = false;
            bool status4 = false;
            for(actionlib_msgs::GoalStatus s : msg.status_list){
                ROS_DEBUG("Status: %d", s.status);
                if(s.status == 4){
                    status4 = true;
                } else if(s.status == 0 || s.status == 1){
                    status1 = true;
                    navigationActive = true;
                }
            }

            if(status4 && !status1){
                ROS_DEBUG("Status 4 and not 1 -> rotate + navigate");
                navigationActive = true;
                rotate();
                sleep(3);
                if(!selectNavTarget(dyn_map,mapResolution)){
                    ROS_INFO("No targets available. Finished.");
                    ros::shutdown;
                }
                navigationActive = false;
                navigate();

            } else if(!status1){
                ROS_INFO("No active target - select new target and start to navigate");
                navigationActive = false;
                if(!selectNavTarget(dyn_map,mapResolution)){
                    ROS_INFO("No targets available. Finished.");
                    ros::shutdown;
                }
                navigate();
            }

        } else {
            ROS_INFO("No active target - select new target and start to navigate");
            if(!selectNavTarget(dyn_map,mapResolution)){
                ROS_INFO("No targets available. Finished.");
                ros::shutdown;
            }
            navigationActive = false;
            navigate();
        }
    }
}

/**
 * @brief AutoExplorationApp::setCurrentRobotPos
 * Sets the current robot position in map and world frame
 */
void AutoExplorationApp::setCurrentRobotPos(){
    robotPosBaseLink.header.seq = robotPosBaseLink.header.seq++;
    robotPosBaseLink.header.stamp = ros::Time::now();
    try {
        tf_listener.waitForTransform("base_link","map", robotPosBaseLink.header.stamp, ros::Duration(3.5));
        tf_listener.transformPoint("map", robotPosBaseLink, robotPosMap);
        tf_listener.waitForTransform("base_link","world", robotPosBaseLink.header.stamp, ros::Duration(3.5));
        tf_listener.transformPoint("world", robotPosBaseLink, robotPosWorld);

    } catch(tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

/**
 * @brief AutoExplorationApp::stop
 * Stops the robot move.
 */
void AutoExplorationApp::stop(){
    if (ros::ok()){
        geometry_msgs::Twist msg;
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
        vel_pub.publish(msg);
        ROS_INFO("Published stop");
        isMoving = false;
    }
}

/**
 * @brief AutoExplorationApp::rotate
 * Rotates the robot move.
 */
void AutoExplorationApp::rotate(){
    if (ros::ok()){
        geometry_msgs::Twist msg;
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = PI/8;

        double current_angle = 0;
        double t0 = ros::Time::now().toSec();
        double t1;

        vel_pub.publish(msg);
        ROS_INFO("Published rotation");

        while(current_angle < 2*PI){
            t1 = ros::Time::now().toSec();
            current_angle = (PI/8)*(t1-t0);
            sleep(1);
        }

        stop();
    }
}

/**
 * @brief AutoExplorationApp::step
 * Moves the robot one "step" forward.
 */
void AutoExplorationApp::step(){
    if (ros::ok()){
        geometry_msgs::Twist msg;
        msg.linear.x = 0.5;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
        vel_pub.publish(msg);
        ROS_INFO("Published step forward");
        sleep(1);
        stop();
    }
}

/**
 * @brief AutoExplorationApp::navigate
 * Function which publishes a point to topic "move_base_simple/goal" in order
 * to let the robot navigate it.
 * @param target
 */
bool AutoExplorationApp::navigate(geometry_msgs::PointStamped target){
    if (ros::ok()){
        geometry_msgs::PoseStamped msg;
        msg.header = target.header;
        msg.pose.position = target.point;
        msg.pose.orientation.w = 1.0;

        target_pub.publish(msg);
        ROS_INFO("Navigating to: %f, %f, %f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        return true;
    }
    return false;
}

/**
 * @brief AutoExplorationApp::navigate
 * Helper function for target navigation.
 * @return
 */
bool AutoExplorationApp::navigate(){
    if(!navigationActive){
        navigationActive = true;
        if(euclideanDistance(targetMap.point,robotPosMap.point) > 0.1){
            return navigate(targetMap);
        }
    } else {
        ROS_INFO("Navigation already active.");
    }
    return false;

}

/**
 * @brief AutoExplorationApp::updateMap
 * Calls gmapping's dynamic_map service to update the map used for target selection
 * @return
 */
bool AutoExplorationApp::updateMap(){
    if(initFinished){
        nav_msgs::GetMap srv;
        if(mapClient.call(srv)){
            mapCallback(srv.response.map);
            return true;
        } else {
            ROS_WARN("Service GetMap failed.");
            return false;
        }
    }
}

/**
 * @brief AutoExplorationApp::selectNavTarget
 * Determines a suitable target destination.
 *
 * @param map - current map in OpenCV Mat type
 * @param resolution - map resolution
 * @return
 */
bool AutoExplorationApp::selectNavTarget(Mat map, float resolution){
    updateMap();

    /// Step 1: Obtain accessible occupancy grid map
    setCurrentRobotPos();
    Mat dist_transform = df.distance_transform(robotPosMap.point, map, resolution);

    /// Step 2: Find good target destinations
    Mat frontiers = fe.extractFrontiers(robotPosMap.point, map, dist_transform);
    Mat frontiers_filtered = frontiers.clone();
    vector<std::pair<Point2f, Point2f>> targets = fe.extractTargets(frontiers_filtered, robotPosMap.point, dist_transform, mapWidth, mapHeight);
    if(targets.size() < 1){
        ROS_WARN("No targets found.");
        navigationActive = true;
        initFinished = false;
        return false;
    }

    /// Step 3: Evaluate and select the best target
    int targetIndex = 0;
    geometry_msgs::Point targetPoint = ts.selectTarget(targets, robotPosMap.point, dist_transform, frontiers);
    geometry_msgs::PointStamped target;
    target.header = robotPosMap.header;
    target.point = targetPoint;
    ROS_INFO("New Target (Map): [%f,%f,%f]", pixelToMapCoords(target).point.x, pixelToMapCoords(target).point.y, pixelToMapCoords(target).point.z);
    targetMap = pixelToMapCoords(target);
    targetMap.header.frame_id = "map";
    tf_listener.waitForTransform("world","map", targetMap.header.stamp, ros::Duration(3.5));
    tf_listener.transformPoint("world", targetMap, targetWorld);
    return true;
}

/**
 * @brief OpenDoorRecognitionApp::spin
 * ROS node controller.
 */
void AutoExplorationApp::spin(){
    ros::Rate lr(3);
    while(ros::ok()){
      try {
        ros::spinOnce();
        lr.sleep();
      } catch(std::runtime_error& err){
        ROS_ERROR("%s: %s", node_name.c_str(), err.what());
        std::exit(1);
      }
    }
    std::cout << "Ros not ok, finished." << std::endl;
}
