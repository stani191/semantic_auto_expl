/**
 * MarkerPublisher.cpp
 * Gathers semantic information which is visualized in rviz using markers. Semantic
 * information includes open doors, closed doors and dangerous areas.
 *
 * @author Matej Stanic (m.stanic@student.uibk.ac.at)
 * @version 1.0
 * @date 03.06.2017
 */

#include "semantic_mapping/MarkerPublisher.h"

/**
 * @brief MarkerPublisher::MarkerPublisher
 * Constructor. Registers subscribers for semantic information and publishers for rviz markers.
 */
MarkerPublisher::MarkerPublisher(){
    input_open_door_sub = n.subscribe("door/open_door", 1, &MarkerPublisher::openDoorCallback, this);
    //input_closed_door_sub = n.subscribe("door/closed_door", 1, &MarkerPublisher::closedDoorCallback, this);
    nav_target_sub = n.subscribe("move_base_simple/goal", 1, &MarkerPublisher::navTargetCallback, this);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ROS_INFO("Marker publisher node started.");
}

/**
 * @brief MarkerPublisher::~MarkerPublisher
 * Destructor.
 */
MarkerPublisher::~MarkerPublisher(){
    ROS_INFO("Marker publisher node terminated.");
}

/**
 * @brief MarkerPublisher::midPoint
 * Calculates the midpoint between two points.
 */
geometry_msgs::Point MarkerPublisher::midPoint(const geometry_msgs::Point p1, const geometry_msgs::Point p2) {
    geometry_msgs::Point mid;
    mid.x = (p1.x + p2.x)/2;
    mid.y = (p1.y + p2.y)/2;
    mid.z = (p1.z + p2.z)/2;

    return mid;
}

/**
 * @brief MarkerPublisher::euclideanDistance
 * Calculates the Euclidean distance between two points.
 */
double MarkerPublisher::euclideanDistance(const geometry_msgs::Point p1, const geometry_msgs::Point p2) {
    return (sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2)));
}

/**
 * @brief MarkerPublisher::deleteDoorMarker
 * @param pose
 * @param doorState
 * @param scale
 */
void MarkerPublisher::deleteDoorMarker(int id){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();

    marker.ns = "doors";
    marker.id = id;

    marker.action = visualization_msgs::Marker::DELETE;
    while (marker_pub.getNumSubscribers() < 1){
        if (!ros::ok()){
            return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker);
}

/**
 * @brief MarkerPublisher::addDoorMarker
 * @param pose
 * @param doorState
 * @param scale
 */
void MarkerPublisher::addDoorMarker(const geometry_msgs::Point pose, std::string doorState, double scale, int id){
    // 3. Publish the door marker - Line strip
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();

    marker.ns = "doors";
    marker.id = id;

    marker.action = visualization_msgs::Marker::ADD;

    if (doorState.compare("open") == 0){
        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    } else if (doorState.compare("closed") == 0){
        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    } else {
        marker.color.a = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        ROS_WARN("Unknown door state.");
    }

    std::stringstream ss;
    ss << id;
    marker.text = "door " + ss.str();

    marker.lifetime = ros::Duration();

    uint32_t marker_shape = visualization_msgs::Marker::CYLINDER;
    marker.type = marker_shape;
    marker.pose.position.x = pose.x;
    marker.pose.position.y = pose.y;
    marker.pose.position.z = pose.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = 1;

    while (marker_pub.getNumSubscribers() < 1){
        if (!ros::ok()){
            return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker);
}

/**
 * @brief MarkerPublisher::replaceDoor
 * Replaces a door with a new one in the door list (world coordinates). Also updates the markers.
 * @param oldDoor: door to be replaced
 * @param newDoor: new door which replaces old door
 */
void MarkerPublisher::replaceDoor(semantic_mapping_msgs::DoorMessage oldDoor, semantic_mapping_msgs::DoorMessage newDoor){
    // Remove old door from door list
    doors.erase(std::remove_if(doors.begin(), doors.end(),
                         [oldDoor](semantic_mapping_msgs::DoorMessage& elem){ return elem.header.seq == oldDoor.header.seq;} ),doors.end());

    // Add new door to door list
    doors.push_back(newDoor);

    // Remove old door marker
    deleteDoorMarker(oldDoor.header.seq);

    // Add new door marker
    geometry_msgs::PointStamped pt_new_start_map;
    geometry_msgs::PointStamped pt_new_end_map;
    listener.waitForTransform("world","map", newDoor.header.stamp, ros::Duration(2.0));
    listener.transformPoint("map", newDoor.pt_start, pt_new_start_map);
    listener.transformPoint("map", newDoor.pt_end, pt_new_end_map);
    addDoorMarker(midPoint(pt_new_start_map.point,pt_new_end_map.point), newDoor.state, euclideanDistance(pt_new_start_map.point,pt_new_end_map.point), newDoor.header.seq);
    ROS_INFO("Replaced door with world coords: start: x=%f, y=%f, z=%f | end: x=%f, y=%f, z=%f | width=%f", newDoor.pt_start.point.x, newDoor.pt_start.point.y, newDoor.pt_start.point.z, newDoor.pt_end.point.x, newDoor.pt_end.point.y, newDoor.pt_end.point.z, euclideanDistance(newDoor.pt_start.point, newDoor.pt_end.point));

}

/**
 * @brief MarkerPublisher::updateNavTargetMarker
 * @param pose
 */
void MarkerPublisher::updateNavTargetMarker(const geometry_msgs::Point pose){

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "target";
    // id must me always the same in order to overwrite old targets
    marker.id = 1;
    marker.action = visualization_msgs::Marker::ADD;

    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.text = "target";
    marker.lifetime = ros::Duration();

    uint32_t marker_shape = visualization_msgs::Marker::CYLINDER;
    marker.type = marker_shape;
    marker.pose.position.x = pose.x;
    marker.pose.position.y = pose.y;
    marker.pose.position.z = pose.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    while (marker_pub.getNumSubscribers() < 1){
        if (!ros::ok()){
            return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker);
}

/**
 * @brief MarkerPublisher::isDuplicate
 * Checks whether a door was already recognized in the past.
 * @param msg: door message
 * @returns true, if the door was already recognized.
 */
bool MarkerPublisher::isDuplicate(semantic_mapping_msgs::DoorMessage msg){
    bool duplicate = false;
    for (semantic_mapping_msgs::DoorMessage& door : doors) {
        if(euclideanDistance(midPoint(msg.pt_end.point,msg.pt_start.point), midPoint(door.pt_end.point,door.pt_start.point)) < euclideanDistance(msg.pt_end.point,msg.pt_start.point)){
            if(msg.quality < door.quality){
                replaceDoor(door,msg);
            } else {
                ROS_INFO("Duplicate door detected with start: x=%f, y=%f, z=%f | end: x=%f, y=%f, z=%f", msg.pt_start.point.x, msg.pt_start.point.y, msg.pt_start.point.z, msg.pt_end.point.x, msg.pt_end.point.y, msg.pt_end.point.z);
            }
            duplicate = true;
        }
    }
    return duplicate;
}

/**
 * @brief MarkerPublisher::openDoorCallback
 * Callback for incoming open doors. Publishes sphere markers for rviz for every open door.
 */
void MarkerPublisher::openDoorCallback(semantic_mapping_msgs::DoorMessage msg){
    // 1. Translate door start/end points into map/world coordinates
    geometry_msgs::PointStamped pt_start_map;
    geometry_msgs::PointStamped pt_start_world;
    geometry_msgs::PointStamped pt_end_map;
    geometry_msgs::PointStamped pt_end_world;

    listener.waitForTransform("map","kinect_depth_optical_frame", msg.header.stamp, ros::Duration(2.0));
    listener.transformPoint("map", msg.pt_start, pt_start_map);
    listener.transformPoint("map", msg.pt_end, pt_end_map);

    listener.waitForTransform("world","kinect_depth_optical_frame", msg.header.stamp, ros::Duration(2.0));
    listener.transformPoint("world", msg.pt_start, pt_start_world);
    listener.transformPoint("world", msg.pt_end, pt_end_world);

    // 2. Save door in list with world coordinates if it is not a duplicate
    msg.pt_start = pt_start_world;
    msg.pt_start.header.frame_id = "world";
    msg.pt_end = pt_end_world;
    msg.pt_end.header.frame_id = "world";

    if(!isDuplicate(msg)){
        doors.push_back(msg);
        addDoorMarker(midPoint(pt_start_map.point,pt_end_map.point), msg.state, euclideanDistance(pt_start_map.point,pt_end_map.point), msg.header.seq);
        ROS_INFO("New open door found with with world coords: start: x=%f, y=%f, z=%f | end: x=%f, y=%f, z=%f | width=%f", pt_start_world.point.x, pt_start_world.point.y, pt_start_world.point.z,pt_end_world.point.x, pt_end_world.point.y, pt_end_world.point.z, euclideanDistance(pt_start_world.point,pt_end_world.point));
    }
}

/*void MarkerPublisher::closedDoorCallback(semantic_mapping_msgs::DoorMessage msg){

}*/

/**
 * @brief MarkerPublisher::navTargetCallback
 * Callback for incoming navigation target points. Publishes a unique marker for rviz which is constantly updated.
 * @param msg
 */
void MarkerPublisher::navTargetCallback(geometry_msgs::PoseStamped msg){
    // 1. translate nav target into map/world coordinates depending on current frame_id

    if (msg.header.frame_id == "map"){
        navTargetMap = msg;

    } else if (msg.header.frame_id == "world"){
        listener.waitForTransform("map","world", msg.header.stamp, ros::Duration(2.0));
        listener.transformPose("map", msg, navTargetMap);

    } else {
        ROS_WARN("Navigation target message has neither map nor world as frame_id");
    }

    // 2. Update nav target marker
    updateNavTargetMarker(navTargetMap.pose.position);
}

/**
 * @brief MarkerPublisher::spin
 * ROS node controller.
 */
void MarkerPublisher::spin(){
    ros::Rate lr(10);
    while(ros::ok()){
        try{
            ros::spinOnce();
            lr.sleep();
        } catch(std::runtime_error& err){
            ROS_ERROR("%s: %s", node_name.c_str(), err.what());
            std::exit(1);
        }
    }
    ROS_ERROR("Ros not ok, marker_publisher finished.");
}
