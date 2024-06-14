#include <ros/ros.h>
#include <potbot_lib/Utility.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

// using namespace visualization_msgs;

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM("Marker is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "interactive_marker_node");
    ros::NodeHandle n;

    // Create an Interactive Marker Server
    interactive_markers::InteractiveMarkerServer server("simple_marker");

    // Create an interactive marker
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = "my_marker";
    int_marker.description = "Simple Interactive Marker";
    int_marker.pose = potbot_lib::utility::get_Pose(0,0,0,0,0,0);

    // Create a sphere marker
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.9;
    marker.scale.y = 0.45;
    marker.scale.z = 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    marker.pose = potbot_lib::utility::get_Pose(0,0,0,0,0,0);

    // Attach the sphere marker to the interactive marker
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(marker);
    int_marker.controls.push_back(control);

    // Add a control for moving the marker
    visualization_msgs::InteractiveMarkerControl move_control;
    move_control.name = "move_x";
    move_control.orientation = potbot_lib::utility::get_Pose(0,0,0,0,M_PI_2,0).orientation;
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(move_control);

    // Add the interactive marker to the server
    server.insert(int_marker, &processFeedback);

    // Apply changes and start the server
    server.applyChanges();

    ros::spin();
}
