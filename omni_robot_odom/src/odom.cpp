#include <ros/ros.h>

#include <omni_robot_odom/OdomPubSub.h>
#include <omni_robot_odom/TF2Broadcaster.h>

int main(int argc, char **argv)
{   
    // Initialize ROS node and NodeHandle
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;

    // Buffer size
    int pub_buffer_size = 1000;
    int sub_buffer_size = 1000;

    // Use OdomPubSub class with necessary parameters:
    // "/odom" publish topic for publishing the robot's current odometry data in odom frame
    // "/odom_map" publish topic for publishing the robot's current odometry data in map frame
    // "/cmd_vel" subscribe topic for reading the robot's current velocity data in the base_link frame
    OdomPubSub odometry(nh, "/odom", "/odom_map", "/cmd_vel", pub_buffer_size, sub_buffer_size);

    // Use TF2Broadcaster class with necessary parameters:
    // "/odom" subscribe topic for reading the robot's current odometry data
    // "odom" as the parent frame ID
    // "base_link" as the child frame ID
    TF2Broadcaster tfbroad(nh, "/odom", sub_buffer_size, "odom", "base_link");

    // Keep the node running and process callbacks
    ros::spin();

    return 0;
}
