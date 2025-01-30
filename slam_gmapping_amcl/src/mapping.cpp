#include <ros/ros.h>

#include <slam_gmapping_amcl/TF2Broadcaster.h>

int main(int argc, char **argv)
{
    // Initialize ROS node and NodeHandle
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle nh;

    // Buffer size for subscribers
    int sub_buffer_size = 1000;

    // Use TF2Broadcaster class with necessary parameters:
    // "/odom" subscribe topic for reading the robot's current odometry data
    // "odom" as the parent frame ID
    // "base_link" as the child frame ID
    TF2Broadcaster tfbroad(nh, "/odom", sub_buffer_size, "odom", "base_link");

    // Keep the node running and process callbacks
    ros::spin();

    return 0;
}
