#include <ros/ros.h>

#include <omni_robot_odom/WheelControlPubSub.h>

int main(int argc, char **argv)
{
    // Initialize ROS node and NodeHandle
    ros::init(argc, argv, "wheelcontrol_node");
    ros::NodeHandle nh;

    // Buffer size
    int pub_buffer_size = 10000;
    int sub_buffer_size = 1000;

    // Use KinematicsPubSub class with necessary parameters:
    // "/wheel_rpm" publish topic for publishing the robot's current wheels data
    // "/cmd_vel" subscribe topic for reading the robot's current velocity data in the base_link frame
    WheelControlPubSub wheelInfo(nh, "/wheel_rpm", "/cmd_vel", pub_buffer_size, sub_buffer_size);

    // Keep the node running and process callbacks
    ros::spin();

    return 0;
}
