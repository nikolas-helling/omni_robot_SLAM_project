#include <ros/ros.h>

#include <omni_robot_odom/KinematicsPubSub.h>

int main(int argc, char **argv)
{
    // Initialize ROS node and NodeHandle
    ros::init(argc, argv, "kinematics_node");
    ros::NodeHandle nh;

    // Buffer size
    int pub_buffer_size = 1000;
    int sub_buffer_size = 1000;

    // Use KinematicsPubSub class with necessary parameters:
    // "/cmd_vel" publish topic for publishing the robot's current velocity data in base_link frame
    // "/wheel_states" subscribe topic for reading the robot's current wheels data
    KinematicsPubSub velConv(nh, "/cmd_vel", "/wheel_states", pub_buffer_size, sub_buffer_size);

    // Keep the node running and process callbacks
    ros::spin();

    return 0;
}
