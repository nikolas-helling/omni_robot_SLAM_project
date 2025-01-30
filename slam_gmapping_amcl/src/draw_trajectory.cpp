#include <ros/ros.h>

#include <slam_gmapping_amcl/SaveMap.h>
#include <slam_gmapping_amcl/TrajectoryPlotter.h>

// Entry point
int main(int argc, char **argv)
{
    // Initialize ROS node and NodeHandle
    ros::init(argc, argv, "savemap_node");
    ros::NodeHandle nh;

    // Buffer size for subscribers
    int sub_buffer_size = 1000;

    // Use TrajectoryPlotter class with necessary parameters:
    // "/amcl_pose" subscribe topic for the robot's estimated pose
    // "/map_metadata" subscribe topic for the map metadata
    TrajectoryPlotter plotter(nh, "/amcl_pose", "/map_metadata", sub_buffer_size);

    // Keep the node running and process callbacks
    ros::spin();

    return 0;
}
