#include <ros/ros.h>

#include <omni_robot_odom/ResetPose.h>

int main(int argc, char **argv)
{
    // Initialize ROS node and NodeHandle
    ros::init(argc, argv, "reset_client_node");
    ros::NodeHandle nh;

    // Check for correct number of arguments
    if (argc != 4)
    {
        ROS_ERROR("Incorrect number of parameters. Specify new values for x, y, and theta.");
        return 1;
    }

    // Create service client
    ros::ServiceClient client = nh.serviceClient<omni_robot_odom::ResetPose>("reset_pose");

    // Create service object and populate client request
    omni_robot_odom::ResetPose srv;
    srv.request.new_init_pose.currentX = std::atof(argv[1]);
    srv.request.new_init_pose.currentY = std::atof(argv[2]);
    srv.request.new_init_pose.currentTheta = std::atof(argv[3]);

    // Call the service
    if (client.call(srv))
    {   
        // Log service request info
        ROS_INFO("New starting pose for odometry set correctly.");
        
        ROS_INFO_STREAM("Old Pose: (" 
                        << srv.response.old_final_pose.currentX << ", " 
                        << srv.response.old_final_pose.currentY << ", " 
                        << srv.response.old_final_pose.currentTheta << ")");
        ROS_INFO_STREAM("New Pose: (" 
                        << srv.request.new_init_pose.currentX << ", " 
                        << srv.request.new_init_pose.currentY << ", " 
                        << srv.request.new_init_pose.currentTheta << ")");
    }
    else
    {
        ROS_ERROR("Failed to call service for pose reset.");
        return 1;
    }

    return 0;
}

