#include <ros/ros.h>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <slam_gmapping_amcl/TF2Broadcaster.h>

TF2Broadcaster::TF2Broadcaster(const ros::NodeHandle& nh, 
                               std::string sub_topic_name, 
                               int sub_buffer_size, 
                               std::string parent_frame_id, 
                               std::string child_frame_id)
    : nh_(nh)
{
    // Subscribe to odometry topic
    tf2_sub_ = nh_.subscribe(sub_topic_name, sub_buffer_size, &TF2Broadcaster::tf2Callback, this);

    // Initialize transform message with frame IDs
    transform_msg_.header.frame_id = parent_frame_id;
    transform_msg_.child_frame_id = child_frame_id;
}

void TF2Broadcaster::tf2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Update timestamp for transform message
    transform_msg_.header.stamp = msg->header.stamp;

    // Set position (x, y, z) values from odometry message
    transform_msg_.transform.translation.x = msg->pose.pose.position.x;
    transform_msg_.transform.translation.y = msg->pose.pose.position.y;
    transform_msg_.transform.translation.z = msg->pose.pose.position.z;
    // transform_msg_.transform.translation.z = 0.0;  // Uncomment for assuming no z translation

    // Set rotation (quaternion) values from odometry message
    transform_msg_.transform.rotation.x = msg->pose.pose.orientation.x;
    transform_msg_.transform.rotation.y = msg->pose.pose.orientation.y;
    transform_msg_.transform.rotation.z = msg->pose.pose.orientation.z;
    transform_msg_.transform.rotation.w = msg->pose.pose.orientation.w;

    // Broadcast the dynamic transform message
    broadcaster_.sendTransform(transform_msg_);
}
