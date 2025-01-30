#ifndef TF2_BROADCASTER_H
#define TF2_BROADCASTER_H

#include <ros/ros.h>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class TF2Broadcaster
{
public:
    // Constructor
    TF2Broadcaster(const ros::NodeHandle& nh, std::string sub_topic_name, int sub_buffer_size, std::string parent_frame_id, std::string child_frame_id);
    
    // Public Methods
    void tf2Callback(const nav_msgs::Odometry::ConstPtr &msg);
    
private:
    // Private Members
    ros::NodeHandle nh_;
    ros::Subscriber tf2_sub_;

    tf2_ros::TransformBroadcaster broadcaster_;
    geometry_msgs::TransformStamped transform_msg_;
};

#endif // TF2_BROADCASTER_H