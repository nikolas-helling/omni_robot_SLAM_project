#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class PathPublisher
{
public:
    // Constructor
    PathPublisher(const ros::NodeHandle& nh, 
                  const std::string& pub1_topic, 
                  const std::string& pub2_topic, 
                  const std::string& sub1_topic,
                  const std::string& sub2_topic,
                  int pub_buffer_size, 
                  int sub_buffer_size)
        : nh_(nh)
    {
        // Advertise publishers
        odom_path_pub_ = nh_.advertise<nav_msgs::Path>(pub1_topic, pub_buffer_size);
        gt_path_pub_ = nh_.advertise<nav_msgs::Path>(pub2_topic, pub_buffer_size);

        // Subscribe to topics
        odom_sub_ = nh_.subscribe(sub1_topic, sub_buffer_size, &PathPublisher::odomCallback, this);
        gt_sub_ = nh_.subscribe(sub2_topic, sub_buffer_size, &PathPublisher::gtCallback, this);
        
        // Initialize paths
        odom_path_.header.frame_id = "map";
        gt_path_.header.frame_id = "map";
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {   
        // Save current pose data from odometry
        geometry_msgs::PoseStamped odom_pose;
        odom_pose.header = msg->header;
        odom_pose.pose = msg->pose.pose;

        // Add estimated pose to the path
        odom_path_.header.stamp = odom_pose.header.stamp;
        odom_path_.poses.push_back(odom_pose);

        // Publish the odometry based path
        odom_path_pub_.publish(odom_path_);
    }

    void gtCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {   
        // Get current ground truth pose in map frame
        geometry_msgs::PoseStamped gt_pose = *msg;
        gt_pose.pose.position.z = 0.0;
        
        // Add ground truth pose to the path
        gt_path_.header.stamp = gt_pose.header.stamp;
        gt_path_.poses.push_back(gt_pose);

        // Publish the ground truth path
        gt_path_pub_.publish(gt_path_);
    }

private:
    // Private Members
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber gt_sub_;
    ros::Publisher odom_path_pub_;
    ros::Publisher gt_path_pub_;

    nav_msgs::Path odom_path_;
    nav_msgs::Path gt_path_;
};

int main(int argc, char** argv)
{
    // Initialize ROS node and NodeHandle
    ros::init(argc, argv, "path_pub_node");
    ros::NodeHandle nh;

    // Buffer size
    int pub_buffer_size = 1000;
    int sub_buffer_size = 1000;
    
    // Use PathPublisher class with necessary parameters:
    // "/odom_path" publish topic for publishing the robot's odometry based path
    // "/gt_path" publish topic for publishing the the robot's ground truth path
    // "/odom_map" subscribe topic for reading the robot's current odometry pose in map frame
    // "/robot/pose" subscribe topic for reading the robot's ground truth pose in map frame
    PathPublisher pathPub(nh, "/odom_map_path", "/gt_path", "/odom_map", "/robot/pose", pub_buffer_size, sub_buffer_size);

    // Keep the node running and process callbacks
    ros::spin();

    return 0;
}
