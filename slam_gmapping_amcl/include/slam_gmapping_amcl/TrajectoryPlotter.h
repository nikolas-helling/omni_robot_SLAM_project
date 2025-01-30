#ifndef TRAJECTORY_PLOTTER_H
#define TRAJECTORY_PLOTTER_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <slam_gmapping_amcl/SaveMap.h>

class TrajectoryPlotter
{
public:
    // Constructor
    TrajectoryPlotter(const ros::NodeHandle& nh, 
                      const std::string& amcl_topic, 
                      const std::string& map_data_topic, 
                      int sub_buffer_size);

    // Public Methods
    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose);
    void mapDataCallback(const nav_msgs::MapMetaData::ConstPtr& map_info);
    bool serviceCallback(slam_gmapping_amcl::SaveMap::Request& req, slam_gmapping_amcl::SaveMap::Response& res);

private:
    // Private Members
    ros::NodeHandle nh_;
    ros::Subscriber amcl_subscriber_;
    ros::Subscriber map_data_subscriber_;
    ros::ServiceServer map_saver_;

    geometry_msgs::PoseWithCovarianceStamped current_pose_;
    geometry_msgs::PoseWithCovarianceStamped last_pose_;
    nav_msgs::MapMetaData map_data_;
    cv::Mat map_image_;
    cv::Mat trajectory_image_;
    std::string cwd_;

    // Private Methods
    void drawTrajectory(const geometry_msgs::PoseWithCovarianceStamped& curr, const geometry_msgs::PoseWithCovarianceStamped& past);
    void saveImage(const std::string& image_name);
};

#endif // TRAJECTORY_PLOTTER_H
