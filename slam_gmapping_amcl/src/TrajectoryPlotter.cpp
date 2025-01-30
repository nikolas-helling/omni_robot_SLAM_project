#include <ros/ros.h>
#include <string>
#include <vector>
#include <ros/package.h>

#include <opencv2/opencv.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <slam_gmapping_amcl/SaveMap.h>
#include <slam_gmapping_amcl/TrajectoryPlotter.h>

TrajectoryPlotter::TrajectoryPlotter(const ros::NodeHandle& nh, 
                                     const std::string& amcl_topic, 
                                     const std::string& map_data_topic, 
                                     int sub_buffer_size)
    : nh_(nh), current_pose_(), last_pose_(), map_data_(), map_image_(), trajectory_image_(), cwd_("")
{
    // Set initial pose values
    current_pose_.pose.pose.position.x = 0;
    current_pose_.pose.pose.position.y = 0;

    // Subscribe to the AMCL pose topic and map metadata topic
    amcl_subscriber_ = nh_.subscribe(amcl_topic, sub_buffer_size, &TrajectoryPlotter::amclCallback, this);
    map_data_subscriber_ = nh_.subscribe(map_data_topic, sub_buffer_size, &TrajectoryPlotter::mapDataCallback, this);

    // Advertise the service for saving map data
    map_saver_ = nh_.advertiseService("save_trajectory_service", &TrajectoryPlotter::serviceCallback, this);

    // Get the current working directory using ROS package path
    cwd_ = ros::package::getPath("slam_gmapping_amcl");

    // Load map image
    map_image_ = cv::imread(cwd_ + "/maps/final_map.pgm", cv::IMREAD_UNCHANGED);
    if (map_image_.empty()) {
        ROS_ERROR("Failed to load base map image.");
    }

    // Initialize trajectory image as map image
    trajectory_image_ = map_image_;
}

void TrajectoryPlotter::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose)
{
    // Store current pose into last_pose
    last_pose_ = current_pose_;

    // Update current pose using data from the amcl_pose
    current_pose_ = *amcl_pose;

    // Call drawTrajectory to draw line between last pose and current pose
    drawTrajectory(current_pose_, last_pose_);
}

void TrajectoryPlotter::mapDataCallback(const nav_msgs::MapMetaData::ConstPtr& map_info)
{
    // Update map data with received map information
    map_data_ = *map_info;
}

bool TrajectoryPlotter::serviceCallback(slam_gmapping_amcl::SaveMap::Request& req, slam_gmapping_amcl::SaveMap::Response& res)
{
    // Save image based on request
    saveImage(req.save_request);
    res.save_response = "Trajectory Save Successful";

    // Log successful save location
    ROS_INFO_STREAM("Map + Current Trajectory saved at: " + cwd_ + "/maps" + "/" + req.save_request + ".pgm");

    return true;
}

void TrajectoryPlotter::drawTrajectory(const geometry_msgs::PoseWithCovarianceStamped& curr, 
                                       const geometry_msgs::PoseWithCovarianceStamped& past)
{
    ROS_INFO_STREAM("Drawing trajectory");

    // Extract map metadata
    double n_pix_x = map_data_.width;
    double n_pix_y = map_data_.height;
    double resolution = map_data_.resolution;
    geometry_msgs::Pose origin = map_data_.origin;

    // Calculate origin offset
    double x0 = -origin.position.x;
    double y0 = n_pix_y * resolution + origin.position.y;

    // Calculate pixel positions for start and end points
    cv::Point start_point((past.pose.pose.position.x + x0) / resolution, (-past.pose.pose.position.y + y0) / resolution);
    cv::Point end_point((curr.pose.pose.position.x + x0) / resolution, (-curr.pose.pose.position.y + y0) / resolution);

    // Choose line properties
    int thickness = 2;
    int line_type = 8;
    cv::Scalar traj_color(128, 0, 0);  // RGB color

    // Draw trajectory line
    cv::line(trajectory_image_, start_point, end_point, traj_color, thickness, line_type);
}

void TrajectoryPlotter::saveImage(const std::string& image_name)
{
    // Check for empty image 
    if (trajectory_image_.empty()) {
        ROS_ERROR_STREAM("Failed to save image. The trajectory image is empty.");
        return;
    }

    // Save image with user defined name
    std::string file_path = cwd_ + "/maps/" + image_name + ".pgm";
    if (!cv::imwrite(file_path, trajectory_image_)) {
        ROS_ERROR_STREAM("Failed to save image to: " + file_path);
    } else {
        ROS_INFO_STREAM("Image saved to: " + file_path);
    }
};