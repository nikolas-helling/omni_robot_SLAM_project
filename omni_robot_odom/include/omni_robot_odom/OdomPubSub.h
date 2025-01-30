#ifndef ODOM_PUB_SUB_H
#define ODOM_PUB_SUB_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <deque>

#include <tf2/LinearMath/Quaternion.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include <omni_robot_odom/integrationConfig.h>
#include <omni_robot_odom/ResetPose.h> 

class OdomPubSub
{
public:
    // Constructor
    OdomPubSub(const ros::NodeHandle& nh,
               const std::string& pub1_topic, 
               const std::string& pub2_topic, 
               const std::string& sub_topic, 
               int pub_buffer_size, 
               int sub_buffer_size);

    // Public Methods
    void subCallback(const geometry_msgs::TwistStamped::ConstPtr& twist);
    bool serviceCallback(omni_robot_odom::ResetPose::Request& req, omni_robot_odom::ResetPose::Response& res);
    void paramCallback(omni_robot_odom::integrationConfig& config, uint32_t level);
    std::vector<double> getInitPose() const;

private:
    // Private Members
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher pub_GT_;
    ros::Subscriber sub_;
    ros::ServiceServer reset_pose_service_;

    dynamic_reconfigure::Server<omni_robot_odom::integrationConfig> dyn_server_;
    dynamic_reconfigure::Server<omni_robot_odom::integrationConfig>::CallbackType dyn_callback_;
    int integration_selector_ = 0;

    double start_odom_x_ = 0.0, start_odom_y_ = 0.0, start_odom_theta_ = 0.0;
    double init_x_, init_y_, init_theta_;
    std::vector<double> init_pose_;
    bool params_fetched_ = false;
    nav_msgs::Odometry current_pose_;
    std::deque<geometry_msgs::TwistStamped::ConstPtr> buffer_;

    // Private Methods
    nav_msgs::Odometry updateOdomPose(double Ts, const std::vector<double>& body_vel);
    nav_msgs::Odometry transformOdomPose(nav_msgs::Odometry& odom_msg);
    std::vector<double> getParams(ros::NodeHandle& nh);
};

#endif // ODOM_PUB_SUB_H

