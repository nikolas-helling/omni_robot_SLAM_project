#include <ros/ros.h>
#include <string>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include <omni_robot_odom/integrationConfig.h>
#include <omni_robot_odom/ResetPose.h>
#include <omni_robot_odom/OdomPubSub.h>

OdomPubSub::OdomPubSub(const ros::NodeHandle& nh, 
                       const std::string& pub1_topic, 
                       const std::string& pub2_topic, 
                       const std::string& sub_topic, 
                       int pub_buffer_size, 
                       int sub_buffer_size)
    : nh_(nh), init_x_(0.0), init_y_(0.0), init_theta_(0.0), init_pose_({0.0, 0.0, 0.0}) 
{
    // Advertise publishers
    pub_ = nh_.advertise<nav_msgs::Odometry>(pub1_topic, pub_buffer_size);
    pub_GT_ = nh_.advertise<nav_msgs::Odometry>(pub2_topic, pub_buffer_size);

    // Subscribe to topics
    sub_ = nh_.subscribe(sub_topic, sub_buffer_size, &OdomPubSub::subCallback, this);

    // Advertise reset_pose service
    reset_pose_service_ = nh_.advertiseService("reset_pose", &OdomPubSub::serviceCallback, this);

    // Set up dynamic reconfigure server
    dyn_callback_ = boost::bind(&OdomPubSub::paramCallback, this, _1, _2);
    dyn_server_.setCallback(dyn_callback_);

    // Log initialization
    ROS_INFO("OdomPubSub initialized with publish topics: '%s', '%s'", pub1_topic.c_str(), pub2_topic.c_str());
    ROS_INFO("OdomPubSub initialized with subscribe topics: '%s'", sub_topic.c_str());
    ROS_INFO("Starting integration 2D pose for odometry (x, y, theta) in odom frame: (%f, %f, %f)", 
            start_odom_x_, start_odom_y_, start_odom_theta_);
    ROS_INFO("Starting integration 2D pose for odometry (x, y, theta) can be reset with resetclient (default: 0.0, 0.0, 0.0).");
    ROS_INFO("Waiting for data...");
}

void OdomPubSub::subCallback(const geometry_msgs::TwistStamped::ConstPtr& twist)
{   
    // Get initial pose parameters (2D: x, y, theta)
    if (!params_fetched_) {
        // Fetch parameters only once
        std::vector<double> init_params = getParams(nh_);
        init_x_ = init_params[0];
        init_y_ = init_params[1];
        init_theta_ = init_params[2];
        params_fetched_ = true;  // Set the flag to true

        // Check if all parameters are 0.0
        if (init_x_ == 0.0 && init_y_ == 0.0 && init_theta_ == 0.0) {
            ROS_WARN("No initial pose set from the data bag. Using default starting pose.");
        } else {
            ROS_INFO("Initial 2D pose (x, y, theta) in map frame (set from data bag): (%f, %f, %f)", 
                    init_x_, init_y_, init_theta_);
        }

        ROS_INFO("Odometry calculations started.");
    }

    // Use deque as data buffer
    const int MIN_BUFFER_SIZE = 1;
    const int MAX_BUFFER_SIZE = 2;
    buffer_.push_front(twist);

    if (buffer_.size() > MIN_BUFFER_SIZE and params_fetched_)
    {
        // Compute time difference and extract twist components (vx, vy, wz)
        const double t_prev = buffer_[1]->header.stamp.toSec();
        const double t_new = buffer_[0]->header.stamp.toSec();
        const double Ts = t_new - t_prev;
        std::vector<double> body_vel = {
            buffer_[1]->twist.linear.x,
            buffer_[1]->twist.linear.y,
            buffer_[1]->twist.angular.z
        };

        // Update current odometry pose through integration
        nav_msgs::Odometry odom_msg = updateOdomPose(Ts, body_vel);

        // Update twist information for current odometry message
        odom_msg.twist.twist.linear.x = body_vel[0]; // vx
        odom_msg.twist.twist.linear.y = body_vel[1]; // vy
        odom_msg.twist.twist.linear.z = 0.0;  // No vertical motion
        odom_msg.twist.twist.angular.x = 0.0; // No roll motion
        odom_msg.twist.twist.angular.y = 0.0; // No pitch motion
        odom_msg.twist.twist.angular.z = body_vel[2]; // wz

        // Publish odometry pose in the odom frame
        odom_msg.header.stamp = buffer_[0]->header.stamp;
        odom_msg.child_frame_id = "base_link";
        odom_msg.header.frame_id = "odom";
        pub_.publish(odom_msg);

        // Transform current odometry pose to map frame
        odom_msg = transformOdomPose(odom_msg);

        // Publish odometry pose in the map frame
        odom_msg.child_frame_id = "base_link";
        odom_msg.header.frame_id = "map";
        pub_GT_.publish(odom_msg);

        // Store current map frame pose
        current_pose_ = odom_msg;

        // Maintain buffer size
        if (buffer_.size() > MAX_BUFFER_SIZE)
        {
            buffer_.pop_back();
        }
    }
}

bool OdomPubSub::serviceCallback(omni_robot_odom::ResetPose::Request& req, omni_robot_odom::ResetPose::Response& res)
{
    // Log current pose values
    ROS_INFO_STREAM("User changed initial odometry pose.");
    ROS_INFO_STREAM("Last Pose: "
                    << "X = " << std::to_string(current_pose_.pose.pose.position.x) << " "
                    << "Y = " << std::to_string(current_pose_.pose.pose.position.y) << " "
                    << "Theta = " << std::to_string(current_pose_.pose.pose.orientation.z));

    // Store the current pose before reset
    res.old_final_pose.currentX = current_pose_.pose.pose.position.x;
    res.old_final_pose.currentY = current_pose_.pose.pose.position.y;
    res.old_final_pose.currentTheta = current_pose_.pose.pose.orientation.z;

    // Set new pose from client request
    current_pose_.pose.pose.position.x = req.new_init_pose.currentX;
    current_pose_.pose.pose.position.y = req.new_init_pose.currentY;
    current_pose_.pose.pose.orientation.z = req.new_init_pose.currentTheta;

    // Update starting odometry pose
    start_odom_x_ = req.new_init_pose.currentX;
    start_odom_y_ = req.new_init_pose.currentY;
    start_odom_theta_ = req.new_init_pose.currentTheta;

    // Log new pose
    ROS_INFO_STREAM("New Pose: "
                    << "X = " << std::to_string(current_pose_.pose.pose.position.x) << " "
                    << "Y = " << std::to_string(current_pose_.pose.pose.position.y) << " "
                    << "Theta = " << std::to_string(current_pose_.pose.pose.orientation.z));

    return true;
}

void OdomPubSub::paramCallback(omni_robot_odom::integrationConfig& config, uint32_t level)
{
  // Changing the integration method parameter
  integration_selector_ = config.integration_selector;

  // Log new integration method
  switch (integration_selector_) {
    case 0:
      ROS_INFO("Euler method selected.");
      break;
    case 1:
      ROS_INFO("Runge-Kutta 2 method selected.");
      break;
    case 2:
      ROS_INFO("Runge-Kutta 4 method selected.");
      break;
    default:
      ROS_WARN("Invalid integration selector. Using Euler integration.");
      integration_selector_ = 0; 
      break;
  }
  ROS_INFO("Set integration params with rqt_reconfigure.");
}

nav_msgs::Odometry OdomPubSub::updateOdomPose(double Ts, const std::vector<double>& body_vel)
{   
    // Initialization
    nav_msgs::Odometry odom_msg;
    const double vx = body_vel[0];
    const double vy = body_vel[1];
    const double wz = body_vel[2];

    // Set integration starting pose
    double x = start_odom_x_, y = start_odom_y_, theta = start_odom_theta_;

    // Normalize theta to [-π, π] to prevent angle drift
    const double PI = 3.14159265358979; // Pi constant
    theta = std::fmod(theta + PI, 2 * PI) - PI;

    // Precompute trigonometric values
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);

    // Integrate using user chosen approximate integration method (DEFAULT: Euler)
    switch (integration_selector_) {
        case 0: { // Euler Integration
        x += (vx * cos_theta - vy * sin_theta) * Ts;
        y += (vx * sin_theta + vy * cos_theta) * Ts;
        theta += wz * Ts;
        break;
        }
        case 1: { // RK2 Integration
        double dTheta_half = wz * Ts / 2.0;
        x += (vx * std::cos(theta + dTheta_half) - vy * std::sin(theta + dTheta_half)) * Ts;
        y += (vx * std::sin(theta + dTheta_half) + vy * std::cos(theta + dTheta_half)) * Ts;
        theta += wz * Ts;
        break;
        }
        case 2: { // RK4 Integration
        double k1x = vx * cos_theta - vy * sin_theta;
        double k1y = vx * sin_theta + vy * cos_theta;
        double k1theta = wz;

        double k2x = vx * std::cos(theta + k1theta * Ts / 2.0) - vy * std::sin(theta + k1theta * Ts / 2.0);
        double k2y = vx * std::sin(theta + k1theta * Ts / 2.0) + vy * std::cos(theta + k1theta * Ts / 2.0);
        double k2theta = wz;

        double k3x = vx * std::cos(theta + k2theta * Ts / 2.0) - vy * std::sin(theta + k2theta * Ts / 2.0);
        double k3y = vx * std::sin(theta + k2theta * Ts / 2.0) + vy * std::cos(theta + k2theta * Ts / 2.0);
        double k3theta = wz;

        double k4x = vx * std::cos(theta + k3theta * Ts) - vy * std::sin(theta + k3theta * Ts);
        double k4y = vx * std::sin(theta + k3theta * Ts) + vy * std::cos(theta + k3theta * Ts);
        double k4theta = wz;

        x += (k1x + 2.0 * k2x + 2.0 * k3x + k4x) * Ts / 6.0;
        y += (k1y + 2.0 * k2y + 2.0 * k3y + k4y) * Ts / 6.0;
        theta += (k1theta + 2.0 * k2theta + 2.0 * k3theta + k4theta) * Ts / 6.0;
        break;
        }
        default: { // Euler Integration
        ROS_WARN("Invalid integration selector. Using Euler integration.");
        x += (vx * cos_theta - vy * sin_theta) * Ts;
        y += (vx * sin_theta + vy * cos_theta) * Ts;
        theta += wz * Ts;
        break;
        }
    }

    // Set integration starting pose for next cycle
    start_odom_x_ = x,
    start_odom_y_ = y;
    start_odom_theta_ = theta;

    // Update odometry pose (x, y, theta)
    odom_msg.pose.pose.position.x = x,
    odom_msg.pose.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    tf2::convert(q, odom_msg.pose.pose.orientation); 

    // Return odometry message
    return odom_msg;
};

nav_msgs::Odometry OdomPubSub::transformOdomPose(nav_msgs::Odometry& odom_msg)
{
    // Transform odometry pose to the map frame
    std::vector<double> init_pose = getInitPose();
    double theta0 = init_pose[2];

    // Transform position (x, y)
    double x = odom_msg.pose.pose.position.x;
    double y = odom_msg.pose.pose.position.y;

    double cos_theta0 = std::cos(theta0);
    double sin_theta0 = std::sin(theta0);

    double x_map = x * cos_theta0 - y * sin_theta0 + init_pose[0];
    double y_map = x * sin_theta0 + y * cos_theta0 + init_pose[1];

    odom_msg.pose.pose.position.x = x_map;
    odom_msg.pose.pose.position.y = y_map;

    // Extract rotation (theta = yaw) 
    tf2::Quaternion q;
    tf2::fromMsg(odom_msg.pose.pose.orientation, q);
    double theta = tf2::getYaw(q); 

    // Add initial yaw offset
    theta += theta0; 

    // Normalize theta to [-π, π] to prevent angle drift
    const double PI = 3.14159265358979; // Pi constant
    theta = std::fmod(theta + PI, 2 * PI) - PI;

    // Move back to quaternion format
    q.setRPY(0, 0, theta); 
    tf2::convert(q, odom_msg.pose.pose.orientation);

    // Return odometry message in map frame
    return odom_msg;
}

std::vector<double> OdomPubSub::getInitPose() const
{
    // Return initial pose
    return init_pose_;
};

std::vector<double> OdomPubSub::getParams(ros::NodeHandle& nh) 
{
    // Initialize
    geometry_msgs::Pose init_pose;

    // Fetch position parameters
    if (!nh.getParam("/init_pose/position/x", init_pose.position.x)) {
        ROS_WARN("Parameter '/init_pose/position/x' not set. Defaulting to 0.0.");
        init_pose.position.x = 0.0;
    }
    if (!nh.getParam("/init_pose/position/y", init_pose.position.y)) {
        ROS_WARN("Parameter '/init_pose/position/y' not set. Defaulting to 0.0.");
        init_pose.position.y = 0.0;
    }
    if (!nh.getParam("/init_pose/position/z", init_pose.position.z)) {
        ROS_WARN("Parameter '/init_pose/position/z' not set. Defaulting to 0.0.");
        init_pose.position.z = 0.0;
    }

    // Fetch orientation parameters
    if (!nh.getParam("/init_pose/orientation/x", init_pose.orientation.x)) {
        ROS_WARN("Parameter '/init_pose/orientation/x' not set. Defaulting to 0.0.");
        init_pose.orientation.x = 0.0;
    }
    if (!nh.getParam("/init_pose/orientation/y", init_pose.orientation.y)) {
        ROS_WARN("Parameter '/init_pose/orientation/y' not set. Defaulting to 0.0.");
        init_pose.orientation.y = 0.0;
    }
    if (!nh.getParam("/init_pose/orientation/z", init_pose.orientation.z)) {
        ROS_WARN("Parameter '/init_pose/orientation/z' not set. Defaulting to 0.0.");
        init_pose.orientation.z = 0.0;
    }
    if (!nh.getParam("/init_pose/orientation/w", init_pose.orientation.w)) {
        ROS_WARN("Parameter '/init_pose/orientation/w' not set. Defaulting to 1.0.");
        init_pose.orientation.w = 1.0;
    }

    // Convert from quaternion to RPY
    tf2::Quaternion q(
        init_pose.orientation.x,
        init_pose.orientation.y,
        init_pose.orientation.z,
        init_pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Return x, y, and yaw
    return {init_pose.position.x, init_pose.position.y, yaw};
};