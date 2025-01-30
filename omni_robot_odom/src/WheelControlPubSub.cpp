#include <ros/ros.h>
#include <vector>
#include <string>

#include <geometry_msgs/TwistStamped.h>

#include <omni_robot_odom/WheelControlPubSub.h>
#include <omni_robot_odom/WheelSpeeds.h>

WheelControlPubSub::WheelControlPubSub(const ros::NodeHandle& nh, 
                                   const std::string& pub_topic, 
                                   const std::string& sub_topic, 
                                   int pub_buffer_size, 
                                   int sub_buffer_size)
    : nh_(nh)
{
    // Advertise publishers
    pub_ = nh_.advertise<omni_robot_odom::WheelSpeeds>(pub_topic, pub_buffer_size);
    
    // Subscribe to topics
    sub_ = nh_.subscribe(sub_topic, sub_buffer_size, &WheelControlPubSub::subCallback, this);

    // Log initialization
    ROS_INFO("WheelControlPubSub initialized with publish topics: '%s'", pub_topic.c_str());
    ROS_INFO("WheelControlPubSub initialized with subscribe topics: '%s'", sub_topic.c_str());
}

void WheelControlPubSub::subCallback(const geometry_msgs::TwistStamped::ConstPtr& twist)
{
    // Initialize
    std::vector<double> vel = {twist->twist.linear.x, twist->twist.linear.y, twist->twist.angular.z};

    // Transform body frame velocities to wheel speeds in rpm 
    std::vector<double> rpm = velocityToRpm(vel);

    // Publish wheel speeds in rpm on /wheel_rpm topic
    omni_robot_odom::WheelSpeeds wheels_rpm;
    wheels_rpm.rpm_fl = rpm[0];
    wheels_rpm.rpm_fr = rpm[1];
    wheels_rpm.rpm_rl = rpm[2];
    wheels_rpm.rpm_rr = rpm[3];

    wheels_rpm.header.stamp = twist->header.stamp;
    // wheels_rpm.child_frame_id = "base_link";
    // wheels_rpm.header.frame_id = "map";
    pub_.publish(wheels_rpm);
}

std::vector<double> WheelControlPubSub::velocityToRpm(const std::vector<double>& vel) const
{
    // Initialize
    double rpm_fl, rpm_fr, rpm_rl, rpm_rr;
    double vx = vel[0];
    double vy = vel[1];
    double wz = vel[2];

    // Set robot parameters
    const double L = 0.2;   // [m] Distance between the front and back wheels (robot length)
    const double W = 0.169; // [m] Distance between left and right wheels (robot width)
    const double R = 0.07;  // [m] Wheel radius
    const double RATIO = 5; // Ratio between motor encoder ticks and wheel rotation (scaling factor)
    const double TICK_PER_ROT = 42; // Number of encoder ticks per full wheel rotation
    const double PI = 3.14159265358979; // Pi constant

    // Compute wheel speed in rpm
    rpm_fl = (vx - vy - (W + L) * wz) / R;
    rpm_fr = (vx + vy + (W + L) * wz) / R;
    rpm_rl = (vx + vy - (W + L) * wz) / R;
    rpm_rr = (vx - vy + (W + L) * wz) / R;

    std::vector<double> vec = {rpm_fl, rpm_fr, rpm_rl, rpm_rr};

    // Convert to rad/min
    for (int i = 0; i < 4; i++)
    {
        vec[i] = vec[i] * RATIO * 60;
    }

    // Return wheel speeds
    return vec;
}
