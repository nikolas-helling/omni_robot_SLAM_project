#include <ros/ros.h>
#include <vector>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>

#include <omni_robot_odom/KinematicsPubSub.h>

KinematicsPubSub::KinematicsPubSub(const ros::NodeHandle& nh, 
                                   const std::string& pub_topic, 
                                   const std::string& sub_topic, 
                                   int pub_buffer_size, 
                                   int sub_buffer_size)
    : nh_(nh)
{
    // Advertise publishers
    pub_ = nh_.advertise<geometry_msgs::TwistStamped>(pub_topic, pub_buffer_size);
    
    // Subscribe to topics
    sub_ = nh_.subscribe(sub_topic, sub_buffer_size, &KinematicsPubSub::subCallback, this);

    // Log initialization
    ROS_INFO("KinematicsPubSub initialized with publish topics: '%s'", pub_topic.c_str());
    ROS_INFO("KinematicsPubSub initialized with subscribe topics: '%s'", sub_topic.c_str());
}

void KinematicsPubSub::subCallback(const sensor_msgs::JointState::ConstPtr& wheel_info)
{
    // Initialize
    std::vector<double> tick_vel(4, 0.0);
    std::vector<double> d_ticks(4, 0.0);
    geometry_msgs::TwistStamped curr_twist;

    // Use deque as a data buffer
    const int MIN_BUFFER_SIZE = 4;
    const int MAX_BUFFER_SIZE = 7;
    buffer_.push_front(wheel_info);

    if (buffer_.size() > MIN_BUFFER_SIZE)
    {
        // Compute time difference
        const double t1 = buffer_[4]->header.stamp.toSec();
        const double t0 = buffer_[0]->header.stamp.toSec();
        const double dt = t0 - t1;

        // Compute tick difference and velocity for each wheel
        for (int i = 0; i < 4; i++)
        {
            d_ticks[i] = buffer_[0]->position[i] - buffer_[4]->position[i];
            tick_vel[i] = d_ticks[i] / dt;
        }

        // Compute body frame twist
        const std::vector<double> twist = velConversion(tick_vel);

        curr_twist.twist.linear.x = twist[0];
        curr_twist.twist.linear.y = twist[1];
        curr_twist.twist.linear.z = twist[2];
        curr_twist.twist.angular.x = twist[3];
        curr_twist.twist.angular.y = twist[4];
        curr_twist.twist.angular.z = twist[5];

        // Publish twist on the velocity topic
        curr_twist.header.stamp = buffer_[0]->header.stamp;
        // curr_twist.child_frame_id = "base_link";  
        // curr_twist.header.frame_id = "map";    
        pub_.publish(curr_twist);

        // Maintain buffer size
        if (buffer_.size() > MAX_BUFFER_SIZE)
        {
            buffer_.pop_back();
        }
    }
}

std::vector<double> KinematicsPubSub::velConversion(const std::vector<double>& tick_vel) const
{   
    // Initialize
    std::vector<double> scaled_tick_vel = tick_vel;
    double vx, vy, vz, wx, wy, wz;

    // Set robot parameters
    const double L = 0.2;   // [m] Distance between the front and back wheels (robot length)
    const double W = 0.169; // [m] Distance between left and right wheels (robot width)
    const double R = 0.07;  // [m] Wheel radius
    const double RATIO = 5; // Ratio between motor encoder ticks and wheel rotation (scaling factor)
    const double TICK_PER_ROT = 42; // Number of encoder ticks per full wheel rotation
    const double PI = 3.14159265358979; // Pi constant

    // Transform encoder tick velocities
    for (int i = 0; i < 4; i++)
    {
        scaled_tick_vel[i] = ((scaled_tick_vel[i]) / RATIO / TICK_PER_ROT) * (2 * PI);
    }

    // Compute twist in body frame from encoder ticks
    vx = ((scaled_tick_vel[0] + scaled_tick_vel[1] + scaled_tick_vel[2] + scaled_tick_vel[3]) / 4) * R;
    vy = ((-scaled_tick_vel[0] + scaled_tick_vel[1] + scaled_tick_vel[2] - scaled_tick_vel[3]) / 4) * R;
    vz = 0.0;
    wx = 0.0;
    wy = 0.0;
    wz = (R / (4 * (W + L))) * (-scaled_tick_vel[0] + scaled_tick_vel[1] - scaled_tick_vel[2] + scaled_tick_vel[3]);
    
    // Return twist
    std::vector<double> vec = {vx, vy, vz, wx, wy, wz};

    return vec;
}
