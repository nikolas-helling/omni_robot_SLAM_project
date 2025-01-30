#ifndef KINEMATICS_PUB_SUB_H
#define KINEMATICS_PUB_SUB_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <deque>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>

class KinematicsPubSub
{
public:
    // Constructor
    KinematicsPubSub(const ros::NodeHandle& nh, 
                     const std::string& pub_topic, 
                     const std::string& sub_topic, 
                     int pub_buffer_size, 
                     int sub_buffer_size);

    // Public Methods
    void subCallback(const sensor_msgs::JointState::ConstPtr& wheel_info);

private:
    // Private Members
    ros::NodeHandle nh_;                   
    ros::Publisher pub_;                  
    ros::Subscriber sub_;           

    std::deque<sensor_msgs::JointState::ConstPtr> buffer_;

    // Private Methods
    std::vector<double> velConversion(const std::vector<double>& tick_vel) const;
};

#endif // KINEMATICS_PUB_SUB_H