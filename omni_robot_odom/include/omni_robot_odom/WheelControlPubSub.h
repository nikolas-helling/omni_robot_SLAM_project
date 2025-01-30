#ifndef WHEEL_CONTROL_PUB_SUB_H
#define WHEEL_CONTROL_PUB_SUB_H

#include <ros/ros.h>
#include <vector>
#include <string>

#include <geometry_msgs/TwistStamped.h>

#include <omni_robot_odom/WheelSpeeds.h>

class WheelControlPubSub
{
public:
    // Constructor
    WheelControlPubSub(const ros::NodeHandle& nh, 
                     const std::string& pub_topic, 
                     const std::string& sub_topic, 
                     int pub_buffer_size, 
                     int sub_buffer_size);

    // Public Methods
    void subCallback(const geometry_msgs::TwistStamped::ConstPtr& twist);

private:
    // Private Members
    ros::NodeHandle nh_;                   
    ros::Publisher pub_;                  
    ros::Subscriber sub_;           

    // Private Methods
    std::vector<double> velocityToRpm(const std::vector<double>& vel) const;
};

#endif // WHEEL_CONTROL_PUB_SUB_H
