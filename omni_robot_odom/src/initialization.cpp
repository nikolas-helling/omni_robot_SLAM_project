#include <ros/ros.h>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class InitialPoseSetter {
public:
    // Constructor
    InitialPoseSetter(const std::string& topic)
        : nh_(), first_msg_received_(false) {
        // Subscribe to topics
        sub_ = nh_.subscribe(topic, 1, &InitialPoseSetter::callback, this);
        }
private:
    // Private Members
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    bool params_fetched_ = false;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    geometry_msgs::PoseStamped init_pose_;
    bool first_msg_received_;
    
    // Private Methods
    void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

        // Check for first message
        if (first_msg_received_) {
            return; // Ignore messages after first one
        }
        first_msg_received_ = true;

        // Set initial pose from first message
        init_pose_ = *msg;

        // Set parameters on the parameter server
        nh_.setParam("init_pose/position/x", init_pose_.pose.position.x);
        nh_.setParam("init_pose/position/y", init_pose_.pose.position.y);
        nh_.setParam("init_pose/position/z", init_pose_.pose.position.z);
        nh_.setParam("init_pose/orientation/x", init_pose_.pose.orientation.x);
        nh_.setParam("init_pose/orientation/y", init_pose_.pose.orientation.y);
        nh_.setParam("init_pose/orientation/z", init_pose_.pose.orientation.z);
        nh_.setParam("init_pose/orientation/w", init_pose_.pose.orientation.w);

        ROS_INFO("InitialPoseSetter: Initial pose parameters set on the parameter server (from data bag).");

        // Publish the static transform
        const std::string parent_frame_id = "map";
        const std::string child_frame_id = "odom";
        publishStaticTransform(parent_frame_id, child_frame_id);

        ROS_INFO("InitialPoseSetter: Static transform published (map -> odom).");
    }

    void publishStaticTransform(const std::string& parent_frame_id, const std::string& child_frame_id) {

        // Set static transform message
        geometry_msgs::TransformStamped transform_msg;

        transform_msg.header.stamp = init_pose_.header.stamp;
        transform_msg.header.frame_id = parent_frame_id;
        transform_msg.child_frame_id = child_frame_id;
        transform_msg.transform.translation.x = init_pose_.pose.position.x;
        transform_msg.transform.translation.y = init_pose_.pose.position.y;
        // transform_msg.transform.translation.z = init_pose_.pose.position.z;
        transform_msg.transform.translation.z = 0.0;
        transform_msg.transform.rotation = init_pose_.pose.orientation;

        // Broadcast static transform
        static_broadcaster_.sendTransform(transform_msg);
    }
};

int main(int argc, char** argv) {

    // Initialize ROS node and NodeHandle
    ros::init(argc, argv, "initialization");
    ros::NodeHandle nh();

    // Use InitialPoseSetter class with necessary parameters:
    // ""/robot/pose"" subscribe topic for reading the robot's ground truth first pose
    InitialPoseSetter initPose("/robot/pose");

    // Keep the node running
    ros::spin();

    return 0;
}
