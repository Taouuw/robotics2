#include "rclcpp/rclcpp.hpp"

#include <Eigen/Dense>

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"

class TF_Publisher : public rclcpp::Node
{
public:
    TF_Publisher();
    
private:

    std::vector<double> _l;
    std::vector<Eigen::Matrix3d> _alignments;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

