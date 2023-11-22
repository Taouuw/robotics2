#include "rclcpp/rclcpp.hpp"

class GripperTest: public rclcpp::Node
{
private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;

    rclcpp::Time _beginning;
    void _timer_callback();

    const double HOME[4];

public:
    ExampleTraj();
};