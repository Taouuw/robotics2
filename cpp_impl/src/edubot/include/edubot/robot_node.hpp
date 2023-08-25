#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot.hpp"

class RobotNode : public rclcpp::Node
{
  public:
  
    RobotNode();
    ~RobotNode();

  private:
  
    Robot* bot;

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;

    rclcpp::TimerBase::SharedPtr _timer;

    void cmd_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void timer_callback();    
};