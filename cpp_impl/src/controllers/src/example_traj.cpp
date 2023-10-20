#include "example_traj.hpp"

constexpr float DEG2RAD = M_PI / 180.0;

ExampleTraj::ExampleTraj() :
  rclcpp::Node("example_traj"),
  HOME{DEG2RAD * 45, DEG2RAD * -50, DEG2RAD * 110, DEG2RAD * 30}
{
    using namespace std::chrono_literals;

    this->_beginning = this->now();
    
    this->_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_cmds", 10);
    this->_timer = this->create_wall_timer(
      40ms, std::bind(&ExampleTraj::_timer_callback, this));
}

void ExampleTraj::_timer_callback()
{
  auto now = this->now();
  auto msg = trajectory_msgs::msg::JointTrajectory();
  msg.header.stamp = now;
  
  double dt = (now - this->_beginning).seconds();
  auto point = trajectory_msgs::msg::JointTrajectoryPoint();
  point.positions = {HOME[0] + 0.1 * M_PI * sin(2 * M_PI / 10.0 * dt),
                    HOME[1] + 0.1 * M_PI * sin(2 * M_PI / 10.0 * dt),
                    HOME[2] + 0.25 * M_PI * (sin(2 * M_PI / 10.0 * dt) - 1),
                    HOME[3] + 0.25 * M_PI * sin(2 * M_PI / 10.0 * dt) 
                  };

  msg.points = {point};

  this->_publisher->publish(msg);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<ExampleTraj>());
  rclcpp::shutdown();
  return 0;
}
