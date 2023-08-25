#include "robot_node.hpp"
#include <cstdio>


RobotNode::RobotNode() 
  : Node("robot_node")
{
  using namespace std::chrono_literals;

  this->declare_parameter("l", std::vector<double>{1.0, 1.0, 1.0, 1.0});
  this->declare_parameter("f", 24.0);
  this->declare_parameter("pub_topic", "joint_states");
  this->declare_parameter("sub_topic", "joint_cmds");

  std::vector<double> l_doub = this->get_parameter("l").as_double_array();
  std::vector<float> l(l_doub.begin(), l_doub.end());
  this->bot = new Robot(l);

  this->joint_cmd_sub = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    this->get_parameter("sub_topic").as_string(),
    10,
    std::bind(&RobotNode::cmd_callback,
              this,
              std::placeholders::_1)
  );

  this->joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(
      this->get_parameter("pub_topic").as_string(), 10);

  this->_timer = this->create_wall_timer(1.0 / this->get_parameter("f").as_double() * 1s,
                                          std::bind(&RobotNode::timer_callback,
                                                    this));
}

RobotNode::~RobotNode()
{
  delete this->bot;
}

void RobotNode::cmd_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{

}
    
void RobotNode::timer_callback()
{
  sensor_msgs::msg::JointState js;
  js.header.stamp = this->now();
  std::vector<float> q_float = this->bot->get_q();
  std::vector<double> q(q_float.begin(), q_float.end());
  js.position = q;

  this->joint_state_pub->publish(js);

}    


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<RobotNode>());
  rclcpp::shutdown();
  return 0;
}
