#include "tf_publisher.hpp"

TF_Publisher::TF_Publisher():
    Node("tf_publisher")
{
    this->declare_parameter("sub_topic", "joint_states");
    this->declare_parameter("l", std::vector<double>({1.0, 1.0, 1.0, 1.0}));
    this->declare_parameter("world2base", std::vector<double>({0.0, 0.0, 0.0, 1.0}));

    this->l = this->get_parameter("l").as_double_array();

    this->joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        this->get_parameter("sub_topic").as_string(),
        10,
        std::bind(&TF_Publisher::joint_callback,
                this,
                std::placeholders::_1)
    );

    this->tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}


void TF_Publisher::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{

    for(uint i = 0; i < this->l.size(); i++)
    {
        geometry_msgs::msg::TransformStamped t;
        //t.header.frame_id = msg->name[i];
        t.header.stamp = msg->header.stamp;

        // TODO compute current transforms from JointState msg

        this->tf_broadcaster_->sendTransform(t);
    }
    
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<TF_Publisher>());
  rclcpp::shutdown();
  return 0;
}