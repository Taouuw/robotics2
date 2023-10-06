#include "tf_publisher.hpp"
#include "common/common.hpp"

using namespace personal;

TF_Publisher::TF_Publisher():
    Node("tf_publisher")
{
    this->declare_parameter("sub_topic", "joint_states");
    this->declare_parameter("l", std::vector<double>({1.0, 1.0, 1.0, 1.0, 1.0}));
    this->declare_parameter("world2base", std::vector<double>({0.0, 0.0, 0.0, 1.0}));
    this->declare_parameter("alignments.joint_0", std::vector<double>({0.0, 0.0, 0.0}));
    this->declare_parameter("alignments.joint_1", std::vector<double>({0.0, -M_PI_2, 0.0}));
    this->declare_parameter("alignments.joint_2", std::vector<double>({0.0, 0.0, 0.0}));
    this->declare_parameter("alignments.joint_3", std::vector<double>({0.0, 0.0, 0.0}));
    this->declare_parameter("alignments.joint_4", std::vector<double>({0.0, 0.0, 0.0}));
    
    this->_l = this->get_parameter("l").as_double_array();

    for(uint i = 0; i < this->_l.size(); i++)
    {
        char param_name[20];
        sprintf(param_name, "alignments.joint_%d", i);
        std::vector<double> euler_angles = this->get_parameter(param_name).as_double_array();
        this->_alignments.push_back(            common::rot_z(euler_angles.at(2))
                                    * common::rot_y(euler_angles.at(1))
                                    * common::rot_x(euler_angles.at(0)));
    }

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

    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "world";
    t.header.stamp = msg->header.stamp;
    t.child_frame_id = "frame_0";

    Eigen::Vector3d translation = {0.0, 0.0, 0.0};
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity() * this->_alignments.at(0);
    t.transform.translation.x = translation.x();
    t.transform.translation.y = translation.y();
    t.transform.translation.z = translation.z();

    Eigen::Quaterniond q(rotation);
    t.transform.rotation.w = q.w();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    
    this->tf_broadcaster_->sendTransform(t);

    for(uint i = 1; i < this->_l.size(); i++)
    {   
        t.header.frame_id = t.child_frame_id;
        char child_frame_id[8];
        sprintf(child_frame_id, "frame_%d", i);
        t.child_frame_id = child_frame_id;

        translation = this->_l.at(i)
                        * Eigen::Vector3d::UnitZ();
        rotation =  this->_alignments.at(i);
        //(common::rot_x(msg->position[i-1]) 
        //                * this->_alignments.at(i));

        t.transform.translation.x = translation.x();
        t.transform.translation.y = translation.y();
        t.transform.translation.z = translation.z();

        q = Eigen::Quaterniond(rotation);
        t.transform.rotation.w = q.w();
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();

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