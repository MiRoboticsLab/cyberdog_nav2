#include "velocity_adaptor/velocity_adaptor.hpp"

namespace cyberdog
{
namespace navigation
{

VelocityAdaptor::VelocityAdaptor()
  : Node("velocity_adaptor")
{
  motion_vel_cmd_pub_ = this->create_publisher<::protocol::msg::MotionServoCmd>(
    "motion_servo_cmd", rclcpp::SystemDefaultsQoS());
  
  nav_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS(),
      std::bind(&VelocityAdaptor::HandleNavCommandVelocity, this, std::placeholders::_1));
}

VelocityAdaptor::~VelocityAdaptor()
{
}

void VelocityAdaptor::HandleNavCommandVelocity(geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (msg == nullptr) {
    INFO("VelocityAdaptor cmd vel == nullptr.");
    return;
  }
  PublishCommandVelocity(msg);
}

void VelocityAdaptor::PublishCommandVelocity(geometry_msgs::msg::Twist::SharedPtr msg)
{
  // INFO("SetCommandVelocity");
  std::vector<float> vel_des {
        static_cast<float>(msg->linear.x),
        static_cast<float>(msg->linear.y),
        static_cast<float>(msg->angular.z)
      };

    std::vector<float> step_height {
        0.05,
        0.05
      };

  ::protocol::msg::MotionServoCmd command;
  command.motion_id = 303;
  command.vel_des = vel_des;
  command.step_height = step_height;
  motion_vel_cmd_pub_->publish(command);
}


}  // namespace navigation
}  // namespace cyberdog