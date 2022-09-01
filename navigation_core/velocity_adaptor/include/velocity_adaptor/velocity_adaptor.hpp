#ifndef VELOCITY_ADAPTOR_VELOCITY_ADAPTOR_HPP
#define VELOCITY_ADAPTOR_VELOCITY_ADAPTOR_HPP

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog {

namespace navigation {

class VelocityAdaptor : public ::rclcpp::Node
{ 
 public:
  explicit VelocityAdaptor();
  ~VelocityAdaptor();

private:
  /**
   * @brief Receive Nav2 publish cmd_vel topic data
   * 
   * @param msg 
   */
  void HandleNavCommandVelocity(geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief 
   * 
   * @param msg 
   */
  void PublishCommandVelocity(geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_vel_sub_ {nullptr};
  rclcpp::Publisher<::protocol::msg::MotionServoCmd>::SharedPtr motion_vel_cmd_pub_ {nullptr};
};

}  // namespace navigation
}  // namespace cyberdog

#endif  // VELOCITY_ADAPTOR_VELOCITY_ADAPTOR_HPP