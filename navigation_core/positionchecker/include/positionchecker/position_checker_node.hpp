#ifndef _POSITION_CHECKEr_NODE_HPP_
#define _POSITION_CHECKEr_NODE_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

using SetBool = std_srvs::srv::SetBool;
namespace CYBERDOG_NAV {
class PositionChecker : public rclcpp::Node {
 public:
  PositionChecker();
  ~PositionChecker();

 private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  void loop();
  bool looping_;
  std::shared_ptr<std::thread> loop_thread_;
  rclcpp::Service<SetBool>::SharedPtr enable_service;
  void serviceCallback(const std::shared_ptr<rmw_request_id_t>,
                       const std::shared_ptr<SetBool::Request> request,
                       std::shared_ptr<SetBool::Response> response);
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_;
};
}  // namespace CYBERDOG_NAV

#endif  // _POSITION_CHECKEr_NODE_HPP_