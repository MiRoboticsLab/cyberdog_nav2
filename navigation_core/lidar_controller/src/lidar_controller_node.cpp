#include "lidar_controller/lidar_controller.hpp"

namespace CYBERDOG_NAV {
LidarController::LidarController()
    : nav2_util::LifecycleNode("lidar_controller") {
  declare_parameter("topic_name", "lidar_switch");
}
LidarController::~LidarController() {}
nav2_util::CallbackReturn LidarController::on_configure(
    const rclcpp_lifecycle::State& /*state*/) {
  std::string topic_name = get_parameter("topic_name").as_string();
  callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  switch_cli_ = create_client<TRIGGERT>(
      topic_name, rmw_qos_profile_services_default, callback_group_);
  return nav2_util::CallbackReturn::SUCCESS;
}
/**
 * @brief Enable lidar by publish topic
 * @param state Lifecycle Node's state
 * @return Success or Failure
 */
nav2_util::CallbackReturn LidarController::on_activate(
    const rclcpp_lifecycle::State& /*state*/) {
  std_srvs::srv::SetBool_Request switcher;
  switcher.data = true;
  // cmd_pub_->on_activate();
  // cmd_pub_->publish(switcher);

  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}
/**
 * @brief Disable lidar by publish topic
 * @param state Lifecycle Node's state
 * @return Success or Failure
 */
nav2_util::CallbackReturn LidarController::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/) {
  std_srvs::srv::SetBool_Request switcher;
  switcher.data = false;
  // cmd_pub_->publish(switcher);
  // cmd_pub_->on_deactivate();
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}
/**
 * @brief Resets the member variables
 * @param state Lifecycle Node's state
 * @return Success or Failure
 */
nav2_util::CallbackReturn LidarController::on_cleanup(
    const rclcpp_lifecycle::State& /*state*/) {
  // cmd_pub_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}
/**
 * @brief Called when in Shutdown state
 * @param state Lifecycle Node's state
 * @return Success or Failure
 */
nav2_util::CallbackReturn LidarController::on_shutdown(
    const rclcpp_lifecycle::State& /*state*/) {
  return nav2_util::CallbackReturn::SUCCESS;
}
}  // namespace CYBERDOG_NAV
