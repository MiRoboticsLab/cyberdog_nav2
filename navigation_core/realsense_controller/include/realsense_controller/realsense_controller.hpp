#ifndef REALSENSE_CONTROLLER_HPP_
#define REALSENSE_CONTROLLER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
using TRIGGERT = std_srvs::srv::SetBool;
namespace CYBERDOG_NAV {
class RealsenseController : public nav2_util::LifecycleNode {
 public:
  RealsenseController();
  ~RealsenseController();

 protected:
  /**
   * @brief Sets up required params and services. Loads map and its parameters
   * from the file
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Enable lidar by publish topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Disable lidar by publish topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Resets the member variables
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& state) override;

 private:
  // A topic on which the control cmd will be published
  rclcpp::Client<TRIGGERT>::SharedPtr switch_cli_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};
}  // namespace CYBERDOG_NAV
#endif  // REALSENSE_CONTROLLER_HPP_