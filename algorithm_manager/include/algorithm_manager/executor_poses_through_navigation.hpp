// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ALGORITHM_MANAGER__EXECUTOR_POSES_THROUGH_NAVIGATION_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_POSES_THROUGH_NAVIGATION_HPP_

#include <string>
#include <memory>
#include <vector>

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "algorithm_manager/executor_base.hpp"
#include "algorithm_manager/lifecycle_node_manager.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cyberdog_visions_interfaces/srv/miloc_map_handler.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorPosesThroughNavigation : public ExecutorBase
{
public:
  using MotionServiceCommand = protocol::srv::MotionResultCmd;
  using NavThroughPosesGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;

  explicit ExecutorPosesThroughNavigation(std::string node_name);
  ~ExecutorPosesThroughNavigation();

  /**
   * @brief Start Poses Though AB
   *
   * @param goal APP or rviz set target pose goal
   */
  void Start(const AlgorithmMGR::Goal::ConstSharedPtr goal) override;

  /**
   * @brief Handle APP set stop Navigation AB
   *
   * @param request The
   * @param response The
   */
  void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) override;

  /**
   * @brief Cancel Navigation Poses Though, it't can debug ros command
   */
  void Cancel() override;

private:
  /**
  * @brief Handle `nav_action_client_` action client response callback function
  *
  * @param goal_handle
  */
  void HandleGoalResponseCallback(NavThroughPosesGoalHandle::SharedPtr goal_handle);

  /**
   * @brief Handle `nav_action_client_` action client feedback callback function
   *
   * @param feedback
   */
  void HandleFeedbackCallback(
    NavThroughPosesGoalHandle::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback);

  /**
   * @brief Handle `nav_action_client_` action client result callback function
   *
   * @param goal_handle
   */
  void HandleResultCallback(const NavThroughPosesGoalHandle::WrappedResult result);

  /**
   * @brief Handle executor_reset_nav command
   *
   * @param msg The executor request
   */
  void HandleTriggerStopCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Check `lifecycle_manager_navigation` and `lifecycle_manager_localization`
   * `real sense` sensor lidar status
   *
   * @return true Return success
   * @return false Return failure
   */
  bool IsDependsReady();

  /**
   * @brief Check action connect server
   *
   * @return true Return success
   * @return false Return failure
   */
  bool IsConnectServer();

  /**
   * @brief Call poses though navigation
   *
   * @param poses Multi poses
   */
  bool StartNavThroughPoses(const std::vector<geometry_msgs::msg::PoseStamped> & poses);

  /**
   * @brief When robot mapping it's should walk smoother
   *
   * @return true Return success
   * @return false Return failure
   */
  bool VelocitySmoother();

  /**
   * @brief Print target goal pose
   *
   * @param pose APP or rviz set target pose goal
   */
  void Debug2String(const std::vector<geometry_msgs::msg::PoseStamped> & poses);

  /**
   * @brief Release source and reset
   *
   */
  void ReleaseSources();

  /**
   * @brief Check vision slam location
   *
   * @return true Return success
   * @return false Return failure
   */
  bool IsUseVisionLocation();

  /**
   * @brief Check lidar slam location
   *
   * @return true Return success
   * @return false Return failure
   */
  bool IsUseLidarLocation();

  /**
   * @brief Set the Location Type object
   *
   * @param outdoor true : vision
   *                false: lidar
   */
  void SetLocationType(bool outdoor);

  /**
   * @brief Set `use_vision_slam_` and `use_lidar_slam_` default value
   */
  void ResetDefaultValue();

  // navigation through poses
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
    nav_through_poses_action_client_;

  nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;
  NavThroughPosesGoalHandle::SharedPtr nav_through_poses_goal_handle_;

  // Lifecycle controller
  std::unique_ptr<nav2_lifecycle_manager::LifecycleManagerClient> nav_client_ {nullptr};

  // Control `map server` lifecycle node
  std::shared_ptr<LifecycleController> map_server_lifecycle_ {nullptr};

  // velocity smoother 'velocity_adaptor_gait'
  std::shared_ptr<nav2_util::ServiceClient<MotionServiceCommand>> velocity_smoother_ {nullptr};

  // Stop lidar and vision location module
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_lidar_trigger_pub_{nullptr};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_vision_trigger_pub_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_nav_trigger_sub_{nullptr};

  // nav trigger
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_stop_trigger_sub_{nullptr};
  std::atomic_bool navigation_reset_trigger_{false};

  // Record lidar or vision flag
  bool use_vision_slam_ {false};
  bool use_lidar_slam_ {false};
};

}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_POSES_THROUGH_NAVIGATION_HPP_
