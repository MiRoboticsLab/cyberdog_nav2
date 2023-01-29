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

#ifndef ALGORITHM_MANAGER__EXECUTOR_AB_NAVIGATION_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_AB_NAVIGATION_HPP_

#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "algorithm_manager/executor_base.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "algorithm_manager/lifecycle_controller.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "cyberdog_visions_interfaces/srv/miloc_map_handler.hpp"
#include "algorithm_manager/timer.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorAbNavigation : public ExecutorBase
{
public:
  using MotionServiceCommand = protocol::srv::MotionResultCmd;
  using MapAvailableResult = cyberdog_visions_interfaces::srv::MilocMapHandler;

  explicit ExecutorAbNavigation(std::string node_name);
  ~ExecutorAbNavigation();

  /**
   * @brief Start Navigation AB
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
   * @brief Cancel Navigation AB, it't can debug ros command
   */
  void Cancel() override;

private:
  /**
   * @brief Handle `nav_action_client_` action client response callback function
   *
   * @param goal_handle
   */
  void HandleGoalResponseCallback(NavigationGoalHandle::SharedPtr goal_handle);

  /**
   * @brief Handle `nav_action_client_` action client feedback callback function
   *
   * @param feedback
   */
  void HandleFeedbackCallback(
    NavigationGoalHandle::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

  /**
   * @brief Handle `nav_action_client_` action client result callback function
   *
   * @param goal_handle
   */
  void HandleResultCallback(const NavigationGoalHandle::WrappedResult result);

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
   * @brief Check current given pose is available
   *
   * @param pose Target pose goal
   * @return true  Return success
   * @return false  Return failure
   */
  bool IsLegal(const AlgorithmMGR::Goal::ConstSharedPtr goal);

  /**
   * @brief Send APP or rviz set target pose goal
   *
   * @param pose Target pose goal
   * @return true Return success
   * @return false Return failure
   */
  bool SendGoal(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Function to check if current goal should be cancelled
   * @return bool True if current goal should be cancelled, false otherwise
   */
  bool ShouldCancelGoal();

  /**
   * @brief Check send async goal timeout
   *
   * @return true Success
   * @return false Failure
   */
  bool CheckTimeout();

  /**
   * @brief Function to check if the action server acknowledged a new goal
   * @param elapsed Duration since the last goal was sent and future goal handle has not completed.
   * After waiting for the future to complete, this value is incremented with the timeout value.
   * @return boolean True if future_goal_handle_ returns SUCCESS, False otherwise
   */
  bool IsFutureGoalHandleComplete(std::chrono::milliseconds & elapsed);

  /**
   * @brief Normalized app given pose
   *
   * @param pose
   */
  void NormalizedGoal(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Sources reset and cleanup
   *
   * @return true Success
   * @return false Failure
   */
  bool ReinitializeAndCleanup();

  /**
   * @brief Reinitialize all lifecycle nodes
   *
   * @return true Success
   * @return false Failure
   */
  bool LifecycleNodesReinitialize();

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
  void Debug2String(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief To String
   *
   * @param status
   */
  void NavigationStatus2String(int8_t status);

  /**
   * @brief Release source and reset
   *
   */
  void ReleaseSources();

  /**
   * @brief Check curent map file available
   *
   * @return true Return success
   * @return false Return failure
   */
  bool CheckMapAvailable(const std::string & map_name = "map.pgm");

  /**
   * Check current is outdoor mode
   */
  bool CheckUseOutdoor(bool & outdoor, const std::string & filename = "map.json");

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
  void ResetPreprocessingValue();
  void PublishZeroPath();
  bool ResetAllLifecyceNodes();

  bool StopRunningRobot();

  void HandleStopRobotNavCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> respose);

  bool CheckExit();

  bool CancelGoal();

  // feedback data
  ExecutorData executor_nav_ab_data_;

  // Navigation lifecycles
  std::unordered_map<std::string, std::shared_ptr<LifecycleController>>
  navigation_lifecycle_;

  // navigation target goal
  nav2_msgs::action::NavigateToPose::Goal target_goal_;

  // nav client as request
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_ {nullptr};

  // navigation goal handle
  NavigationGoalHandle::SharedPtr nav_goal_handle_ {nullptr};

  // Lifecycle controller
  std::unique_ptr<nav2_lifecycle_manager::LifecycleManagerClient> nav_client_ {nullptr}; \
  rclcpp::Time time_goal_sent_;

  // Control localization_node lifecycle
  // std::shared_ptr<LifecycleController> localization_lifecycle_ {nullptr};

  // velocity smoother 'velocity_adaptor_gait'
  std::shared_ptr<nav2_util::ServiceClient<MotionServiceCommand>> velocity_smoother_ {nullptr};

  // Control `map server` lifecycle node
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr vins_location_stop_client_ {nullptr};

  // Control `map server` lifecycle node
  std::shared_ptr<LifecycleController> map_server_lifecycle_ {nullptr};

  // all depend is ready
  bool lifecycle_depend_ready_ {false};

  // Stop lidar and vision location module
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_{nullptr};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_lidar_trigger_pub_{nullptr};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_vision_trigger_pub_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_nav_trigger_sub_{nullptr};

  // nav trigger
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_stop_trigger_sub_{nullptr};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stop_running_server_;
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  std::atomic_bool navigation_reset_trigger_{false};


  // Record lidar or vision flag
  bool use_vision_slam_ {false};
  bool use_lidar_slam_ {false};

  // check current navigation is exit trigger
  bool is_exit_ {false};

  // Preprocessing flag
  bool connect_server_finished_ {false};
  bool start_lifecycle_depend_finished_ {false};
  bool start_velocity_smoother_finished_ {false};

  // mutex
  std::mutex cancel_goal_mutex_;
  std::condition_variable cancel_goal_cv_;
  bool cancel_goal_result_{true};
  std::mutex lifecycle_mutex_;
  std::mutex action_mutex_;
};  // class ExecutorAbNavigation
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_AB_NAVIGATION_HPP_
