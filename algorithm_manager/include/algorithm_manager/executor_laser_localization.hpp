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

#ifndef ALGORITHM_MANAGER__EXECUTOR_LASER_LOCALIZATION_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_LASER_LOCALIZATION_HPP_

#include <string>
#include <memory>
#include <mutex>
#include <condition_variable>

#include "algorithm_manager/executor_base.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "algorithm_manager/timer.hpp"
#include "algorithm_manager/global_pose_publisher.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorLaserLocalization : public ExecutorBase
{
public:
  enum class LocationStatus
  {
    Unknown,
    SUCCESS,
    FAILURE
  };

  /**
   * @brief Construct a new Executor Laser Localization object
   *
   * @param node_name Executor node name
   */
  explicit ExecutorLaserLocalization(std::string node_name);
  ~ExecutorLaserLocalization();

  /**
  * @brief Start lidar localization
  *
  * @param goal APP or rviz set target goal
  */
  void Start(const AlgorithmMGR::Goal::ConstSharedPtr goal) override;

  /**
   * @brief Handle APP set stop lidar localization
   *
   * @param request The GRPC service request command
   * @param response The GRPC service response result
   */
  void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) override;

  /**
   * @brief Cancel lidar localization, it't can debug ros command
   */
  void Cancel() override;

private:
  /**
   * Handle location request status
   */
  void HandleLocationServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  /**
   * @brief Handle lidar relocalization result
   *
   * @param msg The lidar relocalization result
   */
  void HandleRelocalizationCallback(const std_msgs::msg::Int32::SharedPtr msg);

  /**
  * @brief Check `camera/camera` real sense sensor status
  *
  * @return true Return success
  * @return false Return failure
  */
  bool IsDependsReady();

  /**
   * @brief Wait relocalization result is success
   *
   * @param timeout Wait time
   * @return true Return success
   * @return false Return failure
   */
  bool WaitRelocalization(std::chrono::seconds timeout, bool & force_quit);

  /**
   * @brief Enable Lidar Relocalization turn on
   *
   * @return true Return success
   * @return false Return failure
   */
  bool EnableRelocalization();

  /**
   * @brief Enable Lidar Relocalization turn off
   *
   * @return true Return success
   * @return false Return failure
   */
  bool DisableRelocalization();

  void ResetFlags();

  bool ResetAllLifecyceNodes();

  bool SendServerRequest(
    const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
    const std_srvs::srv::SetBool::Request::SharedPtr & request,
    std_srvs::srv::SetBool::Response::SharedPtr & response);

  void HandleStopCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> respose);

  bool StopLocalizationFunctions();

  bool CheckExit();

  // feedback data
  ExecutorData executor_laser_mapping_data_;

  // std::unique_ptr<nav2_lifecycle_manager::LifecycleManagerClient> localization_client_ {nullptr};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr location_status_service_ {nullptr};

  // service client
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> start_client_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> stop_client_ {nullptr};
  PosePublisher::SharedPtr pose_publisher_ {nullptr};

  // serice reset(stop current robot running)
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stop_robot_nav_client_ {nullptr};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stop_running_server_;
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};

  // Subscription lidar localization topic result
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr relocalization_sub_{nullptr};

  // Record relocalization result
  bool relocalization_success_ {false};
  bool relocalization_failure_ {false};

  //  LocationStatus location_status_;
  LocationStatus location_status_;

  // in service
  bool is_activate_ {false};
  bool is_exit_ {false};
  bool is_lifecycle_activate_ {false};
  bool is_slam_service_activate_ {false};

  // mutex
  std::mutex lifecycle_mutex_;
  std::mutex service_mutex_;
  std::mutex realtime_pose_mutex_;
};  // class ExecutorLaserLocalization
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_LASER_LOCALIZATION_HPP_
