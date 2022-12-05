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

#ifndef ALGORITHM_MANAGER__EXECUTOR_LASER_MAPPING_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_LASER_MAPPING_HPP_

#include <string>
#include <memory>

#include "std_msgs/msg/bool.hpp"
#include "algorithm_manager/executor_base.hpp"
#include "algorithm_manager/lifecycle_node_manager.hpp"
#include "visualization/srv/stop.hpp"
#include "nav2_util/service_client.hpp"
#include "protocol/srv/motion_result_cmd.hpp"

namespace cyberdog
{
namespace algorithm
{
class ExecutorLaserMapping : public ExecutorBase
{
public:
  using LifeCycleNodeType = LifecycleNodeManager::LifeCycleNode;
  using MotionServiceCommand = protocol::srv::MotionResultCmd;

  explicit ExecutorLaserMapping(std::string node_name);
  ~ExecutorLaserMapping();

  void Start(AlgorithmMGR::Goal::ConstSharedPtr goal) override;
  void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) override;
  void Cancel() override;
  // void UpdateStatus(const ExecutorStatus & executor_status) override;
  // void GetFeedback(protocol::action::Navigation::Feedback::SharedPtr feedback) override;

private:
  /**
   * @brief declare_parameter for yaml file
   */
  void DeclareParameters();

  /**
   * @brief Get the Parameters object
   */
  void GetParameters();

  /**
   * @brief Check `camera/camera` real sense sensor status
   *
   * @return true Return success
   * @return false Return failure
   */
  bool IsDependsReady();

  /**
   * @brief Lidar start build mapping
   *
   * @return true Return success
   * @return false Return failure
   */
  bool StartBuildMapping();

  /**
   * @brief Lidar stop build mapping
   *
   * @param map_filename Set lidar save map filename
   * @return true Return success
   * @return false Return failure
   */
  bool StopBuildMapping(const std::string & map_filename);

  /**
   * @brief Turn on ot turn off report realtime robot pose
   *
   * @param enable True enable report, false disenable report
   * @return true Return success
   * @return false Return failure
   */
  bool EnableReportRealtimePose(bool enable, bool use_topic = false);

  /**
   * @brief Check localization lifecycle node not activate state
   *
   * @return true Return success
   * @return false Return failure
   */
  bool CheckAvailable();

  /**
   * @brief Enable Lidar Localization turn off
   *
   * @return true Return success
   * @return false Return failure
   */
  bool DisenableLocalization();

  /**
   * @brief When robot mapping it's should walk smoother
   *
   * @return true Return success
   * @return false Return failure
   */
  bool VelocitySmoother();

  /**
   * @brief Radar is mapping
   */
  void PublishBuildMapType();

  /**
   * @brief Set all lifecycle default state
   *
   * @return true Return success
   * @return false Return failure
   */
  bool ResetLifecycleDefaultValue();

  // feedback data
  ExecutorData executor_laser_mapping_data_;

  // // Control lidar mapping lifecycle(Nav2 lifecycle)
  // nav2_lifecycle_manager::LifecycleManagerClient client_mapping_{nullptr};

  // Lifecycle controller
  // std::unique_ptr<nav2_lifecycle_manager::LifecycleManagerClient> mapping_client_ {nullptr};
  // std::unique_ptr<nav2_lifecycle_manager::LifecycleManagerClient> localization_client_ {nullptr};

  // service client
  // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr start_client_ {nullptr};
  // rclcpp::Client<visualization::srv::Stop>::SharedPtr stop_client_ {nullptr};
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr realtime_pose_client_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> start_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<visualization::srv::Stop>> stop_ {nullptr};

  // velocity smoother 'velocity_adaptor_gait'
  std::shared_ptr<nav2_util::ServiceClient<MotionServiceCommand>> velocity_smoother_ {nullptr};

  // Control realsense camera lifecycle
  std::shared_ptr<LifecycleController> localization_client_ {nullptr};
  std::shared_ptr<LifecycleController> mapping_client_ {nullptr};
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stop_client_ {nullptr};

  // lidar mapping alive
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_mapping_trigger_pub_{nullptr};

  // realtime robot pose
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_pose_pub_{nullptr};
  bool start_report_realtime_pose_ {false};

  // timeout parameters
  int localization_service_timeout_;
  int mapping_start_service_timeout_;
  int mapping_stop_service_timeout_;
  int pose_report_service_timeout_;
  int velocity_smoother_service_timeout_;

  // record this sensor is open
  bool is_open_realsense_camera_ {false};
};  // class ExecutorLaserMapping
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_LASER_MAPPING_HPP_
