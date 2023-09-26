// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#include <mutex>

#include "std_msgs/msg/bool.hpp"
#include "algorithm_manager/executor_base.hpp"
#include "visualization/srv/stop.hpp"
#include "nav2_util/service_client.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "algorithm_manager/timer.hpp"
#include "cyberdog_visions_interfaces/srv/miloc_map_handler.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "protocol/srv/set_map_label.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorLaserMapping : public ExecutorBase
{
public:
  using MotionServiceCommand = protocol::srv::MotionResultCmd;
  using MilocMapHandler = cyberdog_visions_interfaces::srv::MilocMapHandler;
  using LabelParam = protocol::srv::SetMapLabel;

  explicit ExecutorLaserMapping(std::string node_name);
  ~ExecutorLaserMapping();

  void Start(AlgorithmMGR::Goal::ConstSharedPtr goal) override;
  void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) override;
  void Cancel() override;

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
  // bool EnableReportRealtimePose(bool enable, bool use_topic = false);

  /**
   * @brief Check localization lifecycle node not activate state
   *
   * @return true Return success
   * @return false Return failure
   */
  bool CheckAvailable();

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
  bool ResetAllLifecyceNodes();

  /**
   * @brief Record outdoor flag
   *
   * @return true Return success
   * @return false Return failure
   */
  bool InvokeOutdoorFlag(const std::string & mapname);

  bool CheckExit();

  bool CloseMappingService();

  // bool CanTransform(const std::string & parent_link, const std::string & clild_link);

  void ResetFlags();

  // feedback data
  ExecutorData executor_laser_mapping_data_;

  // service client
  // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr realtime_pose_client_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<MilocMapHandler>> miloc_client_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> start_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<visualization::srv::Stop>> stop_ {nullptr};
  rclcpp::Client<LabelParam>::SharedPtr outdoor_client_ {nullptr};

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

  bool is_exit_ {false};
  bool is_slam_service_activate_ {false};
  bool is_realtime_pose_service_activate_ {false};

  // mutex
  std::mutex lifecycle_mutex_;
  std::mutex service_mutex_;
  std::mutex realtime_pose_mutex_;

  // tf
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  // std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
};  // class ExecutorLaserMapping
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_LASER_MAPPING_HPP_
