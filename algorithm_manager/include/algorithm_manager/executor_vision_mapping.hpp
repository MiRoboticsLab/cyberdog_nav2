//
// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef ALGORITHM_MANAGER__EXECUTOR_VISION_MAPPING_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_VISION_MAPPING_HPP_

#include <string>
#include <memory>

#include "std_msgs/msg/bool.hpp"
#include "algorithm_manager/executor_base.hpp"
#include "visualization/srv/stop.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "cyberdog_visions_interfaces/srv/miloc_map_handler.hpp"
#include "algorithm_manager/timer.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "protocol/srv/set_map_label.hpp"
#include "algorithm_manager/global_pose_publisher.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorVisionMapping : public ExecutorBase
{
public:
  using MotionServiceCommand = protocol::srv::MotionResultCmd;
  using MapAvailableResult = cyberdog_visions_interfaces::srv::MilocMapHandler;
  using LabelParam = protocol::srv::SetMapLabel;

  explicit ExecutorVisionMapping(std::string node_name);
  void Start(AlgorithmMGR::Goal::ConstSharedPtr goal) override;
  void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) override;
  void Cancel() override;

private:
  /**
   * @brief Check `camera/camera` real sense sensor status
   * and RGB-D sensor
   *
   * @return true Return success
   * @return false Return failure
   */
  bool IsDependsReady();

  /**
   * @brief Vision start build mapping
   *
   * @return true Return success
   * @return false Return failure
   */
  bool StartBuildMapping();

  /**
   * @brief Vision stop build mapping
   *
   * @param map_filename Set Vision save map filename
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
  bool EnableReportRealtimePose(bool enable);

  /**
   * @brief Check curent map building available
   *
   * @return true Return success
   * @return false Return failure
   */
  bool CheckBuildMappingAvailable();

  /**
   * @brief Delete maps
   * @return true Return success
   * @return false Return failure
   */
  bool DeleteMap();

  /**
   * @brief Record outdoor flag
   *
   * @return true Return success
   * @return false Return failure
   */
  bool InvokeOutdoorFlag(const std::string & mapname);

  /**
  * @brief Set all lifecycle default state
  *
  * @return true Return success
  * @return false Return failure
  */
  bool ResetAllLifecyceNodes();

  bool CheckExit();

  bool CloseMappingService();

  bool CanTransform(const std::string & parent_link, const std::string & clild_link);

  void ResetFlags();

  // feedback data
  ExecutorData executor_laser_mapping_data_;

  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> start_client_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> stop_client_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> realtime_pose_client_ {nullptr};

  // Get vision build map available result
  std::shared_ptr<nav2_util::ServiceClient<MapAvailableResult>> mapping_available_client_ {nullptr};
  std::shared_ptr<nav2_util::ServiceClient<MapAvailableResult>> map_delete_client_ {nullptr};

  // vision mapping alive
  rclcpp::Client<LabelParam>::SharedPtr outdoor_client_ {nullptr};

  // record this sensor is open
  bool is_open_realsense_camera_ {false};
  bool is_exit_ {false};
  bool is_slam_service_activate_ {false};
  bool is_realtime_pose_service_activate_ {false};

  // mutex
  std::mutex lifecycle_mutex_;
  std::mutex service_mutex_;
  std::mutex realtime_pose_mutex_;

  // tf
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
};  // class ExecutorVisionMapping
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_VISION_MAPPING_HPP_
