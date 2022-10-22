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

#ifndef ALGORITHM_MANAGER__EXECUTOR_VISION_TRACKING_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_VISION_TRACKING_HPP_

#include <string>
#include <memory>
#include "algorithm_manager/executor_base.hpp"
#include "std_msgs/msg/int32.hpp"
#include "protocol/srv/body_region.hpp"
#include "protocol/srv/algo_manager.hpp"
#include "nav2_util/lifecycle_service_client.hpp"

namespace cyberdog
{
namespace algorithm
{

class ExecutorVisionTracking : public ExecutorBase
{
public:
  using TargetTrackingGoalHandle =
    rclcpp_action::ClientGoalHandle<mcr_msgs::action::TargetTracking>;
  using BodyRegionT = protocol::srv::BodyRegion;
  using Navigation = protocol::action::Navigation;

  explicit ExecutorVisionTracking(std::string node_name);
  void Start(const AlgorithmMGR::Goal::ConstSharedPtr goal) override;
  void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) override;
  void Cancel() override;

  void TrackingSrvCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<BodyRegionT::Request> req,
    std::shared_ptr<BodyRegionT::Response> res);

  uint8_t StartVisionTracking(uint8_t relative_pos, float keep_distance);

  bool TrackingClientCallService(
    rclcpp::Client<protocol::srv::BodyRegion>::SharedPtr & client,
    const sensor_msgs::msg::RegionOfInterest & roi);

  void CallVisionTrackAlgo();
  void OnCancel();

private:
  void HandleGoalResponseCallback(TargetTrackingGoalHandle::SharedPtr goal_handle)
  {
    // (void)goal_handle;
    if (!goal_handle) {
      ERROR("Goal was rejected by server");
      ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
      if (!DeactivateDepsLifecycleNodes()) {
        ERROR("DeactivateDepsLifecycleNodes failed");
      }
      task_abort_callback_();
    } else {
      target_tracking_goal_handle_ = goal_handle;
      INFO("Goal accepted");
    }
  }
  void HandleFeedbackCallback(
    TargetTrackingGoalHandle::SharedPtr,
    const std::shared_ptr<const McrTargetTracking::Feedback> feedback);
  void HandleResultCallback(const TargetTrackingGoalHandle::WrappedResult goal_handle);
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  rclcpp::Node::SharedPtr client_node_;
  // nav2_lifecycle_manager::LifecycleManagerClient client_nav_;
  // std::shared_ptr<nav2_util::LifecycleServiceClient> client_vision_manager_;
  // std::shared_ptr<nav2_util::LifecycleServiceClient> client_tracking_manager_;
  // std::shared_ptr<nav2_util::LifecycleServiceClient> client_realsense_manager_;
  rclcpp::Client<protocol::srv::AlgoManager>::SharedPtr client_vision_algo_;
  rclcpp::Service<BodyRegionT>::SharedPtr service_tracking_object_;
  rclcpp::Client<BodyRegionT>::SharedPtr client_tracking_object_;

  rclcpp_action::Client<mcr_msgs::action::TargetTracking>::SharedPtr
    target_tracking_action_client_;

  mcr_msgs::action::TargetTracking::Goal target_tracking_goal_;

  TargetTrackingGoalHandle::SharedPtr target_tracking_goal_handle_;

  int32_t vision_action_client_feedback_;
  bool start_vision_tracking_;

  rclcpp::TimerBase::SharedPtr feedback_timer_;
};  // class ExecutorVisionTracking
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_VISION_TRACKING_HPP_
