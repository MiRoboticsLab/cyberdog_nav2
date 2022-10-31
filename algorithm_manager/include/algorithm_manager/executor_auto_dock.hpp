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

#ifndef ALGORITHM_MANAGER__EXECUTOR_AUTO_DOCK_HPP_
#define ALGORITHM_MANAGER__EXECUTOR_AUTO_DOCK_HPP_

#include <string>
#include <memory>
#include "algorithm_manager/executor_base.hpp"

namespace cyberdog
{
namespace algorithm
{
class ExecutorAutoDock : public ExecutorBase
{
  using SeatAdjustT = protocol::action::SeatAdjust;
  using GoalHandleSeatAdjust = rclcpp_action::ClientGoalHandle<SeatAdjustT>;
  using AutomaticRechargeT = mcr_msgs::action::AutomaticRecharge;
  using GoalHandleAutomaticRecharge = rclcpp_action::ClientGoalHandle<AutomaticRechargeT>;
  using NavigateToPoseT = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPoseT>;

public:
  explicit ExecutorAutoDock(std::string node_name);
  void Start(const AlgorithmMGR::Goal::ConstSharedPtr goal) override;
  void Stop(
    const StopTaskSrv::Request::SharedPtr request,
    StopTaskSrv::Response::SharedPtr response) override;
  void Cancel() override;
  void OnCancel();

private:
  // std::shared_ptr<cyberdog::algorithm::ExecutorBase> exe_laser_loc_ptr_ = nullptr;
  // std::shared_ptr<cyberdog::algorithm::ExecutorBase> exe_ab_nav_ptr_ = nullptr;

  rclcpp_action::Client<SeatAdjustT>::SharedPtr client_seat_adjust_ptr_;
  rclcpp_action::Client<AutomaticRechargeT>::SharedPtr client_laser_charge_ptr_;
  rclcpp_action::Client<NavigateToPoseT>::SharedPtr client_navtopose_ptr_;

  GoalHandleAutomaticRecharge::SharedPtr laser_charge_goal_handle_;
  bool stage1_goal_done_;
  bool stage2_goal_done_;
  bool stage3_goal_done_;
  geometry_msgs::msg::PoseStamped goal_pose;

  // This section is for the stage2 client interface
  void stage2_goal_response_callback(GoalHandleAutomaticRecharge::SharedPtr goal_handle);
  void stage2_feedback_callback(
    GoalHandleAutomaticRecharge::SharedPtr,
    const std::shared_ptr<const AutomaticRechargeT::Feedback> feedback);
  void stage2_result_callback(const GoalHandleAutomaticRecharge::WrappedResult & result);
  void stage2_send_goal();
};  // class ExecutorAutoDock
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__EXECUTOR_AUTO_DOCK_HPP_
