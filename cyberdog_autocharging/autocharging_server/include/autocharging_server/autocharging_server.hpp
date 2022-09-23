// Copyright (c) 2019 Intel Corporation
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

#ifndef AUTOCHARGING_SERVER__AUTOCHARGING_SERVER_HPP_
#define AUTOCHARGING_SERVER__AUTOCHARGING_SERVER_HPP_

#include <memory>
#include <string>

#include "protocol/action/auto_charging.hpp"
#include "protocol/action/seat_adjust.hpp"
#include "mcr_msgs/action/automatic_recharge.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace cyberdog
{
using AutoChargingT = protocol::action::AutoCharging;
using GoalHandleAutoCharging = rclcpp_action::ServerGoalHandle<AutoChargingT>;
using SeatAdjustT = protocol::action::SeatAdjust;
using GoalHandleSeatAdjust = rclcpp_action::ClientGoalHandle<SeatAdjustT>;
using AutomaticRechargeT = mcr_msgs::action::AutomaticRecharge;
using GoalHandleAutomaticRecharge = rclcpp_action::ClientGoalHandle<AutomaticRechargeT>;
using NavigateToPoseT = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPoseT>;
/**
 * @brief A BT::ActionNodeBase to make decision for tracking mode
 */
class AutoChargingServer : public rclcpp::Node
{
public:
  /**
   * @brief A constructor for mcr_tracking_components::SpinAndSearchAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  explicit AutoChargingServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // This section is for the top server interface
  void handle_accepted(const std::shared_ptr<GoalHandleAutoCharging> goal_handle);
  void execute(const std::shared_ptr<GoalHandleAutoCharging> goal_handle);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleAutoCharging> goal_handle);
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const AutoChargingT::Goal> goal);

  // This section is for the stage1 client interface
  void stage1_goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle);
  void stage1_feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPoseT::Feedback> feedback);
  void stage1_result_callback(const GoalHandleNavigateToPose::WrappedResult & result);
  void stage1_send_goal(const geometry_msgs::msg::PoseStamped pose);

  // This section is for the stage2 client interface
  void stage2_goal_response_callback(GoalHandleAutomaticRecharge::SharedPtr goal_handle);
  void stage2_feedback_callback(
    GoalHandleAutomaticRecharge::SharedPtr,
    const std::shared_ptr<const AutomaticRechargeT::Feedback> feedback);
  void stage2_result_callback(const GoalHandleAutomaticRecharge::WrappedResult & result);
  void stage2_send_goal();

  // This section is for the seat_adjust stage client interface
  void stage3_goal_response_callback(GoalHandleSeatAdjust::SharedPtr goal_handle);
  void stage3_feedback_callback(
    GoalHandleSeatAdjust::SharedPtr,
    const std::shared_ptr<const SeatAdjustT::Feedback> feedback);
  void stage3_result_callback(const GoalHandleSeatAdjust::WrappedResult & result);
  void stage3_send_goal();

  rclcpp_action::Server<AutoChargingT>::SharedPtr action_server_;
  rclcpp_action::Client<SeatAdjustT>::SharedPtr client_seat_adjust_ptr_;
  rclcpp_action::Client<AutomaticRechargeT>::SharedPtr client_stage2_ptr_;
  rclcpp_action::Client<NavigateToPoseT>::SharedPtr client_navtopose_ptr_;

  bool stage1_goal_done_;
  bool stage2_goal_done_;
  bool stage3_goal_done_;
  geometry_msgs::msg::PoseStamped goal_pose;
};

}  // namespace cyberdog

#endif  // AUTOCHARGING_SERVER__AUTOCHARGING_SERVER_HPP_
