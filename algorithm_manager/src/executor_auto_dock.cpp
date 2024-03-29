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

#include <memory>
#include <vector>
#include <string>
#include "algorithm_manager/executor_auto_dock.hpp"

namespace cyberdog
{
namespace algorithm
{

ExecutorAutoDock::ExecutorAutoDock(std::string node_name)
: ExecutorBase(node_name)
{
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "-r", std::string("__node:=") + get_name() + "_client", "--"});
  action_client_node_ = std::make_shared<rclcpp::Node>("_", options);
  // exe_laser_loc_ptr_ = std::make_shared<ExecutorVisionMapping>("LaserLocalization");
  // exe_ab_nav_ptr_ = std::make_shared<ExecutorVisionMapping>("NavAB");
  callback_group_ =
    action_client_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  client_navtopose_ptr_ = rclcpp_action::create_client<NavigateToPoseT>(
    action_client_node_,
    "navigate_to_pose");

  client_laser_charge_ptr_ = rclcpp_action::create_client<AutomaticRechargeT>(
    action_client_node_,
    "automatic_recharge");

  client_seat_adjust_ptr_ = rclcpp_action::create_client<SeatAdjustT>(
    action_client_node_,
    "seatadjust");

  audio_play_client_ = action_client_node_->create_client<protocol::srv::AudioTextPlay>(
    "speech_text_play",
    rmw_qos_profile_services_default,
    callback_group_);

  is_power_wp_charging_ = false;
  bms_sub_ = action_client_node_->create_subscription<protocol::msg::BmsStatus>(
    "bms_status", rclcpp::SystemDefaultsQoS(),
    std::bind(&ExecutorAutoDock::tempcallback, this, std::placeholders::_1));

  GetParams();
  INFO("ExecutorAutoDock server is ready");
  std::thread{[this]()
    {rclcpp::spin(action_client_node_);}}
  .detach();
}
bool ExecutorAutoDock::GetParams()
{
  auto local_share_dir = ament_index_cpp::get_package_share_directory("algorithm_manager");
  auto path = local_share_dir + std::string("/config/AutoDock.toml");

  if (!cyberdog::common::CyberdogToml::ParseFile(path.c_str(), params_toml_)) {
    ERROR("Params config file is not in toml format");
    return false;
  }

  stage1_enable_ = toml::find<bool>(params_toml_, "stage1_enable");
  stage2_enable_ = toml::find<bool>(params_toml_, "stage2_enable");
  stage3_enable_ = toml::find<bool>(params_toml_, "stage3_enable");
  return true;
}
void ExecutorAutoDock::Start(const AlgorithmMGR::Goal::ConstSharedPtr goal)
{
  (void)goal;
  OnlineAudioPlay("铁蛋要回充电桩啦");
  INFO("Auto Dock starting");
  ReportPreparationStatus();
  // INFO("Starting LaserLocalization");
  // exe_laser_loc_ptr_ -> Start(goal);
  // INFO("Starting AB Navigation");
  // exe_ab_nav_ptr_ -> Start(goal);
  SetFeedbackCode(500);
  INFO("FeedbackCode: %d", feedback_->feedback_code);

  if (stage2_enable_) {
    if (!ActivateDepsLifecycleNodes(this->get_name(), 50000)) {
      ERROR("ActivateDepsLifecycleNodes failed");
      ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
      if (!DeactivateDepsLifecycleNodes(50000)) {
        ERROR("DeactivateDepsLifecycleNodes failed");
      }
      task_abort_callback_();
      return;
    }
    // stage2 send goal
    if (!stage2_send_goal()) {
      ERROR("stage2 Preparation failed, call task_abort_callback_ and return.");
      return;
    }

    if (stage3_enable_) {
      INFO("stage2 is running, so now we lock stage3.");
      std::unique_lock<std::mutex> lk_stage3(stage3_process_mutex_);
      stage3_process_cv_.wait(lk_stage3);

      if (!stage2_goal_done_) {
        ERROR("stage2 is failed, unlock and skip stage3, return now.");
        return;
      }

      INFO("stage2 is done, unlock stage3 and go on.");
      seat_try_times_ = 2;
      std::unique_lock<std::mutex> lk_stage3_self(stage3_self_process_mutex_);
      while (!is_power_wp_charging_) {
        INFO("in while cycle");
        if (seat_try_times_-- > 0) {
          INFO("still have chance to try, so call stage3_send_goal.");
          stage3_send_goal();

          INFO("lock stage3 in while cycle, wait for notice.");
          stage3_self_process_cv_.wait(lk_stage3_self);
          INFO("unlock stage3 in while cycle.");
          // std::this_thread::sleep_for(std::chrono::seconds(5));
          auto end = std::chrono::steady_clock::now() + std::chrono::seconds(7);     // 6---->7
          INFO("sleep begin");
          {
            while (rclcpp::ok()) {
              auto now = std::chrono::steady_clock::now();
              auto time_left = end - now;
              if (time_left <= std::chrono::seconds(0) || is_power_wp_charging_) {
                break;
              }
            }
          }
          INFO("sleep end");
        } else {
          seat_try_times_ = 0;
          break;
        }
      }

      if (is_power_wp_charging_) {
        INFO("power wp charging success!");
        OnlineAudioPlay("充电成功");
        task_success_callback_();

      } else {
        INFO("power wp charging failed!");
        OnlineAudioPlay("没充上电，帮我检查一下吧");
        task_success_callback_();     // 仅表示回充过程成功
      }
    }
  }

  // uint8_t goal_result = StartVisionTracking(goal->relative_pos, goal->keep_distance);
  // if (goal_result != Navigation::Result::NAVIGATION_RESULT_TYPE_ACCEPT) {
  //   ERROR("ExecutorVisionTracking::Start Error");
  //   ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
  //   task_abort_callback_();
  // }
}

void ExecutorAutoDock::Stop(
  const StopTaskSrv::Request::SharedPtr request,
  StopTaskSrv::Response::SharedPtr response)
{
  (void)request;
  INFO("AutoDock Stop");
  OnCancel();
  response->result = StopTaskSrv::Response::SUCCESS;
}

void ExecutorAutoDock::Cancel()
{
  INFO("AutoDock Cancel");
  OnCancel();
}

void ExecutorAutoDock::OnCancel()
{
  if (laser_charge_goal_handle_ != nullptr) {
    INFO("Cancel laser_charge_goal_handle_");
    auto future_cancel =
      client_laser_charge_ptr_->async_cancel_goal(laser_charge_goal_handle_);
  } else if (seat_adjust_goal_handle_ != nullptr) {
    INFO("Cancel seat_adjust_goal_handle_");
    auto future_cancel =
      client_seat_adjust_ptr_->async_cancel_goal(seat_adjust_goal_handle_);
  } else {
    WARN("laser_charge_goal_handle_ is nullptr");
    if (!DeactivateDepsLifecycleNodes(50000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    StopReportPreparationThread();
    task_cancle_callback_();
    INFO("OnCancel completed");
  }

  // laser_charge_goal_handle_.reset();
}
// This section is for the stage2 client interface
void ExecutorAutoDock::stage2_goal_response_callback(
  GoalHandleAutomaticRecharge::SharedPtr goal_handle)
{
  if (!goal_handle) {
    ERROR("stage2 Goal was rejected by server");
  } else {
    INFO("stage2 Goal accepted by server, waiting for result");
  }
}

void ExecutorAutoDock::stage2_feedback_callback(
  GoalHandleAutomaticRecharge::SharedPtr,
  const std::shared_ptr<const AutomaticRechargeT::Feedback> feedback)
{
  INFO(
    "stage2 feedback current_distance: %f",
    feedback->current_distance);
}

void ExecutorAutoDock::stage2_result_callback(
  const GoalHandleAutomaticRecharge::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      INFO("stage2 Result SUCCEEDED.");
      stage2_goal_done_ = true;
      // OnCancel();
      if (!DeactivateDepsLifecycleNodes(50000)) {
        ERROR("DeactivateDepsLifecycleNodes failed");
      }
      // StopReportPreparationThread();
      // task_success_callback_();
      laser_charge_goal_handle_.reset();
      if (stage3_enable_) {
        stage3_process_cv_.notify_one();
      }
      if (!stage3_enable_) {
        task_success_callback_();
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      ERROR("stage2 Goal was aborted");
      if (!DeactivateDepsLifecycleNodes(50000)) {
        ERROR("DeactivateDepsLifecycleNodes failed");
      }
      laser_charge_goal_handle_.reset();
      if (stage3_enable_) {
        stage3_process_cv_.notify_one();
      }
      task_abort_callback_();
      return;
    case rclcpp_action::ResultCode::CANCELED:
      ERROR("stage2 Goal was canceled");
      if (!DeactivateDepsLifecycleNodes(50000)) {
        ERROR("DeactivateDepsLifecycleNodes failed");
      }
      laser_charge_goal_handle_.reset();
      if (stage3_enable_) {
        stage3_process_cv_.notify_one();
      }
      StopReportPreparationThread();
      task_cancle_callback_();
      INFO("OnCancel completed");
      return;
    default:
      ERROR("stage2 Unknown result code");
      return;
  }
}

bool ExecutorAutoDock::stage2_send_goal()
{
  using namespace std::placeholders;

  stage2_goal_done_ = false;

  if (!client_laser_charge_ptr_) {
    ERROR("Action client not initialized");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    if (!DeactivateDepsLifecycleNodes(50000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    task_abort_callback_();
    return false;
  }

  if (!client_laser_charge_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    ERROR("Action server not available after waiting");
    // stage2_goal_done_ = true;
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    if (!DeactivateDepsLifecycleNodes(50000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    task_abort_callback_();
    return false;
  }

  auto goal_msg = AutomaticRechargeT::Goal();
  goal_msg.behavior_tree = "";

  INFO("stage2 sending goal");

  auto send_goal_options = rclcpp_action::Client<AutomaticRechargeT>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&ExecutorAutoDock::stage2_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&ExecutorAutoDock::stage2_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&ExecutorAutoDock::stage2_result_callback, this, _1);
  auto future_goal_handle = client_laser_charge_ptr_->async_send_goal(goal_msg, send_goal_options);
  INFO("client_laser_charge_ptr_ async_send_goal");
  if (future_goal_handle.wait_for(std::chrono::milliseconds(5000)) == std::future_status::timeout) {
    ERROR("Cannot Get result client_laser_charge_ptr_");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    if (!DeactivateDepsLifecycleNodes(50000)) {
      ERROR("DeactivateDepsLifecycleNodes async_send_goal failed");
    }
    task_abort_callback_();
    return false;
  } else {
    INFO("client_laser_charge_ptr_  success");
  }
  // Get the goal handle and save so that we can check on completion in the
  // timer callback
  laser_charge_goal_handle_ = future_goal_handle.get();
  if (!laser_charge_goal_handle_) {
    ERROR("Goal was rejected by server");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    if (!DeactivateDepsLifecycleNodes(50000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    task_abort_callback_();
    return false;
  }
  return true;
}

// This section is for the seat_adjust stage client interface
void ExecutorAutoDock::stage3_goal_response_callback(GoalHandleSeatAdjust::SharedPtr goal_handle)
{
  if (!goal_handle) {
    ERROR("stage3 Goal was rejected by server");
  } else {
    INFO("stage3 Goal accepted by server, waiting for result");
  }
}

void ExecutorAutoDock::stage3_feedback_callback(
  GoalHandleSeatAdjust::SharedPtr,
  const std::shared_ptr<const SeatAdjustT::Feedback> feedback)
{
  INFO("stage3 feedback count: %d", feedback->count);
}

void ExecutorAutoDock::stage3_result_callback(const GoalHandleSeatAdjust::WrappedResult & result)
{
  stage3_goal_done_ = true;
  INFO("seat retry has %d times left", seat_try_times_);
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      INFO("stage3 Result received.");
      seat_adjust_goal_handle_.reset();
      INFO("stage3_self_process_cv_.notify_one");
      stage3_self_process_cv_.notify_one();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      ERROR("stage3 Goal was aborted");
      seat_adjust_goal_handle_.reset();
      seat_try_times_ = 0;
      stage3_self_process_cv_.notify_one();
      task_abort_callback_();
      return;
    case rclcpp_action::ResultCode::CANCELED:
      ERROR("stage3 Goal was canceled");
      seat_adjust_goal_handle_.reset();
      seat_try_times_ = 0;
      stage3_self_process_cv_.notify_one();
      StopReportPreparationThread();
      task_cancle_callback_();
      return;
    default:
      ERROR("stage3 Unknown result code");
      return;
  }
}

bool ExecutorAutoDock::stage3_send_goal()
{
  using namespace std::placeholders;

  stage3_goal_done_ = false;

  if (!client_seat_adjust_ptr_) {
    ERROR("Action client not initialized");
    return false;
  }

  if (!client_seat_adjust_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    ERROR("Action server not available after waiting");
    stage3_goal_done_ = true;
    return false;
  }

  auto goal_msg = SeatAdjustT::Goal();
  goal_msg.start = goal_msg.SEATADJUST_GOAL_TYPE_START;

  INFO("stage3 sending goal");

  auto send_goal_options = rclcpp_action::Client<SeatAdjustT>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&ExecutorAutoDock::stage3_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&ExecutorAutoDock::stage3_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&ExecutorAutoDock::stage3_result_callback, this, _1);
  auto future_goal_handle = client_seat_adjust_ptr_->async_send_goal(
    goal_msg,
    send_goal_options);
  INFO("client_seat_adjust_ptr_ async_send_goal");
  if (future_goal_handle.wait_for(std::chrono::milliseconds(7000)) == std::future_status::timeout) {
    ERROR("Cannot Get result client_seat_adjust_ptr_");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    if (!DeactivateDepsLifecycleNodes(50000)) {
      ERROR("DeactivateDepsLifecycleNodes async_send_goal failed");
    }
    task_abort_callback_();
    return false;
  } else {
    INFO("client_seat_adjust_ptr_  success");
  }
  // Get the goal handle and save so that we can check on completion in the
  // timer callback
  seat_adjust_goal_handle_ = future_goal_handle.get();
  if (!seat_adjust_goal_handle_) {
    ERROR("Goal was rejected by server");
    ReportPreparationFinished(AlgorithmMGR::Feedback::TASK_PREPARATION_FAILED);
    if (!DeactivateDepsLifecycleNodes(50000)) {
      ERROR("DeactivateDepsLifecycleNodes failed");
    }
    task_abort_callback_();
    return false;
  }
  return true;
}

void ExecutorAutoDock::tempcallback(const protocol::msg::BmsStatus::SharedPtr msg)     // add ym
{
  // INFO("Receive bms_status %d ", msg->power_wp_charging);
  is_power_wp_charging_ = msg->power_wp_charging;
}


void ExecutorAutoDock::OnlineAudioPlay(const std::string & text)
{
  INFO("call OnlineAudioPlay");
  static bool playing = false;
  if (playing) {
    return;
  }
  auto request = std::make_shared<protocol::srv::AudioTextPlay::Request>();
  request->is_online = true;
  request->module_name = get_name();
  request->text = text;
  playing = true;
  auto callback = [this](rclcpp::Client<protocol::srv::AudioTextPlay>::SharedFuture future) {
      playing = false;
      INFO("Audio play result: %s", future.get()->status == 0 ? "success" : "failed");
    };
  auto future = audio_play_client_->async_send_request(request, callback);
  if (future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::timeout) {
    playing = false;
    ERROR("Cannot get response from AudioPlay");
  }
}

}  // namespace algorithm
}  // namespace cyberdog
