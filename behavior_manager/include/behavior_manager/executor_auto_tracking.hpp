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

#ifndef BEHAVIOR_MANAGER__EXECUTOR_AUTO_TRACKING_HPP_
#define BEHAVIOR_MANAGER__EXECUTOR_AUTO_TRACKING_HPP_
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "cyberdog_debug/backtrace.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "motion_action/motion_macros.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "protocol/msg/audio_play.hpp"
#include "protocol/srv/led_execute.hpp"

namespace cyberdog
{
namespace algorithm
{

struct BehaviorIdMap
{
  std::string property;
  std::int32_t motion_id;
  std::vector<float> vel_des;
  std::vector<float> rpy_des;
  std::vector<float> pos_des;
  std::vector<float> step_height;
  int32_t duration;
  int32_t wait_time;
  std::string module_name;
  bool is_online;
  std::string text;
  std::string client;
  uint8_t target;
  uint8_t mode;
  uint8_t effect;
};  // struct BehaviorIdMap

class ExecutorAutoTracking
{
public:
  explicit ExecutorAutoTracking(const rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    // node_ = std::make_shared<rclcpp::Node>(node_name);
    // std::thread{[this] {rclcpp::spin(node_);}}.detach();
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    motion_result_client_ = node_->create_client<protocol::srv::MotionResultCmd>(
      "motion_result_cmd", rmw_qos_profile_services_default,
      callback_group_);
    audio_play_client_ = node_->create_client<protocol::srv::AudioTextPlay>(
      "speech_text_play", rmw_qos_profile_services_default,
      callback_group_);
    led_execute_client_ = node_->create_client<protocol::srv::LedExecute>(
      "led_execute", rmw_qos_profile_services_default,
      callback_group_);
    std::string toml_file = ament_index_cpp::get_package_share_directory(
      "behavior_manager") + "/config/parameter.toml";
    toml::value config;
    if(!cyberdog::common::CyberdogToml::ParseFile(toml_file, config)) {
      FATAL("Cannot parse %s", toml_file.c_str());
      exit(-1);
    }
    GET_TOML_VALUE(config, "time", time_);
    std::string behavior_id_map_config = ament_index_cpp::get_package_share_directory(
      "behavior_manager") + "/config/auto_tracking.toml";
    toml::value behavior_ids;
    if (!cyberdog::common::CyberdogToml::ParseFile(behavior_id_map_config, behavior_ids)) {
      FATAL("Cannot parse %s", behavior_id_map_config.c_str());
    }
    if (!behavior_ids.is_table()) {
      FATAL("Toml format error");
      exit(-1);
    }
    toml::value values;

    cyberdog::common::CyberdogToml::Get(behavior_ids, "behavior_ids", values);
    for (size_t i = 0; i < values.size(); i++) {
      auto value = values.at(i);
      int32_t behavior_id;
      BehaviorIdMap behavior_id_map;
      GET_TOML_VALUE(value, "behavior_id", behavior_id);
      GET_TOML_VALUE(value, "property", behavior_id_map.property);
      if (behavior_id_map.property == "motion") {
        GET_TOML_VALUE(value, "motion_id", behavior_id_map.motion_id);
        GET_TOML_VALUE(value, "vel_des", behavior_id_map.vel_des);
        GET_TOML_VALUE(value, "rpy_des", behavior_id_map.rpy_des);
        GET_TOML_VALUE(value, "pos_des", behavior_id_map.pos_des);
        GET_TOML_VALUE(value, "step_height", behavior_id_map.step_height);
        GET_TOML_VALUE(value, "duration", behavior_id_map.duration);
        GET_TOML_VALUE(value, "module_name", behavior_id_map.module_name);
        GET_TOML_VALUE(value, "is_online", behavior_id_map.is_online);
        GET_TOML_VALUE(value, "text", behavior_id_map.text);
        GET_TOML_VALUE(value, "client", behavior_id_map.client);
        GET_TOML_VALUE(value, "target", behavior_id_map.target);
        GET_TOML_VALUE(value, "mode", behavior_id_map.mode);
        GET_TOML_VALUE(value, "effect", behavior_id_map.effect);
      }
      else{
        GET_TOML_VALUE(value, "wait_time", behavior_id_map.wait_time);
        GET_TOML_VALUE(value, "module_name", behavior_id_map.module_name);
        GET_TOML_VALUE(value, "is_online", behavior_id_map.is_online);
        GET_TOML_VALUE(value, "text", behavior_id_map.text);
        GET_TOML_VALUE(value, "client", behavior_id_map.client);
        GET_TOML_VALUE(value, "target", behavior_id_map.target);
        GET_TOML_VALUE(value, "mode", behavior_id_map.mode);
        GET_TOML_VALUE(value, "effect", behavior_id_map.effect);
      }
      behavior_id_map_.emplace(behavior_id, behavior_id_map);
    }
    tm = std::thread{[this]() {this->DoAutoTracking();}};
  }
  ~ExecutorAutoTracking()
  {
    tm.join();
  }
  /**
   * @brief
   * trigger=true时启动自主遛狗，trigger=false时停止自主遛狗
   *
   * @param trigger
   */
  void Execute(bool trigger)
  {
    std::unique_lock<std::mutex> lk(auto_tracking_start_mutex_);
    auto_tracking_start_ = trigger;
    if (auto_tracking_start_) {
      auto_tracking_start_cv_.notify_one();
    }
  }
  void Interupt()
  {
    Execute(false);
  }
  void DoAutoTracking()
  {
    while (rclcpp::ok()) {
      if (!auto_tracking_start_) {
        std::unique_lock<std::mutex> lk(auto_tracking_start_mutex_);
        auto_tracking_start_cv_.wait(lk);
        auto_tracking_start_ = true;
      }
      // 按顺序执行不同的动作列表，围绕目标转圈的动作能被打断
      // WalkAround();
      // 其他的动作实现接口
      // Foo0();
      // Foo1();
      std::map<int32_t, BehaviorIdMap>::iterator iter;
      auto req_motion = std::make_shared<protocol::srv::MotionResultCmd::Request>();
      req_motion->vel_des.resize(3);
      req_motion->rpy_des.resize(3);
      req_motion->pos_des.resize(3);
      req_motion->step_height.resize(2);
      auto req_audio = std::make_shared<protocol::srv::AudioTextPlay::Request>();
      auto req_led = std::make_shared<protocol::srv::LedExecute::Request>();
      if (behavior_id_map_.empty()) {
        return;
      }
      for (iter = behavior_id_map_.begin(); iter != behavior_id_map_.end(); iter++) {
        if (iter->second.property == "motion") {
          req_motion->motion_id = iter->second.motion_id;
          req_motion->vel_des[0] = iter->second.vel_des[0];
          req_motion->vel_des[1] = iter->second.vel_des[1];
          req_motion->vel_des[2] = iter->second.vel_des[2];
          req_motion->rpy_des[0] = iter->second.rpy_des[0];
          req_motion->rpy_des[1] = iter->second.rpy_des[1];
          req_motion->rpy_des[2] = iter->second.rpy_des[2];
          req_motion->pos_des[0] = iter->second.pos_des[0];
          req_motion->pos_des[1] = iter->second.pos_des[1];
          req_motion->pos_des[2] = iter->second.pos_des[2];
          req_motion->step_height[0] = iter->second.step_height[0];
          req_motion->step_height[1] = iter->second.step_height[1];
          req_motion->duration = iter->second.duration;
          req_audio->module_name = iter->second.module_name;
          req_audio->is_online = iter->second.is_online;
          req_audio->text = iter->second.text;
          INFO("req_audio->text=%s", req_audio->text.c_str());
          req_led->client = iter->second.client;
          req_led->target = iter->second.target;
          req_led->mode = iter->second.mode;
          req_led->effect = iter->second.effect;
          auto future_motion = motion_result_client_->async_send_request(req_motion);
          auto future_audio = audio_play_client_->async_send_request(req_audio);
          auto future_led = led_execute_client_->async_send_request(req_led);
          if (future_motion.wait_for(std::chrono::milliseconds(3000)) ==
            std::future_status::timeout)
          {
            FATAL("Motion service failed");
            return;
          }
          if (future_audio.wait_for(std::chrono::milliseconds(3000)) ==
            std::future_status::timeout)
          {
            FATAL("Audio service failed");
            return;
          }
          if (future_led.wait_for(std::chrono::milliseconds(3000)) ==
            std::future_status::timeout)
          {
            FATAL("Led service failed");
            return;
          }
        } 
        else 
        {
          auto base_time = std::chrono::system_clock::now();
          auto task_time = base_time +std::chrono::seconds(20);
          while(auto_tracking_start_){
            if(!first_send){
              first_send = true;
              req_audio->module_name = iter->second.module_name;
              req_audio->is_online = iter->second.is_online;
              req_audio->text = iter->second.text;
              INFO("req_audio->text=%s", req_audio->text.c_str());
              req_led->client = iter->second.client;
              req_led->target = iter->second.target;
              req_led->mode = iter->second.mode;
              req_led->effect = iter->second.effect;
              auto future_motion = motion_result_client_->async_send_request(req_motion);
              auto future_audio = audio_play_client_->async_send_request(req_audio);
              auto future_led = led_execute_client_->async_send_request(req_led);
              if (future_motion.wait_for(std::chrono::milliseconds(3000)) ==
                std::future_status::timeout)
              {
                FATAL("Motion service failed");
                return;
              }
              if (future_audio.wait_for(std::chrono::milliseconds(3000)) ==
                std::future_status::timeout)
              {
                FATAL("Audio service failed");
                return;
              }
              if (future_led.wait_for(std::chrono::milliseconds(3000)) ==
                std::future_status::timeout)
              {
                FATAL("Led service failed");
                return;
              }
            }
            if (std::chrono::system_clock::now() >= task_time){
              first_send = false;
	            break;
            }
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(time_));
      }
    }
  }
  // bool WalkAround()
  // {}
  // 其他的动作实现接口
  // Foo0()
  // Foo1();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_result_client_;
  rclcpp::Client<protocol::srv::AudioTextPlay>::SharedPtr audio_play_client_;
  rclcpp::Client<protocol::srv::LedExecute>::SharedPtr led_execute_client_;
  std::mutex auto_tracking_start_mutex_;
  std::condition_variable auto_tracking_start_cv_;
  bool auto_tracking_start_{false};
  std::map<int32_t, BehaviorIdMap> behavior_id_map_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::thread tm;
  int time_;
  bool stop = false;
  bool first_send = false;
};  // class ExecutorAutoTracking
}  // namespace algorithm
}  // namespace cyberdog
#endif  // BEHAVIOR_MANAGER__EXECUTOR_AUTO_TRACKING_HPP_
