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
#ifndef ALGORITHM_MANAGER__TRACKING_INDICATION_HPP_
#define ALGORITHM_MANAGER__TRACKING_INDICATION_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "protocol/srv/led_execute.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "protocol/msg/audio_play.hpp"
#include "protocol/msg/algo_task_status.hpp"
#include "algorithm_manager/algorithm_task_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace cyberdog
{
namespace algorithm
{
struct LedInfo
{
  bool occupation;
  std::string client;
  uint8_t target;
  uint8_t mode;
  uint8_t effect;
  uint8_t r_value;
  uint8_t g_value;
  uint8_t b_value;
};
struct AudioInfo
{
  std::string module_name;
  bool is_online;
  uint16_t play_id;
};
class LedManagerNode : public rclcpp::Node
{
public:
  enum class Status : uint8_t
  {
    kStartUwb,
    kStartHuman,
    kStartFollow,
    kIdle
  };
  explicit LedManagerNode(std::string name);

private:
  void AlgoTaskStatus(const protocol::msg::AlgoTaskStatus::SharedPtr msg);

  void LedRegister(std::shared_ptr<protocol::srv::LedExecute::Request> req, LedInfo & type)
  {
    req->occupation = type.occupation;
    req->client = type.client;
    req->target = type.target;
    req->mode = type.mode;
    req->effect = type.effect;
    req->r_value = type.r_value;
    req->g_value = type.g_value;
    req->b_value = type.b_value;
  }

  void ReqLed(LedInfo & head, LedInfo & tail, LedInfo & mini)
  {
    auto led_execute = std::make_shared<protocol::srv::LedExecute::Request>();
    LedRegister(led_execute, head);
    auto future_head_led = led_execute_client_->async_send_request(led_execute);
    LedRegister(led_execute, tail);
    auto future_tail_led = led_execute_client_->async_send_request(led_execute);
    LedRegister(led_execute, mini);
    auto future_mini_led = led_execute_client_->async_send_request(led_execute);
    if (future_head_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout ||
      future_tail_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout ||
      future_mini_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout)
    {
      ERROR("Cannot get reponse of set Led");
    }
  }

  void ReqAudio(AudioInfo & req)
  {
    auto audio_execute = std::make_shared<protocol::srv::AudioTextPlay::Request>();
    audio_execute->module_name = req.module_name;
    audio_execute->is_online = req.is_online;
    audio_execute->speech.play_id = req.play_id;
    auto future_audio = audio_play_client_->async_send_request(audio_execute);
    if (future_audio.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
      ERROR("Cannot get reponse of AudioPlay");
    }
  }

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr topic_callback_group_;
  rclcpp::Subscription<protocol::msg::AlgoTaskStatus>::SharedPtr algotask_status_sub_ {nullptr};
  rclcpp::Client<protocol::srv::LedExecute>::SharedPtr led_execute_client_ {nullptr};
  rclcpp::Client<protocol::srv::AudioTextPlay>::SharedPtr audio_play_client_ {nullptr};
  Status status_{Status::kIdle};
  bool first_send_{false};
};
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__TRACKING_INDICATION_HPP_
