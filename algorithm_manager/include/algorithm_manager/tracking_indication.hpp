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
#ifndef ALGORITHM_MANAGER__TRACKING_INDICATION_HPP_
#define ALGORITHM_MANAGER__TRACKING_INDICATION_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "protocol/srv/led_execute.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "protocol/srv/audio_volume_set.hpp"
#include "protocol/srv/audio_volume_get.hpp"
#include "protocol/msg/audio_play.hpp"
#include "protocol/msg/algo_task_status.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/msg/audio_play_extend.hpp"
#include "protocol/srv/bes_http_send_file.hpp"
#include "algorithm_manager/algorithm_task_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/exceptions.hpp"

using cyberdog::common::CyberdogJson;
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
  std::string text;
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
  enum class MotionCode : int32_t
  {
    kNormal,
    kActionFailed = 3029,
    kOverTemp = 3033
  };
  explicit LedManagerNode(std::string name);
  ~LedManagerNode();

private:
  int uploadEvent(int task_type, bool is_start, int64_t time);
  void AlgoTaskStatus(const protocol::msg::AlgoTaskStatus::SharedPtr msg);
  void MotionServoRes(const protocol::msg::MotionServoResponse::SharedPtr msg);
  void MotionStatus(const protocol::msg::MotionStatus::SharedPtr msg);
  void TargetTrackingFeedback(
    mcr_msgs::action::TargetTracking_FeedbackMessage::ConstSharedPtr feedback);
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
    audio_execute->is_online = false;
    audio_execute->speech.module_name = req.module_name;
    audio_execute->speech.play_id = 4000;
    auto future_audio = audio_play_client_->async_send_request(audio_execute);
    if (future_audio.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
      ERROR("Cannot get reponse of AudioPlay");
    }
    audio_execute->module_name = req.module_name;
    audio_execute->is_online = req.is_online;
    audio_execute->speech.module_name = req.module_name;
    audio_execute->speech.play_id = req.play_id;
    audio_execute->text = req.text;
    auto future_audio1 = audio_play_client_->async_send_request(audio_execute);
    if (future_audio1.wait_for(std::chrono::milliseconds(3000)) == std::future_status::timeout) {
      ERROR("Cannot get reponse of AudioPlay");
    }
  }
  void AudioPub(AudioInfo & req)
  {
    protocol::msg::AudioPlayExtend msg;
    msg.module_name = req.module_name;
    msg.is_online = false;
    msg.speech.module_name = req.module_name;
    msg.speech.play_id = 4000;
    audio_play_extend_pub_->publish(msg);
    rclcpp::sleep_for(1s);
    msg.module_name = req.module_name;
    msg.is_online = req.is_online;
    msg.speech.module_name = req.module_name;
    msg.speech.play_id = req.play_id;
    msg.text = req.text;
    audio_play_extend_pub_->publish(msg);
  }

  void DogBark(AudioInfo & req)
  {
    protocol::msg::AudioPlayExtend msg;
    msg.module_name = req.module_name;
    msg.is_online = false;
    msg.speech.module_name = req.module_name;
    msg.speech.play_id = 4000;
    audio_play_extend_pub_->publish(msg);
  }

  void AudioVolumeSet(const uint8_t & req)
  {
    auto audio_req = std::make_shared<protocol::srv::AudioVolumeSet::Request>();
    audio_req->volume = req;
    auto future_audio = audio_volume_set_client_->async_send_request(audio_req);
    if (future_audio.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
      ERROR("Cannot get reponse of AudioVolumeSet");
    }
  }

  void AudioVolumeReSet(void)
  {
    // std::chrono::seconds delay(5);
    // rclcpp::sleep_for(delay);
    AudioVolumeSet(audio_volume);
  }

  void AudioVolumeGet(void)
  {
    auto audio_req = std::make_shared<protocol::srv::AudioVolumeGet::Request>();
    auto future_audio = audio_volume_get_client_->async_send_request(audio_req);
    if (future_audio.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
      ERROR("Cannot get reponse of AudioVolumeGet");
    }
    auto result = future_audio.get();
    audio_volume = result->volume;
  }

  std::string upload_url_;
  rclcpp::CallbackGroup::SharedPtr http_cb_group_;
  rclcpp::Client<protocol::srv::BesHttpSendFile>::SharedPtr http_file_client_ {nullptr};
  rclcpp::Subscription<mcr_msgs::action::TargetTracking_FeedbackMessage>::SharedPtr feedback_sub_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr topic_callback_group_;
  rclcpp::Subscription<protocol::msg::AlgoTaskStatus>::SharedPtr algotask_status_sub_ {nullptr};
  rclcpp::Subscription<protocol::msg::MotionServoResponse>::SharedPtr motion_servo_response_sub_
  {nullptr};
  rclcpp::Subscription<protocol::msg::MotionStatus>::SharedPtr motion_status_sub_ {nullptr};
  rclcpp::Client<protocol::srv::LedExecute>::SharedPtr led_execute_client_ {nullptr};
  rclcpp::Client<protocol::srv::AudioTextPlay>::SharedPtr audio_play_client_ {nullptr};
  rclcpp::Client<protocol::srv::AudioVolumeSet>::SharedPtr audio_volume_set_client_ {nullptr};
  rclcpp::Client<protocol::srv::AudioVolumeGet>::SharedPtr audio_volume_get_client_ {nullptr};
  rclcpp::Publisher<protocol::msg::AudioPlayExtend>::SharedPtr audio_play_extend_pub_;
  Status status_{Status::kIdle};
  bool first_send_{false};
  double valid_range_;
  uint8_t audio_volume;
};
}  // namespace algorithm
}  // namespace cyberdog
#endif  // ALGORITHM_MANAGER__TRACKING_INDICATION_HPP_
