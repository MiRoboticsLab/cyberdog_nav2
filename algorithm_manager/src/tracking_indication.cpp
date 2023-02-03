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

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "protocol/srv/led_execute.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "protocol/msg/audio_play.hpp"
#include "protocol/msg/algo_task_status.hpp"
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
  explicit LedManagerNode(std::string name)
  : Node(name)
  {
    service_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    topic_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    led_execute_client_ =
      this->create_client<protocol::srv::LedExecute>(
      "led_execute",
      rmw_qos_profile_services_default, service_callback_group_);
    audio_play_client_ =
      this->create_client<protocol::srv::AudioTextPlay>(
      "speech_text_play",
      rmw_qos_profile_services_default, service_callback_group_);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = topic_callback_group_;
    if (!audio_play_client_->wait_for_service(std::chrono::seconds(2))) {
      ERROR("call audio_execute server not available");
    }
    if (!led_execute_client_->wait_for_service(std::chrono::seconds(2))) {
      ERROR("call audio_execute server not available");
    }
    algotask_status_sub_ = this->create_subscription<protocol::msg::AlgoTaskStatus>(
      "algo_task_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&LedManagerNode::AlgoTaskStatus, this, std::placeholders::_1),
      sub_options);
  }

private:
  void AlgoTaskStatus(const protocol::msg::AlgoTaskStatus::SharedPtr msg)
  {
    if (msg->task_status == 11 && !follow_tags_start_) {
      LedInfo headled_on{1, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
      LedInfo tailled_on{1, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
      LedInfo miniled_on{1, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
      ReqService(headled_on, tailled_on, miniled_on);
      auto audio_execute = std::make_shared<protocol::srv::AudioTextPlay::Request>();
      audio_execute->module_name = this->get_name();
      audio_execute->is_online = false;
      audio_execute->speech.play_id = 31000;
      auto future_audio = audio_play_client_->async_send_request(audio_execute);
      if (future_audio.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
        ERROR("Cannot get reponse of start_uwb_tracking AudioPlay");
      }
      follow_tags_start_ = true;
      status_ = Status::kStartUwb;
    } else if (msg->task_status == 13 && !follow_person_start_) {
      LedInfo headled_on{1, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
      LedInfo tailled_on{1, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
      LedInfo miniled_on{1, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
      ReqService(headled_on, tailled_on, miniled_on);
      auto audio_execute = std::make_shared<protocol::srv::AudioTextPlay::Request>();
      audio_execute->module_name = this->get_name();
      audio_execute->is_online = false;
      audio_execute->speech.play_id = 31001;
      auto future_audio = audio_play_client_->async_send_request(audio_execute);
      if (future_audio.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
        ERROR("Cannot get reponse of start_human_tracking AudioPlay");
      }
      follow_person_start_ = true;
      status_ = Status::kStartHuman;
    } else if (msg->task_status == 3 && !follow_object_start_) {
      LedInfo headled_on{1, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
      LedInfo tailled_on{1, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
      LedInfo miniled_on{1, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
      ReqService(headled_on, tailled_on, miniled_on);
      auto audio_execute = std::make_shared<protocol::srv::AudioTextPlay::Request>();
      audio_execute->module_name = this->get_name();
      audio_execute->is_online = false;
      audio_execute->speech.play_id = 31002;
      auto future_audio = audio_play_client_->async_send_request(audio_execute);
      if (future_audio.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
        ERROR("Cannot get reponse of start_follow AudioPlay");
      }
      follow_object_start_ = true;
      status_ = Status::kStartFollow;
    } else if (msg->task_status == 103 && status_ != Status::kIdle) {
      auto led_execute = std::make_shared<protocol::srv::LedExecute::Request>();
      led_execute->occupation = 0;
      led_execute->client = "tracking";
      led_execute->target = 1;
      auto future_head_led = led_execute_client_->async_send_request(led_execute);
      led_execute->target = 2;
      auto future_tail_led = led_execute_client_->async_send_request(led_execute);
      led_execute->target = 3;
      auto future_mini_led = led_execute_client_->async_send_request(led_execute);
      if (future_head_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout ||
        future_tail_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout ||
        future_mini_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout)
      {
        ERROR("Cannot get reponse of recovery Led");
      }
      auto audio_execute = std::make_shared<protocol::srv::AudioTextPlay::Request>();
      audio_execute->module_name = this->get_name();
      audio_execute->is_online = false;
      if (status_ == Status::kStartUwb) {
        audio_execute->speech.play_id = 31003;
      } else if (status_ == Status::kStartHuman) {
        audio_execute->speech.play_id = 31004;
      } else {
        audio_execute->speech.play_id = 31005;
      }
      auto future_audio = audio_play_client_->async_send_request(audio_execute);
      if (future_audio.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
        ERROR("Cannot get reponse of stop_tracking AudioPlay");
      }
      follow_object_start_ = false;
      follow_tags_start_ = false;
      follow_person_start_ = false;
      status_ = Status::kIdle;
    } else if (msg->task_status == 101 && status_ == Status::kStartUwb) {
      auto led_execute = std::make_shared<protocol::srv::LedExecute::Request>();
      led_execute->occupation = 0;
      led_execute->client = "tracking";
      led_execute->target = 1;
      auto future_head_led = led_execute_client_->async_send_request(led_execute);
      led_execute->target = 2;
      auto future_tail_led = led_execute_client_->async_send_request(led_execute);
      led_execute->target = 3;
      auto future_mini_led = led_execute_client_->async_send_request(led_execute);
      if (future_head_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout ||
        future_tail_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout ||
        future_mini_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout)
      {
        ERROR("Cannot get reponse of recovery Led from unsuccessful startup");
      }
      auto audio_execute = std::make_shared<protocol::srv::AudioTextPlay::Request>();
      audio_execute->module_name = this->get_name();
      audio_execute->is_online = false;
      audio_execute->speech.play_id = 31003;
      auto future_audio = audio_play_client_->async_send_request(audio_execute);
      if (future_audio.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
        ERROR("Cannot get reponse of stop uwb_tracking AudioPlay from unsuccessful startup");
      }
      follow_tags_start_ = false;
      status_ = Status::kIdle;
    } else if (msg->task_status == 101 && status_ == Status::kStartHuman) {
      auto led_execute = std::make_shared<protocol::srv::LedExecute::Request>();
      led_execute->occupation = 0;
      led_execute->client = "tracking";
      led_execute->target = 1;
      auto future_head_led = led_execute_client_->async_send_request(led_execute);
      led_execute->target = 2;
      auto future_tail_led = led_execute_client_->async_send_request(led_execute);
      led_execute->target = 3;
      auto future_mini_led = led_execute_client_->async_send_request(led_execute);
      if (future_head_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout ||
        future_tail_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout ||
        future_mini_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout)
      {
        ERROR("Cannot get reponse of recovery Led from unsuccessful startup");
      }
      auto audio_execute = std::make_shared<protocol::srv::AudioTextPlay::Request>();
      audio_execute->module_name = this->get_name();
      audio_execute->is_online = false;
      audio_execute->speech.play_id = 31004;
      auto future_audio = audio_play_client_->async_send_request(audio_execute);
      if (future_audio.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
        ERROR("Cannot get reponse of stop human_tracking AudioPlay from unsuccessful startup");
      }
      follow_person_start_ = false;
      status_ = Status::kIdle;
    } else if (msg->task_status == 101 && status_ == Status::kStartFollow) {
      auto led_execute = std::make_shared<protocol::srv::LedExecute::Request>();
      led_execute->occupation = 0;
      led_execute->client = "tracking";
      led_execute->target = 1;
      auto future_head_led = led_execute_client_->async_send_request(led_execute);
      led_execute->target = 2;
      auto future_tail_led = led_execute_client_->async_send_request(led_execute);
      led_execute->target = 3;
      auto future_mini_led = led_execute_client_->async_send_request(led_execute);
      if (future_head_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout ||
        future_tail_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout ||
        future_mini_led.wait_for(std::chrono::seconds(2)) == std::future_status::timeout)
      {
        ERROR("Cannot get reponse of recovery Led from unsuccessful startup");
      }
      auto audio_execute = std::make_shared<protocol::srv::AudioTextPlay::Request>();
      audio_execute->module_name = this->get_name();
      audio_execute->is_online = false;
      audio_execute->speech.play_id = 31005;
      auto future_audio = audio_play_client_->async_send_request(audio_execute);
      if (future_audio.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
        ERROR("Cannot get reponse of stop follow AudioPlay from unsuccessful startup");
      }
      follow_object_start_ = false;
      status_ = Status::kIdle;
    }
  }

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

  void ReqService(LedInfo & head, LedInfo & tail, LedInfo & mini)
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

private:
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr topic_callback_group_;
  rclcpp::Subscription<protocol::msg::AlgoTaskStatus>::SharedPtr algotask_status_sub_ {nullptr};
  rclcpp::Client<protocol::srv::LedExecute>::SharedPtr led_execute_client_ {nullptr};
  rclcpp::Client<protocol::srv::AudioTextPlay>::SharedPtr audio_play_client_ {nullptr};
  bool follow_tags_start_ {false};
  bool follow_person_start_ {false};
  bool follow_object_start_ {false};
  Status status_{Status::kIdle};
};
}  // namespace algorithm
}  // namespace cyberdog

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<cyberdog::algorithm::LedManagerNode>("tracking_interation");
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
