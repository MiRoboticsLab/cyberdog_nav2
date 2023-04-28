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

#include <string>
#include <memory>
#include "algorithm_manager/tracking_indication.hpp"
namespace cyberdog
{
namespace algorithm
{

LedManagerNode::LedManagerNode(std::string name)
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

void LedManagerNode::AlgoTaskStatus(const protocol::msg::AlgoTaskStatus::SharedPtr msg)
{
  switch (msg->task_status) {
    case static_cast<int>(ManagerStatus::kExecutingUwbTracking):
      if (status_ == Status::kIdle) {
        LedInfo headled_on{1, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{1, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{1, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31000};
        ReqAudio(audio_play);
        status_ = Status::kStartUwb;
        first_send_ = false;
      }
      break;

    case static_cast<int>(ManagerStatus::kExecutingHumanTracking):
      if (status_ == Status::kIdle) {
        LedInfo headled_on{1, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{1, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{1, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31001};
        ReqAudio(audio_play);
        status_ = Status::kStartHuman;
      }
      break;

    case static_cast<int>(ManagerStatus::kExecutingFollowing):
      if (status_ == Status::kIdle) {
        LedInfo headled_on{1, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{1, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{1, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31002};
        ReqAudio(audio_play);
        status_ = Status::kStartFollow;
      }
      break;

    case static_cast<int>(ManagerStatus::kStoppingTask):
      if (status_ == Status::kStartUwb) {
        LedInfo headled_on{0, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{0, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{0, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31003};
        ReqAudio(audio_play);
        status_ = Status::kIdle;
      } else if (status_ == Status::kStartHuman) {
        LedInfo headled_on{0, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{0, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{0, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31004};
        ReqAudio(audio_play);
        status_ = Status::kIdle;
      } else if (status_ == Status::kStartFollow) {
        LedInfo headled_on{0, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{0, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{0, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31005};
        ReqAudio(audio_play);
        status_ = Status::kIdle;
      }
      break;

    case static_cast<int>(ManagerStatus::kIdle):
      if (status_ == Status::kStartUwb) {
        LedInfo headled_on{0, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{0, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{0, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31003};
        ReqAudio(audio_play);
        status_ = Status::kIdle;
      } else if (status_ == Status::kStartHuman) {
        LedInfo headled_on{0, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{0, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{0, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31004};
        ReqAudio(audio_play);
        status_ = Status::kIdle;
      } else if (status_ == Status::kStartFollow) {
        LedInfo headled_on{0, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{0, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{0, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31005};
        ReqAudio(audio_play);
        status_ = Status::kIdle;
      }
      break;

    default:
      break;
  }
  if (msg->task_sub_status == 15 && !first_send_) {
    AudioInfo audio_play{"tracking", false, 31037};
    ReqAudio(audio_play);
    first_send_ = true;
  }
}

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
