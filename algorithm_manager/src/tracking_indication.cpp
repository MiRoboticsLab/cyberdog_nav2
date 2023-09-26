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
  declare_parameter("feedback_topic", "tracking_target/_action/feedback");
  declare_parameter("valid_range", 3.5);
  std::string feedback_topic_;
  get_parameter("feedback_topic", feedback_topic_);
  get_parameter("valid_range", valid_range_);

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
  rclcpp::PublisherOptions pub_options;
  pub_options.callback_group = topic_callback_group_;
  if (!audio_play_client_->wait_for_service(std::chrono::seconds(2))) {
    ERROR("call audio_execute server not available");
  }
  if (!led_execute_client_->wait_for_service(std::chrono::seconds(2))) {
    ERROR("call led_execute server not available");
  }
  audio_play_extend_pub_ =
    this->create_publisher<protocol::msg::AudioPlayExtend>(
    "speech_play_extend", rclcpp::SystemDefaultsQoS(), pub_options);
  audio_volume_set_client_ = this->create_client<protocol::srv::AudioVolumeSet>(
    "audio_volume_set",
    rmw_qos_profile_services_default, service_callback_group_);

  audio_volume_get_client_ = this->create_client<protocol::srv::AudioVolumeGet>(
    "audio_volume_get",
    rmw_qos_profile_services_default, service_callback_group_);
  if (!audio_volume_set_client_->wait_for_service(std::chrono::seconds(2))) {
    ERROR("call audio_volume_set server not available");
  }
  if (!audio_volume_get_client_->wait_for_service(std::chrono::seconds(2))) {
    ERROR("call audio_volume_get server not available");
  }
  algotask_status_sub_ = this->create_subscription<protocol::msg::AlgoTaskStatus>(
    "algo_task_status", rclcpp::SystemDefaultsQoS(),
    std::bind(&LedManagerNode::AlgoTaskStatus, this, std::placeholders::_1),
    sub_options);
  motion_servo_response_sub_ = this->create_subscription<protocol::msg::MotionServoResponse>(
    "motion_servo_response", rclcpp::SystemDefaultsQoS(),
    std::bind(&LedManagerNode::MotionServoRes, this, std::placeholders::_1),
    sub_options);
  motion_status_sub_ = this->create_subscription<protocol::msg::MotionStatus>(
    "motion_status", rclcpp::SystemDefaultsQoS(),
    std::bind(&LedManagerNode::MotionStatus, this, std::placeholders::_1),
    sub_options);
  feedback_sub_ = create_subscription<mcr_msgs::action::TargetTracking_FeedbackMessage>(
    feedback_topic_, 1,
    std::bind(&LedManagerNode::TargetTrackingFeedback, this, std::placeholders::_1), sub_options);

  http_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  upload_url_ = "device/system/log";
  http_file_client_ =
    this->create_client<protocol::srv::BesHttpSendFile>(
    "bes_http_send_file_srv", rmw_qos_profile_services_default, http_cb_group_);
}

LedManagerNode::~LedManagerNode()
{
  algotask_status_sub_.reset();
  motion_status_sub_.reset();
  feedback_sub_.reset();
}
void LedManagerNode::MotionServoRes(const protocol::msg::MotionServoResponse::SharedPtr msg)
{
  static unsigned int count = 0;
  static int cur_status = static_cast<int>(MotionCode::kNormal);
  if (msg->code == static_cast<int>(MotionCode::kOverTemp) && status_ != Status::kIdle) {
    AudioInfo audio_play{"tracking", true, 0, "注意：电机过热保护,无法站立。"};
    ReqAudio(audio_play);
    // cur_status = static_cast<int>(MotionCode::kOverTemp);
  }
  // if (msg->code == static_cast<int>(MotionCode::kActionFailed) &&
  //   msg->motion_id == 102)
  // {
  //   RCLCPP_INFO(get_logger(), "kActionFailed count %d.", count);
  //   // if(count % 20 == 0) {
  //   AudioInfo audio_play{"tracking", true, 0, "注意：保持安全距离,我要站起来了。"};
  //   ReqAudio(audio_play);
  //   //   count = 0;
  //   // }
  //   // count += 1;
  //   // cur_status = static_cast<int>(MotionCode::kActionFailed);
  // }
  // else {
  //   RCLCPP_INFO(get_logger(), "kNormal .");
  //   cur_status = static_cast<int>(MotionCode::kNormal);
  // }
}
void LedManagerNode::MotionStatus(const protocol::msg::MotionStatus::SharedPtr msg)
{
  // RCLCPP_INFO(get_logger(), "MotionStatus callback. motion_id: %d ",msg->motion_id);
  // if (msg->switch_status == 6) {
  //   AudioInfo audio_play{"tracking", true, 0, "注意：电机过热保护,无法站立。"};
  //   ReqAudio(audio_play);
  // }
  if (msg->motion_id == 118 && status_ != Status::kIdle) {
    RCLCPP_INFO(get_logger(), "Reinforcement standup. ");
    AudioInfo audio_play{"tracking", true, 0, "注意：保持安全距离,我要继续跟随了。"};
    ReqAudio(audio_play);
  }
}
void LedManagerNode::AlgoTaskStatus(const protocol::msg::AlgoTaskStatus::SharedPtr msg)
{
  switch (msg->task_status) {
    case static_cast<int>(ManagerStatus::kExecutingUwbTracking):
      if (status_ == Status::kIdle) {
        AudioVolumeGet();
        AudioVolumeSet(100);
        LedInfo headled_on{1, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{1, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{1, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31000, ""};
        AudioPub(audio_play);
        status_ = Status::kStartUwb;
        int64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        uploadEvent(static_cast<int>(ManagerStatus::kExecutingUwbTracking), true, time);
        first_send_ = false;
      }
      break;

    case static_cast<int>(ManagerStatus::kExecutingHumanTracking):
      if (status_ == Status::kIdle) {
        AudioVolumeGet();
        AudioVolumeSet(100);
        LedInfo headled_on{1, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{1, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{1, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31001, ""};
        AudioPub(audio_play);
        status_ = Status::kStartHuman;
        int64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        uploadEvent(static_cast<int>(ManagerStatus::kExecutingHumanTracking), true, time);
      }
      break;

    case static_cast<int>(ManagerStatus::kExecutingFollowing):
      if (status_ == Status::kIdle) {
        AudioVolumeGet();
        AudioVolumeSet(100);
        LedInfo headled_on{1, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{1, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{1, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31002, ""};
        AudioPub(audio_play);
        status_ = Status::kStartFollow;
        int64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        uploadEvent(static_cast<int>(ManagerStatus::kExecutingFollowing), true, time);
      }
      break;

    case static_cast<int>(ManagerStatus::kStoppingTask):
      if (status_ == Status::kStartUwb) {
        LedInfo headled_on{0, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{0, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{0, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31003, ""};
        AudioPub(audio_play);
        AudioVolumeReSet();
        status_ = Status::kIdle;
        int64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        uploadEvent(static_cast<int>(ManagerStatus::kExecutingUwbTracking), false, time);
      } else if (status_ == Status::kStartHuman) {
        LedInfo headled_on{0, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{0, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{0, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31004, ""};
        AudioPub(audio_play);
        AudioVolumeReSet();
        status_ = Status::kIdle;
        int64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        uploadEvent(static_cast<int>(ManagerStatus::kExecutingHumanTracking), false, time);
      } else if (status_ == Status::kStartFollow) {
        LedInfo headled_on{0, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{0, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{0, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31005, ""};
        AudioPub(audio_play);
        AudioVolumeReSet();
        status_ = Status::kIdle;
        int64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        uploadEvent(static_cast<int>(ManagerStatus::kExecutingFollowing), false, time);
      }
      break;

    case static_cast<int>(ManagerStatus::kIdle):
      if (status_ == Status::kStartUwb) {
        LedInfo headled_on{0, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{0, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{0, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31003, ""};
        AudioPub(audio_play);
        AudioVolumeReSet();
        status_ = Status::kIdle;
        int64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        uploadEvent(static_cast<int>(ManagerStatus::kExecutingUwbTracking), false, time);
      } else if (status_ == Status::kStartHuman) {
        LedInfo headled_on{0, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{0, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{0, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31004, ""};
        AudioPub(audio_play);
        AudioVolumeReSet();
        status_ = Status::kIdle;
        int64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        uploadEvent(static_cast<int>(ManagerStatus::kExecutingHumanTracking), false, time);
      } else if (status_ == Status::kStartFollow) {
        LedInfo headled_on{0, "tracking", 1, 0x02, 0x08, 0xFF, 0XA5, 0X00};
        LedInfo tailled_on{0, "tracking", 2, 0x02, 0x08, 0XFF, 0XA5, 0X00};
        LedInfo miniled_on{0, "tracking", 3, 0x02, 0x30, 0xFF, 0XA5, 0X00};
        ReqLed(headled_on, tailled_on, miniled_on);
        AudioInfo audio_play{"tracking", false, 31005, ""};
        AudioPub(audio_play);
        AudioVolumeReSet();
        status_ = Status::kIdle;
        int64_t time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        uploadEvent(static_cast<int>(ManagerStatus::kExecutingFollowing), false, time);
      }
      break;

    default:
      break;
  }
  // if (msg->task_sub_status == 15 && !first_send_) {
  //   AudioInfo audio_play{"tracking", false, 31037};
  //   AudioPub(audio_play);
  //   first_send_ = true;
  // }
}

void LedManagerNode::TargetTrackingFeedback(
  mcr_msgs::action::TargetTracking_FeedbackMessage::ConstSharedPtr feedbackmsg)
{
  static unsigned int k = 1;
  static float cur_max_spead = 0.0;
  static float last_max_spead = 0.0;
  static float cur_keep_distance = 0.0;
  if (feedbackmsg->feedback.exception_code == nav2_core::DETECTOREXCEPTION) {
    RCLCPP_INFO(get_logger(), "The target is out of sight. Voice prompts are needed.");
    k += 1;
    if (k % 60 == 0) {
      AudioInfo audio_play{"tracking", true, 0, "主人, 我找不到你了，不要丢下我。"};
      AudioPub(audio_play);
    }
  }
  if (feedbackmsg->feedback.current_distance > valid_range_) {
    RCLCPP_INFO(get_logger(), "It is too far away from the target and needs voice prompt.");
    k += 1;
    if (k % 60 == 0) {
      AudioInfo audio_play{"tracking", true, 0, "主人，等等我，跟不上你了。"};
      AudioPub(audio_play);
    }
  }
  if (feedbackmsg->feedback.exception_code == nav2_core::PLANNEREXECPTION ||
    feedbackmsg->feedback.exception_code == nav2_core::CONTROLLEREXECPTION)
  {
    RCLCPP_INFO(get_logger(), "Something is wrong with planning. Voice prompts are needed.");
    AudioInfo audio_play{"tracking", true, 0, "目标丢失，请重启跟随功能。"};
    AudioPub(audio_play);
  }
  if (std::abs(feedbackmsg->feedback.max_x - last_max_spead) > 1e-3) {
    last_max_spead = feedbackmsg->feedback.max_x;
    if (std::abs(feedbackmsg->feedback.max_x - cur_max_spead) > 1e-3) {
      RCLCPP_INFO(get_logger(), "Shift gear!");
      std::string text;
      if (feedbackmsg->feedback.max_x < 1e-3) {
        text = "急停！ ";
        AudioInfo audio_play{"tracking", true, 0, text};
        DogBark(audio_play);
      } else {
        cur_max_spead = feedbackmsg->feedback.max_x;
        text = std::to_string(static_cast<int>(cur_max_spead * 10) / 10) + "点" + std::to_string(
          static_cast<int>(cur_max_spead * 10) % 10) + "米每秒";
      }
      // AudioInfo audio_play{"tracking", true, 0, text};
      // AudioPub(audio_play);
    }
  }
  if (std::abs(feedbackmsg->feedback.keep_distance - cur_keep_distance) > 1e-3) {
    RCLCPP_INFO(get_logger(), "Shift follow distance!");
    cur_keep_distance = feedbackmsg->feedback.keep_distance;
    std::string text = "跟随距离为 " + std::to_string(
      static_cast<int>(cur_keep_distance * 10) / 10) +
      "点" + std::to_string(static_cast<int>(cur_keep_distance * 10) % 10) + "米";
    AudioInfo audio_play{"tracking", true, 0, text};
    AudioPub(audio_play);
  }
}

int LedManagerNode::uploadEvent(int task_type, bool is_start, int64_t time)
{
  if (!http_file_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("bes_http_send_file_srv not available");
    return 2;
  }
  rapidjson::Document json_event(rapidjson::kObjectType);
  CyberdogJson::Add(json_event, "task_type", task_type);
  CyberdogJson::Add(json_event, "is_start", is_start);
  CyberdogJson::Add(json_event, "time", uint64_t(time));
  std::string json_str;
  if (!CyberdogJson::Document2String(json_event, json_str)) {
    ERROR("error while encoding to json");
    return 2;
  }
  auto req = std::make_shared<protocol::srv::BesHttpSendFile::Request>();
  req->method = protocol::srv::BesHttpSendFile::Request::HTTP_METHOD_POST;
  req->url = upload_url_;
  req->info = json_str;
  req->milsecs = 20000;  // 20s
  auto future_result = http_file_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(21));
  if (status == std::future_status::ready) {
    INFO("Success to call bes_http_send_file_srv services.");
  } else {
    WARN(
      "Failed to call bes_http_send_file_srv services.");
    return 2;
  }
  if (future_result.get()->code != 6000) {
    ERROR("Fail to post https request. code: %d", future_result.get()->code);
    return 3;
  }
  INFO("Success to post https request.");
  return 0;
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
