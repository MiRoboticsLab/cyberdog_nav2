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
#include <utility>
#include <numeric>

#include "cyberdog_emergency_stop/emergency_stop.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;

namespace cyberdog
{

namespace cyberdog_emergency_stop
{
EmergencyStop::EmergencyStop()
: Node("cyberdog_emergency_stop")
{
  callbackgroup_lidar_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_lidar_opt = rclcpp::SubscriptionOptions();
  sub_lidar_opt.callback_group = callbackgroup_lidar_;

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&EmergencyStop::HandleLidarData, this, std::placeholders::_1), sub_lidar_opt);

  callbackgroup_leg_odom_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_leg_odom_opt = rclcpp::SubscriptionOptions();
  sub_leg_odom_opt.callback_group = callbackgroup_leg_odom_;
  leg_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom_out", rclcpp::SystemDefaultsQoS(),
    std::bind(&EmergencyStop::HandleLegOdomVelocity, this, std::placeholders::_1),
    sub_leg_odom_opt);
  // callbackgroup1_uwb_raw_ = this->create_callback_group(
  //   rclcpp::CallbackGroupType::MutuallyExclusive);
  // auto sub_uwb_raw_opt1 = rclcpp::SubscriptionOptions();
  // sub_uwb_raw_opt1.callback_group = callbackgroup1_uwb_raw_;
  // uwb_raw_sub1_ = this->create_subscription<>(
  //  "uwb_raw", rclcpp::SystemDefaultsQoS(),
  //  std::bind(&EmergencyStop::HandleUwbAngle, this, std::placeholders::_1),
  //  sub_uwb_raw_opt1
  // );
  // callbackgroup2_uwb_raw_ = this->create_callback_group(
  //   rclcpp::CallbackGroupType::MutuallyExclusive);
  // auto sub_uwb_raw_opt2 = rclcpp::SubscriptionOptions();
  // sub_uwb_raw_opt2.callback_group = callbackgroup2_uwb_raw_;
  // uwb_raw_sub2_ = this->create_subscription<>(
  //  "uwb_raw", rclcpp::SystemDefaultsQoS(),
  //  std::bind(&EmergencyStop::JudgeUwbKeepPublish, this, std::placeholders::_1),
  //  sub_uwb_raw_opt2
  // );
  motion_stop_pub_ = this->create_publisher<::protocol::msg::MotionServoCmd>(
    "motion_servo_cmd", rclcpp::SystemDefaultsQoS());

  motion_command_.motion_id = 309;   // 305快跑;309跟随;112站立
  motion_command_.step_height = step_height_;
  motion_command_.cmd_source = -1;

  // 若语音、灯效的client将在同一个node中同时发出请求
  // 则需对这两个client指定 不同回调组
  speech_mention_client_ = this->create_client<::protocol::srv::AudioTextPlay>("speech_text_play");

  speech_mention_request_ = std::make_shared<::protocol::srv::AudioTextPlay::Request>();
  speech_mention_request_->module_name = "emergency_stop_mention";
  speech_mention_request_->speech.module_name = "mention";

  led_mention_client_ = this->create_client<::protocol::srv::LedExecute>("led_execute");

  head_led_request_ = std::make_shared<::protocol::srv::LedExecute::Request>();
  head_led_request_->client = "tracking";
  head_led_request_->target = 1;

  tail_led_request_ = std::make_shared<::protocol::srv::LedExecute::Request>();
  tail_led_request_->client = "tracking";
  tail_led_request_->target = 2;

  callbackgroup_enable_emergency_stop_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  INFO("Create enable emergency stop service......");
  enable_emergency_stop_service_ =
    this->create_service<std_srvs::srv::SetBool>(
    "enable_stop_emergency_stop",
    std::bind(
      &EmergencyStop::EnableEmergencyStopCallback, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default,
    callbackgroup_enable_emergency_stop_);

  // 换挡发布者
  change_gear_publisher_ = this->create_publisher<std_msgs::msg::Int16>("gear_level", 10);                                                                                                                 //  NOLINT

  down_gear_start_ = std::chrono::steady_clock::now();
  std::this_thread::sleep_for(std::chrono::seconds(up_gear_period_threshold_));
  up_gear_start_ = std::chrono::steady_clock::now();

  const std::string toml_path = ament_index_cpp::get_package_share_directory(
    "cyberdog_emergency_stop") +
    "/config/configuration.toml";
  toml::value configuration;
  if (!cyberdog::common::CyberdogToml::ParseFile(toml_path, configuration)) {
    ERROR("Cannot parse %s", toml_path.c_str());
    return;
  }
}

EmergencyStop::~EmergencyStop()
{
}

void EmergencyStop::GetParaFromToml(const std::string & toml_path)
{
  INFO("Lidar points number is %d", lidar_total_points_);
  toml::value configuration;
  if (!cyberdog::common::CyberdogToml::ParseFile(toml_path, configuration)) {
    ERROR("Cannot parse %s", toml_path.c_str());
    return;
  }

  toml::value specific_configuration;

  float arctan;
  float current_area_range_start, current_area_range_end;
  int expand_lidar_range = 0;

  // 急停区域(stop)
  common::CyberdogToml::Get(configuration, "stop", specific_configuration);

  stop_length_ = toml::find<float>(specific_configuration, "length");
  stop_lidar_coeff_ = toml::find<float>(specific_configuration, "lidar_coeff");

  stop_totol_points_threshold_ = toml::find<int>(specific_configuration, "totol_points_threshold");

  stop_linear_vel_threshold_ =
    toml::find<float>(specific_configuration, "stop_linear_vel_threshold");

  number_send_stop_ = toml::find<int>(specific_configuration, "send_stop_number");


  // 提取 急停区域的雷达数据 对应的角度范围
  arctan = std::atan((l91_width_ / 2) / stop_length_);
  current_area_range_start = (M_PI / 2 - arctan);
  current_area_range_end = (M_PI / 2 + arctan);

  stop_lidar_index_start_ =
    static_cast<int>((current_area_range_start / M_PI * lidar_total_points_ + 1));
  stop_lidar_index_end_ =
    static_cast<int>((current_area_range_end / M_PI * lidar_total_points_ + 1));

  expand_lidar_range = (stop_lidar_index_start_ - stop_lidar_index_start_ / stop_lidar_coeff_);

  stop_lidar_index_start_ -= expand_lidar_range;
  stop_lidar_index_end_ += expand_lidar_range;

  INFO(
    "Lidar index start and finish are respectively %d, %d", stop_lidar_index_start_,
    stop_lidar_index_end_);

  // 执行后退独有的参数配置
  common::CyberdogToml::Get(configuration, "recoil", specific_configuration);

  enable_recoil_ = toml::find<bool>(specific_configuration, "enable");
  recoil_speed_ = toml::find<float>(specific_configuration, "speed");
  number_send_recoil_ = toml::find<int>(specific_configuration, "send_recoil_number");
  // 空旷区域
  common::CyberdogToml::Get(configuration, "free", specific_configuration);

  up_gear_period_threshold_ = toml::find<int>(specific_configuration, "up_gear_period_threshold");  //  NOLINT

  free_linear_vel_threshold_ = toml::find<float>(
    specific_configuration,
    "free_linear_vel_threshold");

  // 区域一
  common::CyberdogToml::Get(configuration, "area1", specific_configuration);

  enable_gear_ = toml::find<bool>(
    specific_configuration,
    "enable_gear");

  if (enable_gear_) {
    area1_length_ = toml::find<float>(specific_configuration, "length");

    area1_totol_points_threshold_ =
      toml::find<int>(specific_configuration, "totol_points_threshold");
    area1_points_as_group_ = toml::find<int>(specific_configuration, "points_as_group");
    area1_groups_threshold_ = toml::find<int>(specific_configuration, "groups_threshold");
    area1_linear_vel_threshold_ = toml::find<float>(
      specific_configuration,
      "area1_linear_vel_threshold");
  }
  // 跟随根据uwb与L91的angle参数进行旋转
  // common::CyberdogToml::Get(configuration, "turning", specific_configuration);
  // turning_vel_ = toml::find<float>(specific_configuration, "truning_vel");
  // number_send_turning_ = toml::find<int>(specific_configuration, "send_stop_number");

  thread_pool_ = std::make_shared<cyberdog::thread_pool::ThreadPool>(1);
}

void EmergencyStop::ObstaclePointsInitialize()
{
  stop_points_count_ = 0;
  gear1_points_count_[0] = 0;
  gear1_points_count_[1] = 0;
  invalid_lidar_points_ = 0;
}

// bool EmergencyStop::LidarPointIsInStopArea(
//   const int & point_index,
//   const float & current_distance)
// {
//   float angle = point_index * angle_increment_;

//   if (point_index < stop_lidar_index_start_) {
//     INFO("Area is Stop and Right:%f", (stop_length_ / std::tan(stop_lidar_index_start_ * angle_increment_)));  //  NOLINT
//     if (current_distance <
//       ((stop_length_ / std::tan(stop_lidar_index_start_ * angle_increment_)) /
//       std::cos(angle))){
//       // INFO("Stop area1 occurs a obstacle point and i is %d", point_index);
//       ++stop_points_count_;
//       return true;
//     }
//   } else if (point_index >= stop_lidar_index_start_ && point_index < (lidar_total_points_ / 2)) {
//     if (current_distance < (stop_length_ / std::cos(std::fabs(M_PI / 2 - angle)))){
//       // INFO("Stop area2 occurs a obstacle point and i is %d", point_index);
//       ++stop_points_count_;
//       return true;
//     }
//   } else if (point_index >= (lidar_total_points_ / 2) && point_index < stop_lidar_index_end_) {
//     if (current_distance < (stop_length_ / std::cos(std::fabs(angle - M_PI / 2)))){
//       // INFO("Stop area3 occurs a obstacle point and i is %d", point_index);
//       ++stop_points_count_;
//       return true;
//     }
//   } else if (point_index >= stop_lidar_index_end_) {
//     INFO("Area is Stop and Left:%f", (stop_length_ / std::tan(M_PI - stop_lidar_index_end_ * angle_increment_)));  //  NOLINT
//     if (current_distance <
//       ((stop_length_ / std::tan(M_PI - stop_lidar_index_end_ * angle_increment_)) /
//       std::cos(std::fabs(M_PI - angle)))){
//       // INFO("Stop area4 occurs a obstacle point and i is %d", point_index);
//       ++stop_points_count_;
//       return true;
//     }
//   }
//   return false;
// }

// bool EmergencyStop::LidarPointIsInDownGearArea(const int & point_index,
//     const float & current_distance)
// {
//   float angle = point_index * angle_increment_;

//   if (point_index < area1_lidar_index_start_) {
//     // INFO("Area is Gear and Right:%f",  (area1_length_ / std::tan(area1_lidar_index_start_ * angle_increment_)));  //  NOLINT
//     if (current_distance <
//       ((area1_length_ / std::tan(area1_lidar_index_start_ * angle_increment_)) /
//       std::cos(angle))){
//       INFO("Gear area1 occurs a obstacle point and i is %d", point_index);
//       gear1_points_count_[0]++;
//       return true;
//     }
//   } else if (point_index >= area1_lidar_index_start_ && point_index < (lidar_total_points_ / 2)) {  //  NOLINT
//     if (current_distance < (area1_length_ / std::cos(std::fabs(M_PI / 2 - angle)))){
//         INFO("Gear area2 occurs a obstacle point and i is %d", point_index);
//         gear1_points_count_[1]++;
//         return true;
//     }
//   } else if (point_index >= (lidar_total_points_ / 2) && point_index < area1_lidar_index_end_) {
//     if (current_distance < (area1_length_ / std::cos(std::fabs(angle - M_PI / 2)))){
//       INFO("Gear area3 occurs a obstacle point and i is %d", point_index);
//       gear1_points_count_[2]++;
//       return true;
//     }
//   } else if (point_index >= area1_lidar_index_end_) {
//     // INFO("Gear is Gear, Left:%f", specific_area.c_str(), (area1_length_ / std::tan(M_PI - area1_lidar_index_end_ * angle_increment_)));  //  NOLINT
//     if (current_distance <
//       ((area1_length_ / std::tan(M_PI - area1_lidar_index_end_ * angle_increment_)) /
//       std::cos(std::fabs(M_PI - angle)))){
//       INFO("Gear area4 occurs a obstacle point and i is %d", point_index);
//       gear1_points_count_[3]++;
//       return true;
//     }
//   }
//   return false;
// }

bool EmergencyStop::LidarPointIsInStopArea(const float & current_distance)
{
  if (current_distance <= stop_length_) {
    ++stop_points_count_;
    return true;
  }
  return false;
}

bool EmergencyStop::LidarPointIsInDownGearArea(
  const int & point_index,
  const float & current_distance)
{
  if (point_index >= stop_lidar_index_start_ && point_index <= lidar_total_points_ / 2 &&
    current_distance <= area1_length_ && current_distance >= (stop_length_ + 0.1))
  {
    ++gear1_points_count_[0];
    return true;
  } else if (point_index >= lidar_total_points_ / 2 && point_index <= stop_lidar_index_end_ &&  //  NOLINT
    current_distance <= area1_length_ && current_distance >= (stop_length_ + 0.1))
  {
    ++gear1_points_count_[1];
    return true;
  }
  return false;
}

void EmergencyStop::PublishMotionCommand(const float & linear_vel)
{
  rclcpp::WallRate loop_rate(10ms);

  switch (pub_motion_type_) {
    case MotionType::kSTOP:
      {
        vel_des_[0] = 0.0;
        vel_des_[2] = 0.0;
        motion_command_.vel_des = vel_des_;
        motion_command_.value = 8;
        int send_stop_count = 0;
        INFO("The speed is %f m/s when stop command is triggered", linear_vel);
        while (rclcpp::ok() && send_stop_count < number_send_stop_) {
          motion_stop_pub_->publish(motion_command_);
          ++send_stop_count;
          loop_rate.sleep();
        }
        INFO("Sending stop motion command has been finished!");
        break;
      }
    // case MotionType::kROTATE:
    //   {
    //     vel_des_[0] = 0.0;
    //     if(counterclockwise_){
    //       vel_des_[2] = turning_vel_;
    //     } else{
    //       vel_des_[2] = -turning_vel_;
    //     }
    //     motion_command_.vel_des = vel_des_;
    //     int send_truning_count = 0;
    //     INFO("The speed is %f m/s when the turning command is triggered", linear_vel);

    //     while (rclcpp::ok() && send_truning_count < number_send_turning_) {
    //       motion_stop_pub_->publish(motion_command_);
    //       ++send_truning_count;
    //       loop_rate.sleep();
    //     }
    //     INFO("Sending turning motion command has been finished!");
    //     break;
    //   }

    case MotionType::kRECOIL:
      {
        vel_des_[0] = recoil_speed_;
        vel_des_[2] = 0.0;
        motion_command_.vel_des = vel_des_;
        int send_recoil_count = 0;
        INFO("The speed is %f m/s when the recoil command is triggered", linear_vel);

        while (rclcpp::ok() && send_recoil_count < number_send_recoil_) {
          motion_stop_pub_->publish(motion_command_);
          ++send_recoil_count;
          loop_rate.sleep();
        }
        INFO("Sending recoil motion command has been finished!");
        break;
      }
  }
}

void EmergencyStop::ExcuteStopMotion()
{
  thread_pool_->enqueue(std::bind(&EmergencyStop::PublishMotionCommand, this, linear_vec_));
  // std::thread publish_motion_thread(&EmergencyStop::PublishMotionCommand, this, linear_vec_);
  LedMention(true);
  // publish_motion_thread.join();
}

void EmergencyStop::HandleLidarData(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (enable_emergency_stop_) {
    if (lidar_total_points_ == 0) {
      lidar_total_points_ = msg->ranges.size();
      angle_increment_ = msg->angle_increment;
      const std::string toml_path = ament_index_cpp::get_package_share_directory(
        "cyberdog_emergency_stop") +
        "/config/configuration.toml";
      GetParaFromToml(toml_path);
    }

    for (int i = stop_lidar_index_start_; i < stop_lidar_index_end_; ++i) {
      if (msg->intensities[i] == 0.0 || msg->ranges[i] == 0.0) {
        invalid_lidar_points_++;
        continue;
      }

      // 障碍物感知 及 对应的换挡策略
      // thread_pool_->enqueue(
      //   std::bind(
      //     &EmergencyStop::LidarPointIsInStopArea, this, std::ref(i),
      //     std::ref(msg->ranges[i])));
      bool isInstopArea = LidarPointIsInStopArea(msg->ranges[i]);
      if (!isInstopArea) {
        LidarPointIsInDownGearArea(i, msg->ranges[i]);
      }

      if (stop_points_count_ >= stop_totol_points_threshold_ &&
        linear_vec_ >= stop_linear_vel_threshold_ && !sending_stop_motion_)
      {
        INFO("Stop and obstacle points is %d", stop_points_count_);

        // 发送request，tracking_base回调响应换挡至急停档位
        gear_level_.data = 2;
        change_gear_publisher_->publish(gear_level_);

        // speech_mention_request_->is_online = true;
        // speech_mention_request_->speech.play_id = 4;
        // speech_mention_request_->text = "停";
        // SendSpeechRequest();

        sending_stop_motion_ = true;
        pub_motion_type_ = MotionType::kSTOP;
        ExcuteStopMotion();

        break;
      } else if (gear1_points_count_[0] >= area1_points_as_group_ &&  //  NOLINT
        gear1_points_count_[1] >= area1_points_as_group_ &&
        linear_vec_ >= area1_linear_vel_threshold_)
      {
        valid_area1_++;
        // 若当前已为 降档, 不再触发 降档
        if (pub_motion_type_ == MotionType::kGEARONE && valid_area1_ == 2) {
          // speech_mention_request_->is_online = true;
          // speech_mention_request_->speech.play_id = 1;
          // speech_mention_request_->text = "已七";
          // SendSpeechRequest();

          down_gear_start_ = std::chrono::steady_clock::now();
          valid_area1_ = 0;
          break;
        } else if (pub_motion_type_ != MotionType::kGEARONE && valid_area1_ == 2) {
          gear_level_.data = 1;
          change_gear_publisher_->publish(gear_level_);
          INFO("Gear1");
          down_gear_start_ = std::chrono::steady_clock::now();
          // speech_mention_request_->is_online = true;
          // speech_mention_request_->speech.play_id = 1;
          // speech_mention_request_->text = "七";
          // SendSpeechRequest();
          INFO(
            "Gear1 and stop points are %d, %d, %d", gear1_points_count_[0],
            gear1_points_count_[1], stop_points_count_);

          pub_motion_type_ = MotionType::kGEARONE;
          obstacle_groups_count_ = 0;
          valid_free_ = 0;
          valid_area1_ = 0;
        }

        break;
      } else if ((stop_points_count_ <= 9 && gear1_points_count_[0] >= area1_points_as_group_ &&  //  NOLINT
        gear1_points_count_[1] < area1_points_as_group_ && i >= (stop_lidar_index_end_ - 2) &&
        linear_vec_ >= free_linear_vel_threshold_) ||
        (stop_points_count_ <= 9 && gear1_points_count_[0] < area1_points_as_group_ &&
        gear1_points_count_[1] >= area1_points_as_group_ && i >= (stop_lidar_index_end_ - 2) &&
        linear_vec_ >= free_linear_vel_threshold_) ||
        (stop_points_count_ <= 9 && gear1_points_count_[0] < area1_points_as_group_ &&
        gear1_points_count_[1] < area1_points_as_group_ && i >= (stop_lidar_index_end_ - 2) &&
        linear_vec_ >= free_linear_vel_threshold_))
      {
        valid_free_++;
        if (pub_motion_type_ == MotionType::kFREE && valid_free_ == 2) {
          // speech_mention_request_->is_online = true;
          // speech_mention_request_->speech.play_id = 1;
          // speech_mention_request_->text = "已空";
          // SendSpeechRequest();

          up_gear_start_ = std::chrono::steady_clock::now();
          valid_free_ = 0;
          break;
        } else if (pub_motion_type_ != MotionType::kFREE && valid_free_ == 2) {
          // 从上一次降档到升档的时长
          up_gear_start_ = std::chrono::steady_clock::now();
          change_gear_period_ =
            (std::chrono::duration_cast<std::chrono::duration<float>>)(
            up_gear_start_ -
            down_gear_start_).count();                                                                                                  //  NOLINT

          if (change_gear_period_ >= up_gear_period_threshold_) {
            INFO("Free");
            gear_level_.data = 0;
            change_gear_publisher_->publish(gear_level_);
            // speech_mention_request_->is_online = true;
            // speech_mention_request_->speech.play_id = 1;
            // speech_mention_request_->text = "空";
            // SendSpeechRequest();
            change_gear_period_ = 0.0;
            INFO(
              "Free! Area1 and Stop area points are %d, %d, %d", stop_points_count_,
              gear1_points_count_[0], gear1_points_count_[1]);
            INFO("Invalid points count is %d", invalid_lidar_points_);
            pub_motion_type_ = MotionType::kFREE;
          }
          valid_free_ = 0;
          valid_area1_ = 0;
          invalid_lidar_points_ = 0;
        }
        break;
      }
    }
    ObstaclePointsInitialize();
    // detect_motion_period_ =
    //         (std::chrono::duration_cast<std::chrono::duration<float>>)(
    //         std::chrono::steady_clock::now() - lidar_detect_start_ ).count();
    //         INFO("The time it takes detect obstacle is %f seconds", detect_motion_period_);
  }
}

void EmergencyStop::HandleLegOdomVelocity(nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (enable_emergency_stop_) {
    linear_vec_ = msg->twist.twist.linear.x;
    if (linear_vec_ <= 0.1 && sending_stop_motion_) {
      if (enable_recoil_) {
        pub_motion_type_ = MotionType::kRECOIL;
        PublishMotionCommand(linear_vec_);
      }
      sending_stop_motion_ = false;
      LedMention(false);
    }
  }
}

// void EmergencyStop::HandleUwbAngle(::protocol::msg::UwbRaw::SharedPtr msg)
// {
//   if(enable_emergency_stop_ &&  msg->header.frame_id == "head_tof"){
//     // received_uwb_frame_count_++;
//       current_angle_ = msg->angle;
//       if( current_angle_ >= M_PI/6 ){
//         counterclockwise_ = false;
//         judge_uwb_publish_data_ = true;
//         previous_frame_nanosec_ = msg-> header.stamp.nanosec;
//         // 暂停300ms（依据uwb_raw话题hz定），以判断有没有uwb是否仍然在持续发送数据
//         std::this_thread::sleep_for(std::chrono::milliseconds(300));
//         if(continous_publish_uwb_data_){
//           pub_motion_type_ = MotionType::kROTATE;
//           PublishMotionCommand(linear_vec_);
//           judge_uwb_publish_data_ = false;
//         }
//       } else if( current_angle_ <= (-M_PI/6) ){
//         counterclockwise_ = true;
//         judge_uwb_publish_data_ = true;
//         previous_frame_nanosec_ = msg-> header.stamp.nanosec;
//         // 暂停300ms（依据uwb_raw话题hz定），以判断有没有uwb是否仍然在持续发送数据
//         std::this_thread::sleep_for(std::chrono::milliseconds(300));
//         if(continous_publish_uwb_data_){
//           pub_motion_type_ = MotionType::kROTATE;
//           PublishMotionCommand(linear_vec_);
//           judge_uwb_publish_data_ = false;
//         }
//       }
//   }
// }


// void EmergencyStop::JudgeUwbKeepPublish(::protocol::msg::UwbRaw::SharedPtr msg)
// {
//   if(enable_emergency_stop_ && judge_uwb_publish_data_){
//     current_frame_nanosec_ = msg-> header.stamp.nanosec;
//     if(current_frame_nanosec_ != 0 && current_frame_nanosec_ != previous_frame_nanosec_){
//       continous_publish_uwb_data_ = true;
//     } else{
//       continous_publish_uwb_data_ = false;
//     }
//   }
// }

void EmergencyStop::EnableEmergencyStopCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    enable_emergency_stop_ = true;
    response->success = true;
    response->message = "Emergency stop enabled";
    INFO("%s", response->message.c_str());
  } else {
    enable_emergency_stop_ = false;
    response->success = true;
    response->message = "Emergency stop disabled";
    INFO("%s", response->message.c_str());
  }
}

void EmergencyStop::SendSpeechRequest()
{
  while (!speech_mention_client_->wait_for_service(std::chrono::seconds(3))) {
    ERROR("Emergency stop speech mention error, cannot get the speech service.");
    return;
  }

  auto future_result = speech_mention_client_->async_send_request(speech_mention_request_);
  if (future_result.get()->code != 0) {
    ERROR("Emergency stop speech mention error, service response return false.");
    return;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // INFO("Speech mention finished!");
}

bool EmergencyStop::LedMention(bool enableLed)
{
  while (!led_mention_client_->wait_for_service(std::chrono::seconds(2s))) {
    ERROR("Led request failed, cannot get head led service");
    return false;
  }

  if (enableLed) {
    EmergencyStopLedParaSetting();

    auto head_led_future_result = led_mention_client_->async_send_request(head_led_request_);
    if (head_led_future_result.get()->code != 0) {
      ERROR("Head led execute error, response code is %d .", head_led_future_result.get()->code);
      return false;
    }

    auto tail_led_future_result = led_mention_client_->async_send_request(tail_led_request_);
    if (tail_led_future_result.get()->code != 0) {
      ERROR("Tail led execute error, response code is %d .", tail_led_future_result.get()->code);
      return false;
    }
    return true;
  } else {
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    // 恢复跟随灯效
    TrackingLedParaSetting();

    auto head_led_future_result = led_mention_client_->async_send_request(head_led_request_);
    if (head_led_future_result.get()->code != 0) {
      ERROR("Head led release error, response code is %d .", head_led_future_result.get()->code);
      return false;
    }

    auto tail_led_future_result = led_mention_client_->async_send_request(tail_led_request_);
    if (tail_led_future_result.get()->code != 0) {
      ERROR("Tail led release error, response code is %d .", tail_led_future_result.get()->code);
      return false;
    }
    return true;
  }
}

void EmergencyStop::EmergencyStopLedParaSetting()
{
  head_led_request_->occupation = 1;
  head_led_request_->mode = 1;
  head_led_request_->effect = 0xA3;

  tail_led_request_->occupation = 1;
  tail_led_request_->mode = 1;
  tail_led_request_->effect = 0xA3;
}

void EmergencyStop::TrackingLedParaSetting()
{
  head_led_request_->occupation = 1;
  head_led_request_->mode = 0x02;
  head_led_request_->effect = 0x08;
  head_led_request_->r_value = 0XFF;
  head_led_request_->g_value = 0XA5;
  head_led_request_->b_value = 0X00;

  tail_led_request_->occupation = 1;
  tail_led_request_->mode = 0x02;
  tail_led_request_->effect = 0x08;
  tail_led_request_->r_value = 0XFF;
  tail_led_request_->g_value = 0XA5;
  tail_led_request_->b_value = 0X00;
}
}  // namespace cyberdog_emergency_stop
}  // namespace cyberdog
