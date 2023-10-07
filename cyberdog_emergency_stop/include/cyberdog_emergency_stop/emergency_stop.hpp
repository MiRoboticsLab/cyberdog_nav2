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

#ifndef CYBERDOG_EMERGENCY_STOP__EMERGENCY_STOP_HPP_
#define CYBERDOG_EMERGENCY_STOP__EMERGENCY_STOP_HPP_

#include <string>
#include <cmath>
#include <chrono>
#include <array>
#include <vector>
#include <unordered_map>
#include <memory>
#include <thread>
#include <mutex>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/uwb_raw.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "protocol/srv/led_execute.hpp"
#include "protocol/srv/motor_temp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "cyberdog_emergency_stop/thread_pool.hpp"

namespace cyberdog
{

namespace cyberdog_emergency_stop
{

/**
 * @class EmergencyStop
 * @brief Forced to stop following in case of emergency
 */
class EmergencyStop : public rclcpp::Node
{
public:
  // 执行优先级： kSTOP > kGEARONE > kFREE
  enum class MotionType
  {
    kSTOP,
    kRECOIL,
    kGEARONE,
    kFREE,
    kROTATE,
    kUNKONW,
  };

  EmergencyStop();
  ~EmergencyStop();

  // 该三个回调函数 在同一个Node中进行同时回调
  /**
   * @brief realsense获得的障碍物信息
   *
   * @param msg realsense的数据
   */
  // void HandleRealsenseData(sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // 雷达数据获得的障碍物信息
  void HandleLidarData(sensor_msgs::msg::LaserScan::SharedPtr msg);

  // 腿式里程计中得速度信息
  void HandleLegOdomVelocity(nav_msgs::msg::Odometry::SharedPtr msg);

  // UWB_raw中得head_tof与标签的角度
  // void HandleUwbAngle(::protocol::msg::UwbRaw::SharedPtr msg);

  // void JudgeUwbKeepPublish(::protocol::msg::UwbRaw::SharedPtr msg);

  // 使能 去能回调函数
  void EnableEmergencyStopCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void SendSpeechRequest();

  /**
   * @brief 急停的灯效交互
   *
   * @param enableLed true---急停灯效；
   *                  false---恢复跟随灯效
   * @return true
   * @return false
   */
  bool LedMention(bool enableLed);

  void EmergencyStopLedParaSetting();
  void TrackingLedParaSetting();

private:
  /**
   * @brief Get the parameter From toml file
   *
   * @param toml_path
   */
  void GetParaFromToml(const std::string & toml_path);

  void ObstaclePointsInitialize();

  bool LidarPointIsInStopArea(const float & current_distance);

  bool LidarPointIsInDownGearArea(
    const int & point_index,
    const float & current_distance);

  void ExcuteStopMotion();

  MotionType pub_motion_type_{MotionType::kUNKONW};

  void PublishMotionCommand(const float & linear_vel);

  // 订阅雷达
  rclcpp::CallbackGroup::SharedPtr callbackgroup_lidar_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_ {nullptr};

  // 订阅速度
  rclcpp::CallbackGroup::SharedPtr callbackgroup_leg_odom_{nullptr};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_odom_sub_ {nullptr};

  rclcpp::CallbackGroup::SharedPtr callbackgroup2_uwb_raw_ {nullptr};
  rclcpp::Subscription<::protocol::msg::UwbRaw>::SharedPtr uwb_raw_sub2_ {nullptr};

  float turning_vel_ {0.5};
  int number_send_turning_ {50};

  // 发送停止运动的pub
  rclcpp::CallbackGroup::SharedPtr callbackgroup_stop_{nullptr};
  rclcpp::Publisher<::protocol::msg::MotionServoCmd>::SharedPtr motion_stop_pub_ {nullptr};

  // 交互（语音、灯效）
  rclcpp::Client<::protocol::srv::AudioTextPlay>::SharedPtr speech_mention_client_{nullptr};
  rclcpp::Client<::protocol::srv::LedExecute>::SharedPtr led_mention_client_{nullptr};

  // L91外围轮廓宽度
  const float l91_width_ {0.32};

  // 急刹
  // 急刹区域范围 (两个参数分别控制范围的长度和宽度)
  float stop_length_ {0.7};
  float stop_lidar_coeff_ {1.0};

  int stop_lidar_index_start_ {};
  int stop_lidar_index_end_ {};
  // 关于激光点数阈值
  int stop_totol_points_threshold_ {25};

  // 急停独有配置
  float stop_linear_vel_threshold_ {0.7};
  int number_send_stop_ {50};

  // 后退独有配置
  bool enable_recoil_ {true};
  int number_send_recoil_ {50};
  float recoil_speed_ {-0.8};

  // 降档(1档区域)
  // 是否启用换挡
  bool enable_gear_ {true};

  // 1档(降档)区域范围
  float area1_length_ {1.5};

  // 降档速度阈值
  float area1_linear_vel_threshold_ {0.3};

  // 关于激光点数阈值
  int area1_totol_points_threshold_ {15};
  int area1_points_as_group_ {7};
  int area1_groups_threshold_ {2};

  // 降档区域中的小区域点组判断
  int obstacle_groups_count_ {0};

  // 线程池计算管理各个区域探测范围
  std::shared_ptr<thread_pool::ThreadPool> thread_pool_ {nullptr};

  // 激光雷达的总点数
  int lidar_total_points_ {0};
  float angle_increment_ {0.0};

  // 统计落在区域一的激光点数
  // int gear1_points_count_ {0};
  // 区域一中(1) (2) (3) (4)各小区域的 障碍物点数
  int points_as_group_ {5};
  std::array<int, 4> gear1_points_count_{0, 0, 0, 0};

  // 锁
  std::mutex mtx_;

  // 统计落在各区域内的激光点数
  int stop_points_count_ {0};

  // 当前线速度
  float linear_vec_ {};

  // 停止、后退的运动指令接口对象
  ::protocol::msg::MotionServoCmd motion_command_;

  std::vector<float> vel_des_ {0.0, 0.0, 0.0};
  std::vector<float> step_height_ {0.2, 0.2};

  bool sending_stop_motion_ {false};

  // 语音提醒
  std::shared_ptr<::protocol::srv::AudioTextPlay::Request> speech_mention_request_ {nullptr};

  // 灯效请求
  std::shared_ptr<::protocol::srv::LedExecute::Request> head_led_request_;
  std::shared_ptr<::protocol::srv::LedExecute::Request> tail_led_request_;

  // 统计降档到升档的时长
  std::chrono::steady_clock::time_point down_gear_start_;
  std::chrono::steady_clock::time_point up_gear_start_;

  // 升档时长阈值，超出该阈值且环境条件满足才进行升档
  int up_gear_period_threshold_ {3};
  float change_gear_period_{0.0};

  // 升档速度阈值
  float free_linear_vel_threshold_ {0.3};

  // 判断障碍物出现到执行对应运动所花费的时间
  float detect_motion_period_{};

  // 该帧激光点数无效值统计
  int invalid_lidar_points_ {0};

  int valid_area1_ {0};
  int valid_free_ {0};

  // 使能服务的标志位
  bool enable_emergency_stop_ {false};
  rclcpp::CallbackGroup::SharedPtr callbackgroup_enable_emergency_stop_ {nullptr};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_emergency_stop_service_ {nullptr};

  // 向 换挡话题 publisher
  std_msgs::msg::Int16 gear_level_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr change_gear_publisher_ {nullptr};
};
}  // namespace cyberdog_emergency_stop
}  // namespace cyberdog

#endif  // CYBERDOG_EMERGENCY_STOP__EMERGENCY_STOP_HPP_
