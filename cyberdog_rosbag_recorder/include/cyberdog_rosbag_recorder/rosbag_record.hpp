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

#ifndef CYBERDOG_ROSBAG_RECORDER__ROSBAG_RECORD_HPP_
#define CYBERDOG_ROSBAG_RECORDER__ROSBAG_RECORD_HPP_

#include <chrono>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
namespace cyberdog
{
namespace rosbag
{

class TopicsRecorder : public rclcpp::Node
{
public:
  TopicsRecorder();
  ~TopicsRecorder();
  bool Record();

private:
  void SnapshotServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  std::vector<std::string> GetParams();

  bool CheckUseRosbag();

  void Start(const std::vector<std::string> & topics);

  void Stop();

  void StartTask();

  void StopTask();

  std::string GetRosbagFilePath();

  std::string TimeAsStr();

  std::string ExecuteCmdLine(const std::string & str_cmd);

  std::unique_ptr<std::thread> start_thread_{nullptr};
  std::unique_ptr<std::thread> stop_thread_{nullptr};

  std::mutex mutex_;
  bool start_{false};
  bool stop_{false};
  bool use_rosbag_record_{false};
  std::string rosbag_file_path_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr nav_stop_server_{nullptr};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr server_{nullptr};
};
}  // namespace rosbag
}  // namespace cyberdog


#endif  // CYBERDOG_ROSBAG_RECORDER__ROSBAG_RECORD_HPP_
