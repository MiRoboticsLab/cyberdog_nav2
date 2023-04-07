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

#include "cyberdog_rosbag_recorder/rosbag_record.hpp"

#include <stdlib.h>
#include <chrono>
#include <sstream>

#include "rcpputils/split.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace cyberdog
{
namespace rosbag
{

std::string kROSBagDirectory = "./";

TopicsRecorder::TopicsRecorder()
: Node("rosbag_snapshotter"),
  rosbag_file_path_(kROSBagDirectory)
{
  server_ = create_service<std_srvs::srv::SetBool>(
    "rosbag_snapshot_trigger",
    std::bind(
      &TopicsRecorder::SnapshotServiceCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  start_thread_ = std::make_unique<std::thread>(std::bind(&TopicsRecorder::StartTask, this));
  stop_thread_ = std::make_unique<std::thread>(std::bind(&TopicsRecorder::StopTask, this));
}

TopicsRecorder::~TopicsRecorder()
{
}

bool TopicsRecorder::Record()
{
  return true;
}

void TopicsRecorder::SnapshotServiceCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    // std::unique_lock<std::mutex> locker(mutex_);
    start_ = true;
    stop_ = false;
  } else {
    start_ = false;
    stop_ = true;
  }

  response->success = true;
}

std::vector<std::string> TopicsRecorder::GetParams()
{
  std::string config = ament_index_cpp::get_package_share_directory("cyberdog_rosbag_recorder") +
    "/config/topics.toml";

  std::vector<std::string> topics;
  toml::value toml_topics;

  // topics
  if (!cyberdog::common::CyberdogToml::ParseFile(config, toml_topics)) {
    FATAL("Cannot parse %s", config.c_str());
    return topics;
  }

  // use rosbag
  use_rosbag_record_ = toml::find<bool>(toml_topics, "use");
  if (!use_rosbag_record_) {
    WARN("Rosbag recorder is not use, please set flag use: true, you can record.");
    return topics;
  }

  // rosbag file path
  rosbag_file_path_ = toml::find<std::string>(toml_topics, "rosbag_file_path");

  topics = toml::find<std::vector<std::string>>(toml_topics, "topics");
  std::vector<std::string> namespace_topics;

  for (auto topic: topics) {
    auto namespace_topic = this->get_namespace() + std::string("/") + topic;
    INFO("topic: %s", namespace_topic.c_str());
    namespace_topics.push_back(namespace_topic);
  }

  return namespace_topics;
}

bool TopicsRecorder::CheckUseRosbag()
{
  return use_rosbag_record_;
}

void TopicsRecorder::Start(const std::vector<std::string> & topics)
{
  std::string filename = GetRosbagFilePath() + "/" + TimeAsStr();
  std::string cmd = "ros2 bag record -o " + filename + " ";

  for (auto topic : topics) {
    auto cmd_str = topic + " ";
    cmd += cmd_str;
  }

  system(cmd.c_str());
  start_ = false;
}

void TopicsRecorder::Stop()
{
  std::string cmd = "ps -ef | grep \"ros2 bag record -o 20\" | grep -v grep | awk '{print $2}'";
  auto result = ExecuteCmdLine(cmd);

  auto pids = rcpputils::split(result, '\n');
  std::cout << "pids: " << pids.size() << std::endl;

  for (auto pid : pids) {
    std::string kill_cmd = "kill -9 " + pid;
    ExecuteCmdLine(kill_cmd);
  }
  stop_ = false;
}

void TopicsRecorder::StartTask()
{
  while (true) {
    if (start_) {
      auto topics = GetParams();
      Start(topics);
      start_ = false;
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void TopicsRecorder::StopTask()
{
  while (true) {
    if (stop_) {
      Stop();
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

std::string TopicsRecorder::GetRosbagFilePath()
{
  return rosbag_file_path_;
}

std::string TopicsRecorder::TimeAsStr()
{
  std::stringstream msg;
  const auto now = std::chrono::system_clock::now();
  const auto now_in_t = std::chrono::system_clock::to_time_t(now);
  msg << std::put_time(std::localtime(&now_in_t), "%Y-%m-%d-%H-%M-%S");
  return msg.str();
}


std::string TopicsRecorder::ExecuteCmdLine(const std::string & str_cmd)
{
  char buf[10240] = {0};
  FILE * pf = NULL;

  if ( (pf = popen(str_cmd.c_str(), "r")) == NULL) {
    return "";
  }

  std::string strResult;
  while (fgets(buf, sizeof buf, pf)) {
    strResult += buf;
  }

  pclose(pf);

  unsigned int iSize = strResult.size();
  if (iSize > 0 && strResult[iSize - 1] == '\n') {     // linux
    strResult = strResult.substr(0, iSize - 1);
  }

  return strResult;
}


}  // namespace rosbag
}  // namespace cyberdog
