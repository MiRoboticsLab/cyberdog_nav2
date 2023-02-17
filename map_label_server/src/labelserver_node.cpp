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

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <regex>
#include <unordered_set>

#include "map_label_server/labelserver_node.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "filesystem/filesystem.hpp"
#include "cyberdog_common/cyberdog_json.hpp"

namespace CYBERDOG_NAV
{
LabelServer::LabelServer()
: rclcpp::Node("LabelServer")
{
  label_store_ = std::make_shared<cyberdog::navigation::LabelStore>();
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // set map label
  set_label_server_ = this->create_service<protocol::srv::SetMapLabel>(
    "set_label",
    std::bind(
      &LabelServer::HandleSetLabelServiceCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_default, callback_group_);

  // get map and map's labels
  get_label_server_ = this->create_service<protocol::srv::GetMapLabel>(
    "get_label",
    std::bind(
      &LabelServer::HandleGetLabelServiceCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_default, callback_group_);

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // outdoor flag
  outdoor_server_ = this->create_service<protocol::srv::SetMapLabel>(
    "outdoor",
    std::bind(
      &LabelServer::HandleOutdoor, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_default, callback_group_);
}

LabelServer::~LabelServer() {}

void LabelServer::HandleGetLabelServiceCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<protocol::srv::GetMapLabel::Request> request,
  std::shared_ptr<protocol::srv::GetMapLabel::Response> response)
{
  std::unique_lock<std::mutex> ulk(mut);
  response->success = protocol::srv::GetMapLabel_Response::RESULT_SUCCESS;

  INFO("----------GetLabel----------");
  INFO("request map_name : %s", request->map_name.c_str());

  std::string map_name = GetMapName(label_store_->map_label_directory());
  std::string map_filename = label_store_->map_label_directory() + map_name + ".pgm";
  std::string label_filename = label_store_->map_label_directory() + map_name + ".json";
  std::string map_yaml_config = label_store_->map_label_directory() + map_name + ".yaml";

  if (!filesystem::exists(label_filename)) {
    ERROR("label json filename(%s) is not exist.", label_filename.c_str());
    response->success = protocol::srv::GetMapLabel_Response::RESULT_FAILED;
    return;
  }

  // Load map's labels
  std::vector<protocol::msg::Label> labels;
  bool is_outdoor = false;
  label_store_->Read(label_filename, labels, is_outdoor);
  if (!labels.empty()) {
    for (auto label : labels) {
      response->label.labels.push_back(label);
    }
  }

  if (is_outdoor) {
    response->success = CheckVisonMapStatus();
  } else {
    if (map_name.empty()) {
      WARN("User have not create map, map %s not exist.", map_name.c_str());
      response->success = 2;
      return;
    }
  }

  // Load map's yaml config
  nav_msgs::msg::OccupancyGrid map;
  bool ok = LoadMapMetaInfo(map_yaml_config, map);
  if (!ok) {
    WARN("Map yaml config file (%s) not exist.", map_yaml_config.c_str());
    response->success = protocol::srv::GetMapLabel_Response::RESULT_FAILED;
    return;
  }

  if (response->success == 0) {
    // Set response result.
    response->label.map.info.resolution = map.info.resolution;
    response->label.map.info.width = map.info.width;
    response->label.map.info.height = map.info.height;
    response->label.map.info.origin = map.info.origin;
    response->label.map.data = map.data;
  }

  // is_outdoor
  response->label.is_outdoor = is_outdoor;
  response->label.map_name = map_name;
  INFO("Get outdoor value : %d", is_outdoor);

  // publish map
  occ_pub_->publish(map);
}

void LabelServer::HandleSetLabelServiceCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<protocol::srv::SetMapLabel::Request> request,
  std::shared_ptr<protocol::srv::SetMapLabel::Response> response)
{
  INFO("----------SetLabel----------");
  INFO("request map_name: %s", request->label.map_name.c_str());

  std::string map_filename = label_store_->map_label_directory() + request->label.map_name + ".pgm";
  std::string label_filename = label_store_->map_label_directory() + request->label.map_name +
    ".json";

  if (!label_store_->IsExist(map_filename)) {
    INFO("Map not exist, not allow set label function.");
    response->success = protocol::srv::GetMapLabel_Response::RESULT_FAILED;
    return;
  }

  // remove map and label tag
  if (request->only_delete && request->label.labels.size() == 0) {
    INFO("Removing map : %s", request->label.map_name.c_str());
    RemoveMap(label_store_->map_label_directory());
    response->success = protocol::srv::SetMapLabel_Response::RESULT_SUCCESS;
    return;
  }

  // check label file exist
  if (!label_store_->IsExist(label_filename)) {
    ERROR("map label json file(%s) not exist", label_filename.c_str());
    response->success = protocol::srv::SetMapLabel_Response::RESULT_FAILED;
    return;
  }

  // remove label
  if (request->only_delete && request->label.labels.size()) {
    for (auto label : request->label.labels) {
      INFO("Removing label: %s", label.label_name.c_str());
      bool success = label_store_->RemoveLabel(label_filename, label.label_name.c_str());
      if (!success) {
        ERROR("Remove label: %s failed", label.label_name.c_str());
        response->success = protocol::srv::SetMapLabel_Response::RESULT_FAILED;
        return;
      } else {
        ERROR("Remove label: %s success", label.label_name.c_str());
      }
    }
    response->success = protocol::srv::SetMapLabel_Response::RESULT_SUCCESS;
    return;
  }

  // Check multi tag, if exit multi label tag return failure
  bool legality = CheckDuplicateTags(request->label.labels);
  if (!legality) {
    response->success = protocol::srv::SetMapLabel_Response::RESULT_FAILED;
    ERROR("User user save more than one of the same tags");
    return;
  }

  // set `map_name` in json file(doc)
  rapidjson::Document doc(rapidjson::kObjectType);
  label_store_->SetMapName(request->label.map_name, doc);

  // outdoor
  INFO("Request set is_outdoor value : %d", request->label.is_outdoor);
  bool outdoor = false;
  GetOutdoorValue(label_filename, outdoor);
  label_store_->SetOutdoor(outdoor, doc);

  for (size_t i = 0; i < request->label.labels.size(); i++) {
    // print
    INFO(
      "Saving label [%s] : [%f, %f]",
      request->label.labels[i].label_name.c_str(),
      request->label.labels[i].physic_x,
      request->label.labels[i].physic_y);

    // save label
    auto label = std::make_shared<protocol::msg::Label>();
    label->set__physic_x(request->label.labels[i].physic_x);
    label->set__physic_y(request->label.labels[i].physic_y);
    label_store_->AddLabel(label_filename, request->label.labels[i].label_name, label, doc);
  }
  // save
  label_store_->Write(label_filename, doc);
  response->success = protocol::srv::SetMapLabel_Response::RESULT_SUCCESS;
}

bool LabelServer::LoadMapMetaInfo(const std::string & map_name, nav_msgs::msg::OccupancyGrid & map)
{
  auto status = nav2_map_server::loadMapFromYaml(map_name, map);
  if (status != nav2_map_server::LOAD_MAP_STATUS::LOAD_MAP_SUCCESS) {
    WARN("Load map yaml config error.");
    return false;
  }

  INFO("Get yaml map success.");
  INFO("resolution : %f", map.info.resolution);
  INFO("width : %d", map.info.width);
  INFO("height : %d", map.info.height);
  INFO("map.data size : %ld", map.data.size());
  return true;
}

bool LabelServer::RemoveMap(const std::string & map_name_directory)
{
  filesystem::remove_all(map_name_directory);
  return true;
}

std::string LabelServer::GetMapName(const std::string & map_name_directory)
{
  std::string mapname = "";
  if (!filesystem::exists(map_name_directory)) {
    ERROR("directory is not exist.");
    return mapname;
  }

  bool find = false;
  std::regex file_suffix("(.*)(.pgm)");   // *.pgm
  filesystem::path path(map_name_directory);
  for (auto filename : filesystem::directory_iterator(path)) {
    if (std::regex_match(filename.path().c_str(), file_suffix)) {
      INFO("Get map filename : %s", filename.path().c_str());
      find = true;
      mapname = filename.path().c_str();
      break;
    }
  }

  if (find) {
    int pos = mapname.find_last_of('/');
    std::string filename(mapname.substr(pos + 1));
    mapname = filename.substr(0, filename.rfind("."));
  }

  return mapname;
}

void LabelServer::HandleOutdoor(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<protocol::srv::SetMapLabel::Request> request,
  std::shared_ptr<protocol::srv::SetMapLabel::Response> response)
{
  if (request->label.is_outdoor) {
    INFO("Handle vision outdoor request");
  } else {
    INFO("Handle lidar outdoor request");
  }

  if (request->label.map_name.empty()) {
    ERROR("map filename %s is empty", request->label.map_name.c_str());
    response->success = false;
    return;
  }

  std::string label_filename = label_store_->map_label_directory() + request->label.map_name +
    ".json";
  SetOutdoorFlag(label_filename, request->label.is_outdoor);
  response->success = true;
}

void LabelServer::SetOutdoorFlag(const std::string & filename, bool outdoor)
{
  std::lock_guard<std::mutex> locker(mutex_);
  rapidjson::Document doc(rapidjson::kObjectType);
  label_store_->SetOutdoor(outdoor, doc);
  label_store_->Write(filename, doc);
}

bool LabelServer::ReqeustVisionBuildingMapAvailable(bool & map_status, const std::string & map_name)
{
  if (!filesystem::exists(map_name)) {
    ERROR("Current map json file is not exist.");
    return false;
  }
  rapidjson::Document document(rapidjson::kObjectType);
  cyberdog::common::CyberdogJson::ReadJsonFromFile(map_name, document);
  bool outdoor = false;
  for (auto it = document.MemberBegin(); it != document.MemberEnd(); ++it) {
    std::string key = it->name.GetString();
    if (key == "is_outdoor") {
      outdoor = it->value.GetBool();
      INFO("Function ReqeustVisionBuildingMapAvailable() get is_outdoor: %d", outdoor);
      break;
    }
  }

  if (!outdoor) {
    map_status = true;
    return true;
  }

  if (map_result_client_ == nullptr) {
    map_result_client_ = std::make_shared<nav2_util::ServiceClient<MapAvailableResult>>(
      "get_miloc_status", shared_from_this());
  }

  // Client request
  bool connect = map_result_client_->wait_for_service(std::chrono::seconds(5));
  if (!connect) {
    ERROR("Waiting for miloc map handler the service. but cannot connect the service.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<MapAvailableResult::Request>();
  // request->map_id = 0;

  bool result = false;
  try {
    auto future_result = map_result_client_->invoke(request, std::chrono::seconds(10));
    if (future_result->code == 0 || future_result->code == 300) {
      map_status = true;
      result = true;
    } else if (future_result->code == 301 || future_result->code == 302) {
      map_status = false;
      result = false;
    }
    result = future_result->code == 0;
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
  }
  return result;
}


bool LabelServer::CheckDuplicateTags(const std::vector<protocol::msg::Label> & labels)
{
  std::unordered_multiset<std::string> tags;
  for (const auto & label : labels) {
    if (tags.count(label.label_name)) {
      ERROR("Same tag name: [%s]", label.label_name.c_str());
      return false;
    }
    tags.emplace(label.label_name);
  }
  return true;
}

bool LabelServer::GetOutdoorValue(const std::string & filename, bool & outdoor)
{
  std::vector<protocol::msg::Label> labels;
  label_store_->Read(filename, labels, outdoor);
  INFO("Read from file %s outdoor value : %d", filename.c_str(), outdoor);
  return true;
}

int LabelServer::CheckVisonMapStatus()
{
  if (map_result_client_ == nullptr) {
    map_result_client_ = std::make_shared<nav2_util::ServiceClient<MapAvailableResult>>(
      "get_miloc_status", shared_from_this());
  }

  // Client request
  bool connect = map_result_client_->wait_for_service(std::chrono::seconds(2));
  if (!connect) {
    ERROR("Waiting for miloc map handler the service timeout.");
    return false;
  }

  // Set request data
  auto request = std::make_shared<MapAvailableResult::Request>();
  request->map_id = 1;

  // success = 2 —— 正在构建地图
  // success = 3 —— 构建地图失败,请重新建图
  // success = 4 —— 查询地图失败，请重启机器狗
  int status = -1;
  try {
    //   0: 重定位地图可用
    // 300: 重定位地图不可用，正在构建中
    // 301: 重定位地图不可用，上次离线建图出错，需要重新建图
    // 302: 重定位地图不可用，需要重新扫图⁣
    auto future_result = map_result_client_->invoke(request, std::chrono::seconds(5));
    if (future_result->code == 0) {
      INFO("Relocation map is available");
      status = 0;
    } else if (future_result->code == 300) {
      ERROR("Current vision map is building.");
      status = 2;
    } else if (future_result->code == 301 || future_result->code == 302) {
      ERROR("Current vision map is unavailable, please remapping.");
      status = 3;
    }
  } catch (const std::exception & e) {
    ERROR("%s", e.what());
    status = 4;
  }
  return status;
}

}  // namespace CYBERDOG_NAV
