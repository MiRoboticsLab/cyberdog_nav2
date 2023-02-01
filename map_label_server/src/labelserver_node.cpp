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

#define GLOBAL_MAP_LOCATION "/home/mi/mapping/"

#include "map_label_server/labelserver_node.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "filesystem/filesystem.hpp"
#include "cyberdog_common/cyberdog_json.hpp"

namespace CYBERDOG_NAV
{
LabelServer::LabelServer()
: rclcpp::Node("LabelServer")
{
  map_label_store_ptr_ = std::make_shared<cyberdog::navigation::LabelStore>();

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // map name handle
  map_server_ = this->create_service<std_srvs::srv::SetBool>(
    "map_name",
    std::bind(
      &LabelServer::HandleRequestUserSaveMapName, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_default, callback_group_);

  // set map label
  set_label_server_ = this->create_service<protocol::srv::SetMapLabel>(
    "set_label",
    std::bind(
      &LabelServer::handle_set_label, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_default, callback_group_);

  // get map and map's labels
  get_label_server_ = this->create_service<protocol::srv::GetMapLabel>(
    "get_label",
    std::bind(
      &LabelServer::handle_get_label, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_default, callback_group_);

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // lidar mapping flag
  // vision_mapping_sub_ = this->create_subscription<std_msgs::msg::Bool>(
  //   "vision_mapping_alive ",
  //   rclcpp::SystemDefaultsQoS(),
  //   std::bind(
  //     &LabelServer::HandleLidarIsMappingMessages, this,
  //     std::placeholders::_1));

  vision_outdoor_server_ = this->create_service<std_srvs::srv::SetBool>(
    "vision_outdoor",
    std::bind(
      &LabelServer::HandleVisionOutdoor, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_default, callback_group_);

  // // vision mapping flag
  // lidar_mapping_sub_ = this->create_subscription<std_msgs::msg::Bool>(
  //   "vision_mapping_alive",
  //   rclcpp::SystemDefaultsQoS(),
  //   std::bind(
  //     &LabelServer::HandleVisionIsMappingMessages, this,
  //     std::placeholders::_1));

  laser_outdoor_server_ = this->create_service<std_srvs::srv::SetBool>(
    "lidar_outdoor",
    std::bind(
      &LabelServer::HandleLaserOutdoor, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_default, callback_group_);
}

LabelServer::~LabelServer() {}

void LabelServer::handle_get_label(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<protocol::srv::GetMapLabel::Request> request,
  std::shared_ptr<protocol::srv::GetMapLabel::Response> response)
{
  std::unique_lock<std::mutex> ulk(mut);
  std::string map_name = GLOBAL_MAP_LOCATION + request->map_name;

  INFO("[ Get label server request: ------------------------------------- ]");
  INFO("map_name : %s", request->map_name.c_str());
  // bool map_status = false;
  // bool ready = ReqeustVisionBuildingMapAvailable(map_status, request->map_name);
  // if (!ready && !map_status) {
  //   WARN("Current map not available.");
  //   return;
  // }

  std::string map_filename = request->map_name + ".pgm";
  if (!map_label_store_ptr_->IsExist(map_filename)) {
    WARN("Map not exist.");

    // clear map data
    response->label.map.data.clear();
    response->label.map.info.resolution = 0.0f;
    response->label.map.info.width = 0;
    response->label.map.info.height = 0;
    response->success = protocol::srv::GetMapLabel_Response::RESULT_SUCCESS;
    return;
  }

  // Load map's yaml config
  nav_msgs::msg::OccupancyGrid map;
  bool ok = LoadMapMetaInfo(request->map_name, map);
  if (!ok) {
    WARN("Map yaml config file not exist.");
    response->success = protocol::srv::GetMapLabel_Response::RESULT_SUCCESS;
    return;
  }

  // Load map's labels
  std::string label_filename = request->map_name + ".json";
  std::vector<protocol::msg::Label> labels;
  bool is_outdoor = false;
  // map_label_store_ptr_->Read(map_label_store_ptr_->map_label_directory()
  // + label_filename, labels);
  map_label_store_ptr_->Read(
    map_label_store_ptr_->map_label_directory() + label_filename, labels, is_outdoor);

  if (!labels.empty()) {
    for (auto label : labels) {
      response->label.labels.push_back(label);
    }
  }

  // Set response result.
  response->label.map.info.resolution = map.info.resolution;
  response->label.map.info.width = map.info.width;
  response->label.map.info.height = map.info.height;
  response->label.map.info.origin = map.info.origin;
  response->label.map.data = map.data;

  // is_outdoor
  response->label.is_outdoor = is_outdoor;
  response->label.map_name = request->map_name;
  response->success = protocol::srv::GetMapLabel_Response::RESULT_SUCCESS;
  INFO("Current building  map is outdoor : %d", is_outdoor);

  // publish map
  occ_pub_->publish(map);
}

void LabelServer::handle_set_label(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<protocol::srv::SetMapLabel::Request> request,
  std::shared_ptr<protocol::srv::SetMapLabel::Response> response)
{
  INFO("[ Set label server request: ------------------------------------- ]");
  std::string map_filename = request->label.map_name + ".pgm";
  INFO("map name: %s", map_filename.c_str());
  if (!map_label_store_ptr_->IsExist(map_filename)) {
    INFO("Map not exist, not set label function.");
    response->success = protocol::srv::GetMapLabel_Response::RESULT_FAILED;
    return;
  }

  // remove map and label tag
  if (request->only_delete && request->label.labels.size() == 0) {
    INFO("Remove map : %s", request->label.map_name.c_str());
    RemoveMap(map_filename);
    ResetFlags();
    response->success = protocol::srv::SetMapLabel_Response::RESULT_SUCCESS;
    return;
  }

  // remove label
  if (request->only_delete && request->label.labels.size()) {
    std::string label_filename_suffix = request->label.map_name + ".json";
    std::string label_filename = map_label_store_ptr_->map_label_directory() +
      label_filename_suffix;

    for (auto label : request->label.labels) {
      map_label_store_ptr_->RemoveLabel(label_filename, label.label_name.c_str());
    }
    ResetFlags();
    response->success = protocol::srv::SetMapLabel_Response::RESULT_SUCCESS;
    return;
  }

  // Check multi tag, if exit multi label tag return failure
  bool legality = CheckDuplicateTags(request->label.labels);
  if (!legality) {
    response->success = protocol::srv::SetMapLabel_Response::RESULT_FAILED;
    ERROR("User user entered more than one of the same tags");
    return;
  }

  std::string label_filename_suffix = request->label.map_name + ".json";
  if (!map_label_store_ptr_->IsExist(label_filename_suffix)) {
    ERROR("map label json file(%s) has exist", label_filename_suffix.c_str());
    response->success = protocol::srv::SetMapLabel_Response::RESULT_FAILED;
    return;
  }

  rapidjson::Document doc(rapidjson::kObjectType);
  map_label_store_ptr_->SetMapName(map_filename, map_filename, doc);

  // outdoor
  INFO("is_outdoor flag : %d", request->label.is_outdoor);
  bool outdoor = false;
  GetOutdoorValue(label_filename_suffix, outdoor);
  map_label_store_ptr_->SetOutdoor(outdoor, doc);

  for (size_t i = 0; i < request->label.labels.size(); i++) {
    // print
    INFO(
      "label [%s] : [%f, %f]",
      request->label.labels[i].label_name.c_str(),
      request->label.labels[i].physic_x,
      request->label.labels[i].physic_y);

    // save label
    auto label = std::make_shared<protocol::msg::Label>();
    label->set__physic_x(request->label.labels[i].physic_x);
    label->set__physic_y(request->label.labels[i].physic_y);
    map_label_store_ptr_->AddLabel(label_filename_suffix, request->label.labels[i].label_name, label, doc);
  }

  // save
  map_label_store_ptr_->Write(label_filename_suffix, doc);
  response->success = protocol::srv::SetMapLabel_Response::RESULT_SUCCESS;
}

void LabelServer::read_map_label(std::string filename, LABEL & label)
{
  FILE * infile;
  infile = fopen(filename.c_str(), "r");
  if (infile == NULL) {
    RCLCPP_ERROR(get_logger(), "Error opened file");
    return;
  }
  fread(&label, sizeof(LABEL), 1, infile);
  fclose(infile);
}

void LabelServer::writ_map_label(std::string filename, const LABEL & label)
{
  FILE * outfile;
  outfile = fopen(filename.c_str(), "w");
  if (outfile == NULL) {
    RCLCPP_ERROR(get_logger(), "Error opened file");
    return;
  }
  fwrite(&label, sizeof(LABEL), 1, outfile);
  fclose(outfile);
}

bool LabelServer::makeMapFolder(std::string filename)
{
  if (mkdir(filename.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)) {
    return true;
  }
  return false;
}

bool LabelServer::isFolderExist(std::string path)
{
  struct stat buffer;
  return stat(path.c_str(), &buffer) == 0 && buffer.st_mode & S_IFDIR;
}

bool LabelServer::isFileExixt(std::string path)
{
  struct stat buffer;
  return stat(path.c_str(), &buffer) == 0 && buffer.st_mode & S_IFREG;
}

bool LabelServer::removeFile(std::string path)
{
  return remove(path.c_str()) == 0;
}

void LabelServer::PrintMapData()
{
  nav_msgs::msg::OccupancyGrid map;
  std::string yaml_map = "/home/quan/Downloads/mapping/map.yaml";
  auto status = nav2_map_server::loadMapFromYaml(yaml_map, map);
  if (nav2_map_server::LOAD_MAP_STATUS::LOAD_MAP_SUCCESS == status) {
    INFO("Get yaml map success.");
    // response->label.resolution = map.info.resolution;
    // response->label.width = map.info.width;
    // response->label.height = map.info.height;
    // response->label.origin = map.info.origin;
    // response->label.data = map.data;


    // for (int i = 0; i < 25; i++) {
    //   response->label.map.data.push_back(i % 100);
    // }

    std::cout << "resolution: " << map.info.resolution << std::endl;
    std::cout << "width: " << map.info.width << std::endl;
    std::cout << "height: " << map.info.height << std::endl;

    std::cout << "\n\n";
    std::cout << "[" << std::endl;
    for (long unsigned int i = 0; i < map.data.size(); i++) {
      printf("%d, ", map.data[i]);
    }
    std::cout << "]" << std::endl;
  }
}

bool LabelServer::LoadMapMetaInfo(const std::string & map_name, nav_msgs::msg::OccupancyGrid & map)
{
  std::string map_yaml_config = "/home/mi/mapping/" + map_name + ".yaml";
  auto status = nav2_map_server::loadMapFromYaml(map_yaml_config, map);

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

bool LabelServer::RemoveMap(const std::string & map_name)
{
  (void)map_name;
  filesystem::remove_all(map_label_store_ptr_->map_label_directory());
  return true;
}

void LabelServer::HandleRequestUserSaveMapName(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  INFO("request save map's name.");

  if (request->data) {
    response->message = robot_map_name();
    response->success = true;
    return;
  }

  response->message = "";
  response->success = false;
}

void LabelServer::set_robot_map_name(const std::string & name)
{
  robot_map_name_ = name;
}

std::string LabelServer::robot_map_name() const
{
  return robot_map_name_;
}

// void LabelServer::HandleVisionIsMappingMessages(const std_msgs::msg::Bool::SharedPtr msg)
// {
//   INFO("Current building map is vision.");
//   if (msg == nullptr) {
//     return;
//   }

//   if (msg->data) {
//     use_vision_create_map_ = msg->data;
//     SetOutdoorFlag(use_vision_create_map_);
//   }
// }

// void LabelServer::HandleLidarIsMappingMessages(const std_msgs::msg::Bool::SharedPtr msg)
// {
//   INFO("Current building map is lidar.");
//   if (msg == nullptr) {
//     return;
//   }

//   if (msg->data) {
//     use_lidar_create_map_ = msg->data;
//     SetOutdoorFlag(false);
//   }
// }

void LabelServer::HandleVisionOutdoor(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  INFO("Handle vision outdoor request client");

  if (!request->data) {
    response->success = false;
    return;
  }

  use_vision_create_map_ = true;
  SetOutdoorFlag(true);
  response->success = true;
}

void LabelServer::HandleLaserOutdoor(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  INFO("Handle lidar outdoor request client");

  if (!request->data) {
    response->success = false;
    return;
  }

  use_lidar_create_map_ = true;
  SetOutdoorFlag(false);
  response->success = true;
}

void LabelServer::SetOutdoorFlag(bool outdoor)
{
  std::lock_guard<std::mutex> locker(mutex_);
  const std::string map_name = "map";
  std::string label_filename = map_label_store_ptr_->map_label_directory() + "map.json";
  rapidjson::Document doc(rapidjson::kObjectType);
  map_label_store_ptr_->SetOutdoor(outdoor, doc);
  map_label_store_ptr_->Write(label_filename, doc);
  INFO("Label server set outdoor : %d", outdoor);
}

bool LabelServer::ReqeustVisionBuildingMapAvailable(bool & map_status, const std::string & map_name)
{
  std::string map_json_filename = "/home/mi/mapping/" + map_name + ".json";
  if (!filesystem::exists(map_json_filename)) {
    ERROR("Current map json file is not exist.");
    return false;
  }

  rapidjson::Document document(rapidjson::kObjectType);
  cyberdog::common::CyberdogJson::ReadJsonFromFile(map_json_filename, document);

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
      INFO("[%s : %lf, %lf]", label.label_name.c_str(), label.physic_x, label.physic_y);
      return false;
    }
    tags.emplace(label.label_name);
  }
  return true;
}

void LabelServer::ResetFlags()
{
  use_lidar_create_map_ = false;
  use_lidar_create_map_ = false;
}

bool LabelServer::GetOutdoorValue(const std::string & filename, bool & outdoor)
{
  std::vector<protocol::msg::Label> labels;
  map_label_store_ptr_->Read(
    map_label_store_ptr_->map_label_directory() + filename, labels, outdoor);
  
  INFO("Read from file %s outdoor value : %d", filename.c_str(), outdoor);
  return true;
}

}  // namespace CYBERDOG_NAV
