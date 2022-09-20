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

#define GLOBAL_MAP_LOCATION "/home/mi/mapping/"

#include "map_label_server/labelserver_node.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "filesystem/filesystem.hpp"

namespace CYBERDOG_NAV
{
LabelServer::LabelServer()
: rclcpp::Node("LabelServer")
{
  PrintMapData();
  map_label_store_ptr_ = std::make_shared<cyberdog::navigation::LabelStore>();

  callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  set_label_server_ = this->create_service<protocol::srv::SetMapLabel>(
    "set_label",
    std::bind(
      &LabelServer::handle_set_label, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_default, callback_group_);

  get_label_server_ = this->create_service<protocol::srv::GetMapLabel>(
    "get_label",
    std::bind(
      &LabelServer::handle_get_label, this, std::placeholders::_1,
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

  INFO("map_name : %s", map_name.c_str());

  std::string map_filename = request->map_name + ".pgm";
  if (!map_label_store_ptr_->IsExist(map_filename)) {
    WARN("Map not exist.");
    response->success = protocol::srv::GetMapLabel_Response::RESULT_SUCCESS;
    return;
  }

  // Load map's yaml config
  nav_msgs::msg::OccupancyGrid map;
  bool ok = LoadMapMetaInfo(request->map_name , map);
  if (!ok) {
    WARN("Map yaml config file not exist.");
    response->success = protocol::srv::GetMapLabel_Response::RESULT_SUCCESS;
    return;
  }

  // Load map's labels
  std::string label_filename = request->map_name + ".json";
  std::vector<protocol::msg::Label> labels;
  map_label_store_ptr_->Read(map_label_store_ptr_->map_label_directory() + label_filename, labels);
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

  response->label.map_name = request->map_name;
  response->success = protocol::srv::GetMapLabel_Response::RESULT_SUCCESS;
}

void LabelServer::handle_set_label(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<protocol::srv::SetMapLabel::Request> request,
  std::shared_ptr<protocol::srv::SetMapLabel::Response> response)
{
  INFO("LabelServer::handle_set_label ");

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
    response->success = protocol::srv::SetMapLabel_Response::RESULT_SUCCESS;
    return;
  } 

  // remove label
  if (request->only_delete && request->label.labels.size()) {
    // TODO
    response->success = protocol::srv::SetMapLabel_Response::RESULT_SUCCESS;
    return;
  }

  std::string label_filename_suffix = request->label.map_name + ".json";
  std::string label_filename = map_label_store_ptr_->map_label_directory() + label_filename_suffix;

  if (!map_label_store_ptr_->IsExist(label_filename)) {
    bool exist = map_label_store_ptr_->CreateMapLabelFile(
    map_label_store_ptr_->map_label_directory(), label_filename_suffix);
    if (!exist) {
      WARN("Current map label json file has exist.");
    }
  }
  
  rapidjson::Document doc(rapidjson::kObjectType);
  map_label_store_ptr_->SetMapName(map_filename, map_filename, doc);
  for (size_t i = 0; i < request->label.labels.size(); i++) {
    // print
    INFO("label [%s] : [%f, %f]",
      request->label.labels[i].label_name.c_str(),
      request->label.labels[i].physic_x,
      request->label.labels[i].physic_y);

    // save label
    auto label = std::make_shared<protocol::msg::Label>();
    label->set__physic_x(request->label.labels[i].physic_x);
    label->set__physic_y(request->label.labels[i].physic_y);
    map_label_store_ptr_->AddLabel(label_filename, request->label.labels[i].label_name, label, doc);
  }

  // save
  map_label_store_ptr_->Write(label_filename, doc);
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
    for (int i = 0; i < map.data.size(); i++) {
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
  INFO("map.data size : %d", map.data.size());

  return true;
}

bool LabelServer::RemoveMap(const std::string & map_name)
{
  filesystem::remove_all(map_label_store_ptr_->map_label_directory());
  return true;
}

}  // namespace CYBERDOG_NAV
