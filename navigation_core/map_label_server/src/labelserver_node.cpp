#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

//#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#define GLOBAL_MAP_LOCATION "/home/quan/Downloads/mapping/"

#include "map_label_server/labelserver_node.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace CYBERDOG_NAV {
LabelServer::LabelServer() : rclcpp::Node("LabelServer") {

  map_label_store_ptr_ = std::make_shared<cyberdog::navigation::LabelStore>();
  
  callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  set_label_server_ = this->create_service<protocol::srv::SetMapLabel>(
      "set_label",
      std::bind(&LabelServer::handle_set_label, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3),
      rmw_qos_profile_default, callback_group_);

  get_label_server_ = this->create_service<protocol::srv::GetMapLabel>(
      "get_label",
      std::bind(&LabelServer::handle_get_label, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3),
      rmw_qos_profile_default, callback_group_);
}
LabelServer::~LabelServer() {}
void LabelServer::handle_get_label(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<protocol::srv::GetMapLabel::Request> request,
    std::shared_ptr<protocol::srv::GetMapLabel::Response> response) {
  std::unique_lock<std::mutex> ulk(mut);
  std::string map_name = GLOBAL_MAP_LOCATION + request->map_name;

  INFO("map_name : %s", map_name.c_str());
  response->label.map_name = request->map_name;

  if (isFolderExist(map_name)) {
    RCLCPP_ERROR(get_logger(), "found map %s", map_name.c_str());
    // response->success = protocol::srv::GetMapLabel_Response::RESULT_FAILED;

    DIR* dirp = opendir(map_name.c_str());
    struct dirent* dp;
    while ((dp = readdir(dirp)) != NULL) {
      LabelT label;
      protocol::msg::Label l;

      RCLCPP_ERROR(get_logger(), "file: %s", dp->d_name);
      if (!strncmp(dp->d_name, ".", 1) || !strncmp(dp->d_name, "..", 2)) {
        continue;
      }
      read_map_label(map_name + "/" + dp->d_name, label);
      l.label_name = dp->d_name;
      l.physic_x = label.x;
      l.physic_y = label.y;
      response->label.labels.push_back(l);
    }
  }


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


    response->label.map.info.resolution = map.info.resolution;
    response->label.map.info.width = map.info.width;
    response->label.map.info.height = map.info.height;
    response->label.map.info.origin = map.info.origin;
    response->label.map.data = map.data;

    // for (int i = 0; i < 25; i++) {
    //   response->label.map.data.push_back(i % 100);
    // }

    INFO("resolution : %f", response->label.map.info.resolution);
    INFO("width : %d", response->label.map.info.width);
    INFO("height : %d", response->label.map.info.height);
    INFO("map.data size : %d", response->label.map.data.size());
  }

  // closedir(dirp);
  response->success = protocol::srv::GetMapLabel_Response::RESULT_SUCCESS;
}

void LabelServer::handle_set_label(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<protocol::srv::SetMapLabel::Request> request,
    std::shared_ptr<protocol::srv::SetMapLabel::Response> response) {

  INFO("LabelServer::handle_set_label ");

  std::unique_lock<std::mutex> ulk(mut);
  std::string map_path =
      std::string(GLOBAL_MAP_LOCATION) + request->label.map_name;
  if (!isFolderExist(map_path)) {
    makeMapFolder(map_path);
  }
  // remove exist label
  for (size_t i = 0; i < request->label.labels.size(); i++) {
    RCLCPP_ERROR(get_logger(), "map_path: %s, label_name:%s", map_path.c_str(),
                 request->label.labels[i].label_name.c_str());
    std::string label_name =
        map_path + "/" + request->label.labels[i].label_name;
    if (isFileExixt(label_name)) {
      removeFile(label_name);
    }

    if (!request->only_delete) {
      RCLCPP_ERROR(get_logger(), "new label:%s", label_name.c_str());
      LABEL label;
      label.x = request->label.labels[i].physic_x;
      label.y = request->label.labels[i].physic_y;
      writ_map_label(label_name, label);
    }
  }
  response->success = protocol::srv::SetMapLabel_Response::RESULT_SUCCESS;
}
void LabelServer::read_map_label(std::string filename, LABEL& label) {
  FILE* infile;
  infile = fopen(filename.c_str(), "r");
  if (infile == NULL) {
    RCLCPP_ERROR(get_logger(), "Error opened file");
    return;
  }
  fread(&label, sizeof(LABEL), 1, infile);
  fclose(infile);
}

void LabelServer::writ_map_label(std::string filename, const LABEL& label) {
  FILE* outfile;
  outfile = fopen(filename.c_str(), "w");
  if (outfile == NULL) {
    RCLCPP_ERROR(get_logger(), "Error opened file");
    return;
  }
  fwrite(&label, sizeof(LABEL), 1, outfile);
  fclose(outfile);
}

bool LabelServer::makeMapFolder(std::string filename) {
  if (mkdir(filename.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)) {
    return true;
  }
  return false;
}

bool LabelServer::isFolderExist(std::string path) {
  struct stat buffer;
  return (stat(path.c_str(), &buffer) == 0 && buffer.st_mode & S_IFDIR);
}

bool LabelServer::isFileExixt(std::string path) {
  struct stat buffer;
  return (stat(path.c_str(), &buffer) == 0 && buffer.st_mode & S_IFREG);
}

bool LabelServer::removeFile(std::string path) {
  return (remove(path.c_str()) == 0);
}

}  // namespace CYBERDOG_NAV