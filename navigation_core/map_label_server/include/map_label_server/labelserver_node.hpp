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

#ifndef _LABELSERVER_NODE_
#define _LABELSERVER_NODE_

#include "protocol/srv/get_map_label.hpp"
#include "protocol/srv/set_map_label.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav2_map_server/map_server.hpp"
#include "nav2_map_server/map_io.hpp"
#include "map_label_server/label_store.hpp"
namespace CYBERDOG_NAV
{
  
typedef struct LABEL {
  float x;
  float y;
} LabelT;

class LabelServer : public rclcpp::Node {
 public:
  LabelServer();
  ~LabelServer();

 private:
  std::mutex mut;
  rclcpp::Service<protocol::srv::SetMapLabel>::SharedPtr set_label_server_;
  rclcpp::Service<protocol::srv::GetMapLabel>::SharedPtr get_label_server_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  void handle_set_label(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<protocol::srv::SetMapLabel::Request> request,
      std::shared_ptr<protocol::srv::SetMapLabel::Response> response);

  void handle_get_label(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<protocol::srv::GetMapLabel::Request> request,
      std::shared_ptr<protocol::srv::GetMapLabel::Response> response);

//the func is called in the func "handle_get_label"
  //void read_map_label(std::string filename, LABEL& label);

//the func is called in both "handle_get_label" and "handle_set_label"
  //bool isFolderExist(std::string path);

//the func is called in the func "handle_set_label"
  //void write_map_label(std::string filename, const LABEL& label);

//the func is called in the func "handle_set_label"
  //bool makeMapFolder(std::string filename);

//the func is called in the func "handle_set_label"
  //bool isFileExist(std::string path);

//the func is called in the func "handle_set_label"
  //bool removeFile(std::string path);

  //void writeLabel(std::string path, LABEL label);

  std::shared_ptr<cyberdog::navigation::LabelStore> map_label_store_ptr_ {nullptr};
};
}  // namespace CYBERDOG_NAV
#endif  // _LABELSERVER_NODE_

