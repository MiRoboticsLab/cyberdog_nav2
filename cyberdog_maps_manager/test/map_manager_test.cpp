
// Copyright (c) 2021 Xiaomi Corporation
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

#include <gtest/gtest.h>

#include <vector>
#include <memory>
#include <string>
#include <unordered_map>


#include <rclcpp/rclcpp.hpp>  // NOLINT
#include "std_msgs/msg/int32.hpp"

#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "cyberdog_maps_manager/maps_manager.hpp"

using std::placeholders::_1;

namespace cyberdog
{
namespace maps_manager
{

void Test_Request2Json()
{
  INFO("#################### [case 1] ##########################");
  MapsManager::CommandRequest request;
  std::vector<std::string> maps_name {
    "map1"
  };

  std::vector<uint64_t> ids {
    10001
  };

  request.maps_name = maps_name;
  // request.ids = ids;
  request.name_code = 1001;
  request.timestamp = 12345;

  auto manager = std::make_shared<MapsManager>();
  auto result = manager->ToString(request);
  INFO("json : %s", result.c_str());
}

void Test_RequestArray2Json()
{

  INFO("#################### [case 2 array] ##########################");
  MapsManager::CommandRequest request;
  std::vector<std::string> maps_name {
    "map1",
    "map2",
    "map3",
    "map4",
    "map5",
  };

  std::vector<uint64_t> ids {
    10001,
    20002,
    30003,
    40004,
    50005
  };

  request.maps_name = maps_name;
  request.ids = ids;
  request.name_code = 1001;
  request.timestamp = 12345;

  auto manager = std::make_shared<MapsManager>();
  auto result = manager->ToString(request);
  INFO("json : %s", result.c_str());
}


void Run()
{
  Test_Request2Json();
  Test_RequestArray2Json();
}

class MapManagerTester : public rclcpp::Node
{
public:
  MapManagerTester()
  : Node("MapManagerTest")
  {
    manager_ = std::make_shared<MapsManager>();
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "maps_manager_test", 10, std::bind(&MapManagerTester::HandleCommandTestCallback, this, _1));
  }

private:
  // Debug and test
  void HandleCommandTestCallback(std_msgs::msg::Int32::ConstSharedPtr msg)
  {
    INFO("Handle command test...");
    if (msg->data == 1) {
      TestSave(1);
    } else if (msg->data == 2) {
      TestSave(2);
    } else if (msg->data == 3) {
      TestDelete(3);
    } else if (msg->data == 4) {
      TestDelete(4);
    } else if (msg->data == 5) {
      TestUpdate(5);
    } else if (msg->data == 6) {
      TestUpdate(6);
    } else if (msg->data == 7) {
      TestQuery(7);
    } else if (msg->data == 8) {
      TestQuery(8);
    }
  }

  // type == 1 lidar
  // type == 2 vision
  void TestSave(int type)
  {
    MapsManager::CommandRequest request;
    std::vector<std::string> maps_name {
      "map1"
    };

    // std::vector<uint64_t> ids {
    //   10001
    // };

    request.maps_name = maps_name;
    // request.ids = ids;
    request.timestamp = 12345;

    bool send_success = false;
    if (type == 1) {
      request.name_code = 1001;
      send_success = manager_->Save(MapsManager::MapType::Lidar, request);
    } else if (type == 2) {
      request.name_code = 1002;
      send_success = manager_->Save(MapsManager::MapType::Vision, request);
    }

    if (!send_success) {
      ERROR("Send command error");
    }
  }

  // type == 3 lidar
  // type == 4 vision
  void TestDelete(int type)
  {
    MapsManager::CommandRequest request;
    std::vector<std::string> maps_name {
      "map3",
      "map4"
    };

    std::vector<uint64_t> ids {
      3,
      4
    };

    request.maps_name = maps_name;
    request.ids = ids;
    request.timestamp = 12345;

    bool send_success = false;
    if (type == 3) {
      request.name_code = 1003;
      send_success = manager_->Delete(MapsManager::MapType::Lidar, request);
    } else if (type == 4) {
      request.name_code = 1004;
      send_success = manager_->Delete(MapsManager::MapType::Vision, request);
    }

    if (!send_success) {
      ERROR("Send command error");
    }
  }

  // type == 5 lidar
  // type == 6 vision
  void TestUpdate(int type)
  {
    MapsManager::CommandRequest request;
    std::vector<std::string> maps_name {
      "map_5",
      "map_6"
    };

    std::vector<uint64_t> ids {
      1,
      2
    };

    request.maps_name = maps_name;
    request.ids = ids;
    request.timestamp = 12345;

    bool send_success = false;
    if (type == 5) {
      request.name_code = 1005;
      send_success = manager_->Update(MapsManager::MapType::Lidar, request);
    } else if (type == 6) {
      request.name_code = 1006;
      send_success = manager_->Update(MapsManager::MapType::Vision, request);
    }

    if (!send_success) {
      ERROR("Send command error");
    }
  }

  // type == 7 lidar
  // type == 8 vision
  void TestQuery(int type)
  {
    INFO("--> TestQuery: ");

    std::vector<MapsManager::MapInfo> maps;
    MapsManager::CommandRequest request;
    request.timestamp = 12345;

    bool send_success = false;
    if (type == 7) {
      request.name_code = 1007;
      send_success = manager_->Query(MapsManager::MapType::Lidar, request);
    } else if (type == 8) {
      request.name_code = 1008;

      INFO("--> TestQuery: ###########");
      send_success = manager_->Query(MapsManager::MapType::Vision, request, maps);
    }

    if (!send_success) {
      ERROR("Send command error");
    }
  }


  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  std::shared_ptr<MapsManager> manager_ {nullptr};
};


}  // namespace maps_manager
}  // namespace cyberdog

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("MapManagerTest");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cyberdog::maps_manager::MapManagerTester>());
  rclcpp::shutdown();
  return 0;
}
