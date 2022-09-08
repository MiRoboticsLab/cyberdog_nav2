#ifndef _LABELSERVER_NODE_
#define _LABELSERVER_NODE_

#include "protocol/srv/get_map_label.hpp"
#include "protocol/srv/set_map_label.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav2_map_server/map_server.hpp"
#include "nav2_map_server/map_io.hpp"
#include "map_label_server/label_store.hpp"
namespace CYBERDOG_NAV {
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
  void read_map_label(std::string filename, LABEL& label);

  void writ_map_label(std::string filename, const LABEL& label);

  bool makeMapFolder(std::string filename);

  bool isFolderExist(std::string path);

  bool isFileExixt(std::string path);

  bool removeFile(std::string path);

  void writeLabel(std::string path, LABEL label);

  void PrintMapData();

  std::shared_ptr<cyberdog::navigation::LabelStore> map_label_store_ptr_ {nullptr};
};
}  // namespace CYBERDOG_NAV
#endif  // _LABELSERVER_NODE_

