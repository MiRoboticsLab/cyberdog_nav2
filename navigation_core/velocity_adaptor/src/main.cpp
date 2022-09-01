#include "velocity_adaptor/velocity_adaptor.hpp"

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cyberdog::navigation::VelocityAdaptor>());
  rclcpp::shutdown();
  return 0;
}
