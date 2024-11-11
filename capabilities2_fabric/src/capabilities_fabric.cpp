#include <capabilities2_executor/capabilities_fabric.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CapabilitiesFabric>());
  rclcpp::shutdown();
  return 0;
}
