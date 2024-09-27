#include <capabilities2_executor/capabilities_executor.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CapabilitiesExecutor>());
  rclcpp::shutdown();
  return 0;
}
