#include <capabilities2_server/capabilities_server.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<capabilities2_server::CapabilitiesServer>());
  rclcpp::shutdown();
  return 0;
}
