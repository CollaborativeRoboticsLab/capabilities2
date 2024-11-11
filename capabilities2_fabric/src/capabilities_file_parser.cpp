#include <capabilities2_executor/capabilities_file_parser.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CapabilitiesFileParser>());
  rclcpp::shutdown();
  return 0;
}
