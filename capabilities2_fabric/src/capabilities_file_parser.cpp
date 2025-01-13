#include <capabilities2_fabric/capabilities_file_parser.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto parser_node = std::make_shared<CapabilitiesFileParser>();
  
  parser_node->initialize();  // Call initialize after construction

  rclcpp::spin(parser_node);
  rclcpp::shutdown();

  return 0;
}
