#include <capabilities2_fabric/capabilities_fabric_client.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto parser_node = std::make_shared<CapabilitiesFabricClient>();
  
  parser_node->initialize();  // Call initialize after construction

  rclcpp::spin(parser_node);
  rclcpp::shutdown();

  return 0;
}
