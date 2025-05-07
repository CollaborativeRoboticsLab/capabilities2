#include <capabilities2_fabric/capabilities_fabric_client.hpp>

int main(int argc, char* argv[])
{
  // Initialize the ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // Create a shared pointer to the CapabilitiesFabricClient
  auto parser_node = std::make_shared<CapabilitiesFabricClient>();
  
  // Initialize the node
  parser_node->initialize();  // Call initialize after construction

  // Spin the node to process callbacks
  rclcpp::spin(parser_node);

  rclcpp::shutdown();

  return 0;
}
