#include <capabilities2_server/capabilities_server.hpp>

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create the node object and executor object
  auto node = std::make_shared<capabilities2_server::CapabilitiesServer>();
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // Add the node to the executor
  exec->add_node(node);

  // Spin the executor
  exec->spin();

  // Shutdown ROS 2
  rclcpp::shutdown();
  
  return 0;
}