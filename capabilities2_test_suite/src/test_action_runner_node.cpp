

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create the node object and executor object
  auto server_node = std::make_shared<Capabilities2::TestActionServer>();
  auto client_node = std::make_shared<Capabilities2::Capabilities2Client>();

  // Initialize the node
  client_node->initialize();

  // Create a MultiThreadedExecutor
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // Add the node to the executor
  exec->add_node(server_node);
  
  // Spin the executor
  exec->spin();

  // Shutdown ROS 2
  rclcpp::shutdown();
  
  return 0;
}