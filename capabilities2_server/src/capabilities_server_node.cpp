#include <capabilities2_server/capabilities_server.hpp>
#include <atomic>
#include <chrono>
#include <csignal>

namespace
{
static_assert(std::atomic<bool>::is_always_lock_free);
std::atomic<bool> shutdown_requested{false};

void requestShutdown(int)
{
  shutdown_requested.store(true, std::memory_order_relaxed);
}
}

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestShutdown);
  std::signal(SIGTERM, requestShutdown);

  // Create the node object and executor object
  auto node = std::make_shared<capabilities2_server::CapabilitiesServer>();

  // Initialize the node
  node->initialize();

  // Create a MultiThreadedExecutor
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // Add the node to the executor
  exec->add_node(node);

  auto shutdown_timer = node->create_wall_timer(std::chrono::milliseconds(100), [&exec]() {
    if (shutdown_requested.load(std::memory_order_relaxed))
    {
      exec->cancel();
    }
  });

  // Spin the executor
  exec->spin();

  shutdown_timer->cancel();
  shutdown_timer.reset();
  exec->remove_node(node);
  node->shutdown_bonds();
  node.reset();

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}