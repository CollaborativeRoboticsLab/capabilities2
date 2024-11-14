#pragma once

#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <capabilities2_runner/runner_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <cstdlib>
#include <memory>

namespace capabilities2_runner
{

/**
 * @brief launch runner base class
 *
 * Create a launch file runner to run a launch file based capability
 */
class LaunchRunner : public RunnerBase
{
public:
  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  LaunchRunner() : RunnerBase()
  {
  }

  /**
   * @brief Starter function for starting the launch runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config) override
  {
    // Start the launch file in a separate thread
    launch_thread_ = std::thread(&LaunchRunner::startLaunchFile, this);
  }

  /**
   * @brief stop function to cease functionality and shutdown
   *
   */
  virtual void stop() override
  {
    // if the node pointer is empty then throw an error
    // this means that the runner was not started and is being used out of order

    if (!node_)
      throw runner_exception("cannot stop runner that was not started");

    // Join the thread and stop the launch if the node is shutting down
    if (launch_thread_.joinable())
    {
      std::string command = "pkill -f 'ros2 launch " + get_package_name() + " " + run_config_.runner + "'";

      // Kill the launch process if still running
      std::system(command.c_str());
      launch_thread_.join();
    }
  }

  void startLaunchFile()
  {
    std::string command = "ros2 launch " + get_package_name() + " " + run_config_.runner;
    RCLCPP_INFO(node_->get_logger(), "Executing command: %s", command.c_str());

    // Run the command
    int result = std::system(command.c_str());

    if (result != 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to launch file with command: %s", command.c_str());
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Successfully launched file: %s", run_config_.runner.c_str());
    }
  }

  // throw on trigger function
  std::optional<std::function<void(tinyxml2::XMLElement*)>> trigger(tinyxml2::XMLElement* parameters) override
  {
    throw runner_exception("cannot trigger this is a no-trigger action runner");
  }

  std::thread launch_thread_;
};

}  // namespace capabilities2_runner