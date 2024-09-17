#pragma once

#include <filesystem>
#include <tinyxml2.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <std_msgs/msg/string.hpp>
#include <capabilities2_msgs/action/launch.hpp>
#include <capabilities2_runner/action_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief launch runner base class
 *
 * Create a launch file runner to run a launch file based capability
 */
class LaunchRunner : public ActionRunner<capabilities2_msgs::action::Launch>
{
public:
  LaunchRunner() : ActionRunner()
  {
  }

  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr,
							       std::function<void(const std::string&)> on_stopped = nullptr) override
  {
    // store node pointer and run_config
    init_action(node, run_config, "capabilities_launch_proxy/launch", on_started, on_terminated, on_stopped);

    // get the package path from environment variable
    std::string package_path;
    try
    {
      package_path = ament_index_cpp::get_package_share_directory(get_package_name());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get package share directory: %s", e.what());
      throw runner_exception("failed to get package share directory");
    }

    // resolve launch path
    // get full path to launch file
    // join package path with package name using path functions
    std::string launch_file_path = std::filesystem::path(package_path).append(run_config_.runner).string();

    // the launch file path
    RCLCPP_DEBUG(node_->get_logger(), "launch file path: %s", launch_file_path.c_str());

    // create a launch goal
    capabilities2_msgs::action::Launch::Goal goal;
    goal.launch_file_path = launch_file_path;

    // launch runner using action client
    action_client_->async_send_goal(goal, send_goal_options_);
  }

  virtual void trigger(std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr) override
  {
    // do nothing
  }

private:
  /** launch file path */
  std::string launch_file_path;
};

}  // namespace capabilities2_runner
