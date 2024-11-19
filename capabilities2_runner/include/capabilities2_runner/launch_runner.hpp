#pragma once

#include <filesystem>
#include <capabilities2_runner/runner_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <cstring>
#include <sys/wait.h>

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
    init_base(node, run_config);

    package_name = run_config_.runner.substr(0, run_config_.runner.find("/"));
    launch_name = run_config_.runner.substr(run_config_.runner.find("/") + 1);

    std::string command = "source install/setup.bash && ros2 launch " + package_name + " " + launch_name;

    int childPid = fork();

    if (childPid == 0)
    {
      // Child process start
      execlp("/bin/bash", "/bin/bash", "-c", command.c_str(), NULL);
      perror("execlp failed to execute ROS 2 launch file");  // If execlp fails
      exit(EXIT_FAILURE);
      // Child process end
    }

    if (childPid == -1)
    {
      std::string error_msg = "Failed to start " + launch_name + " from " + package_name;
      RCLCPP_ERROR(node_->get_logger(), error_msg.c_str());
      throw std::runtime_error(error_msg);
    }
    else
    {
      std::string info_msg =
          "Started " + launch_name + " from " + package_name + " with PID : " + std::to_string(childPid);
      RCLCPP_INFO(node_->get_logger(), info_msg.c_str());
    }
  }

  /**
   * @brief stop function to cease functionality and shutdown
   *
   */
  virtual void stop() override
  {
    if (childPid != -1)
    {
      kill(childPid, SIGTERM);     // Send termination signal to child
      waitpid(childPid, NULL, 0);  // Wait for child to terminate
      RCLCPP_INFO(node_->get_logger(), "%s launch file from %s stopped : %d", launch_name.c_str(),
                  package_name.c_str());
      childPid = -1;

      if (kill(childPid, SIGTERM) == 0)
      {
        if (waitpid(childPid, NULL, 0) != childPid)
        {
          RCLCPP_WARN(node_->get_logger(), "%s in process %d did not terminate; sending SIGKILL", launch_name.c_str(), childPid);
          kill(childPid, SIGKILL);
        }
      }
    }
  }

  // throw on trigger function
  std::optional<std::function<void(tinyxml2::XMLElement*)>> trigger(tinyxml2::XMLElement* parameters) override
  {
    throw runner_exception("No Trigger as this is launch runner");
  }

  pid_t childPid = -1;
  std::string launch_name;
  std::string package_name;
};

}  // namespace capabilities2_runner