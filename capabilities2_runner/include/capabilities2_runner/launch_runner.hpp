#pragma once

#include <capabilities2_runner/runner_base.hpp>
#include <capabilities2_msgs/srv/launch_start.hpp>
#include <capabilities2_msgs/srv/launch_stop.hpp>

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
  using LaunchStart = capabilities2_msgs::srv::LaunchStart;
  using LaunchStop = capabilities2_msgs::srv::LaunchStop;

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

    // create an service client
    start_service_client_ = node_->create_client<LaunchStart>("/capabilities/launch_start");

    RCLCPP_INFO(node_->get_logger(), "%s waiting for service: /capabilities/launch_start",
                run_config_.interface.c_str());

    if (!start_service_client_->wait_for_service(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(node_->get_logger(), "%s failed to connect to service: /capabilities/launch_start",
                   run_config_.interface.c_str());
      throw runner_exception("Failed to connect to server: /capabilities/launch_start");
    }

    RCLCPP_INFO(node_->get_logger(), "%s connected to service: /capabilities/launch_start",
                run_config_.interface.c_str());

    // create an service client
    stop_service_client_ = node_->create_client<LaunchStop>("/capabilities/launch_stop");

    // wait for action server
    RCLCPP_INFO(node_->get_logger(), "%s waiting for service: /capabilities/launch_stop",
                run_config_.interface.c_str());

    if (!stop_service_client_->wait_for_service(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(node_->get_logger(), "%s failed to connect to service: /capabilities/launch_stop",
                   run_config_.interface.c_str());
      throw runner_exception("Failed to connect to server: /capabilities/launch_stop");
    }

    RCLCPP_INFO(node_->get_logger(), "%s connected to service: /capabilities/launch_stop",
                run_config_.interface.c_str());

    // generate a reequest from launch_name and package_name
    auto request_msg = std::make_shared<LaunchStart::Request>();

    request_msg->package_name = package_name;
    request_msg->launch_file_name = launch_name;

    RCLCPP_INFO(node_->get_logger(), "Requesting to launch %s from %s", launch_name.c_str(), package_name.c_str());

    auto result_future = start_service_client_->async_send_request(
        request_msg, [this](typename rclcpp::Client<LaunchStart>::SharedFuture future) {
          if (!future.valid())
          {
            RCLCPP_ERROR(node_->get_logger(), "Request to launch %s from %s failed", launch_name.c_str(), package_name.c_str());
            return;
          }

          RCLCPP_INFO(node_->get_logger(), "Request to launch %s from %s succeeded", launch_name.c_str(), package_name.c_str());

          auto response = future.get();

          childPid = response->pid;
        });
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

    // generate a reequest from launch_name and package_name
    auto request_msg = std::make_shared<LaunchStop::Request>();

    request_msg->pid = childPid;

    RCLCPP_INFO(node_->get_logger(), "Requesting to stop %s with PID %d", launch_name.c_str(), childPid);

    auto result_future = stop_service_client_->async_send_request(
        request_msg, [this](typename rclcpp::Client<LaunchStop>::SharedFuture future) {
          if (!future.valid())
          {
            RCLCPP_ERROR(node_->get_logger(), "Request to stop %s from %s failed ", launch_name.c_str(), package_name.c_str());
            return;
          }

          RCLCPP_INFO(node_->get_logger(), "Request to launch %s from %s succeeded ", launch_name.c_str(), package_name.c_str());
        });
  }

  // throw on trigger function
  std::optional<std::function<void(tinyxml2::XMLElement*)>> trigger(tinyxml2::XMLElement* parameters) override
  {
    throw runner_exception("No Trigger as this is launch runner");
  }

  int childPid = -1;
  std::string launch_name;
  std::string package_name;

  rclcpp::Client<LaunchStart>::SharedPtr start_service_client_;
  rclcpp::Client<LaunchStop>::SharedPtr stop_service_client_;
};

}  // namespace capabilities2_runner