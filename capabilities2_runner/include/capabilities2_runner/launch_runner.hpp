#pragma once

#include <filesystem>
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

  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config, std::string& parameters, 
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr) override
  {
    // store node pointer and run_config
    init_base(node, run_config);

    // create an action client
    action_client_ = rclcpp_action::create_client<capabilities2_msgs::action::Launch>(node_, "/capabilities_launch_"
                                                                                             "proxy/launch");

    // wait for action server
    RCLCPP_INFO(node_->get_logger(), "%s waiting for launch action", run_config_.interface.c_str());

    if (!action_client_->wait_for_action_server(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(node_->get_logger(), "%s failed to connect to action server", run_config_.interface.c_str());
      throw runner_exception("failed to connect to action server");
    }

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

    // TEST: check the path
    RCLCPP_INFO(node_->get_logger(), "launch file path: %s", launch_file_path.c_str());

    // create a launch goal
    capabilities2_msgs::action::Launch::Goal goal;
    goal.launch_file_path = launch_file_path;

    // set send goal options for the action
    auto send_goal_options = rclcpp_action::Client<capabilities2_msgs::action::Launch>::SendGoalOptions();
    // goal response callback
    send_goal_options.goal_response_callback =
        [this, on_started, on_terminated](
            const rclcpp_action::ClientGoalHandle<capabilities2_msgs::action::Launch>::SharedPtr& goal_handle) {
          // store goal handle
          goal_handle_ = goal_handle;
        };
        
    // result callback
    send_goal_options.result_callback =
        [this, on_started, on_terminated](
            const rclcpp_action::ClientGoalHandle<capabilities2_msgs::action::Launch>::WrappedResult& wrapped_result) {
          if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
          {
            // publish event
            if (on_started)
            {
              on_started(run_config_.interface);
            }
          }
          else
          {
            // send terminated event
            if (on_terminated)
            {
              on_terminated(run_config_.interface);
            }

            // throw runner_exception("launch action failed");
          }
        };

    // launch runner using action client
    action_client_->async_send_goal(goal, send_goal_options);
  }

  virtual void stop(std::function<void(const std::string&)> on_stopped = nullptr) override
  {
    // if the node pointer is empty then throw an error
    // this means that the runner was not started and is being used out of order
    if (!node_)
    {
      throw runner_exception("cannot stop runner that was not started");
    }

    // throw an error if the action client is null
    // this can happen if the runner is not able to find the action resource
    if (!action_client_)
    {
      throw runner_exception("cannot stop runner action that was not started");
    }

    // stop runner using action client
    if (goal_handle_)
    {
      try
      {
        auto cancel_future = action_client_->async_cancel_goal(
            goal_handle_, [this, on_stopped](action_msgs::srv::CancelGoal_Response::SharedPtr response) {
              if (response->return_code != action_msgs::srv::CancelGoal_Response::ERROR_NONE)
              {
                // throw runner_exception("failed to cancel runner");
              }

              // publish event
              if (on_stopped)
              {
                on_stopped(run_config_.interface);
              }
            });

        // wait for action to be stopped
        // hold the thread for 2 seconds
        // to help keep callbacks in scope
        rclcpp::spin_until_future_complete(node_, cancel_future, std::chrono::seconds(2));
      }
      catch (const rclcpp_action::exceptions::UnknownGoalHandleError& e)
      {
        RCLCPP_ERROR(node_->get_logger(), "failed to cancel goal: %s", e.what());
        throw runner_exception(e.what());
      }
    }
  }

private:
  std::string launch_file_path;

  rclcpp_action::ClientGoalHandle<capabilities2_msgs::action::Launch>::SharedPtr goal_handle_;
};

}  // namespace capabilities2_runner
