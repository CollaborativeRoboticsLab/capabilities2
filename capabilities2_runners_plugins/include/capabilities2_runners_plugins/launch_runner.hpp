#pragma once

#include <std_msgs/msg/string.hpp>
#include <capabilities2_msgs/action/launch.hpp>
#include <capabilities2_runners_plugins/action_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief launch runner base class
 *
 * Create a launch file runner to run a launch file based capability
 *
 */
class LaunchRunner : public ActionRunner<capabilities2_msgs::action::Launch>
{
public:
  LaunchRunner() : ActionRunner()
  {
  }

  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& opts,
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr) override
  {
    init_action(node, opts, "capabilities2_msg/action/Launch");

    // get full path to launch file
    std::string launch_file_path = get_package_name() + "/" + run_config_.runner;

    // create a launch goal
    capabilities2_msgs::action::Launch::Goal goal;
    goal.launch_file_path = launch_file_path;

    // launch runner using action client
    std::shared_future<rclcpp_action::ClientGoalHandle<capabilities2_msgs::action::Launch>::SharedPtr> goal_handle =
        action_client_->async_send_goal(goal);

    // if goal handle is null then throw an error
    if (goal_handle.get() == nullptr)
    {
      // send terminated event
      if (on_terminated)
      {
        on_terminated(run_config_.interface);
      }

      throw std::runtime_error("failed to launch runner");
    }

    // publish event
    if (on_started)
    {
      on_started(run_config_.interface);
    }
  }

  virtual void stop(std::function<void(const std::string&)> on_stopped = nullptr) override
  {
    // if the node pointer is empty then throw an error
    // this means that the runner was not started and is being used out of order
    if (!node_)
    {
      throw std::runtime_error("cannot stop runner that was not started");
    }

    // stop runner
    auto cancel_future = action_client_->async_cancel_all_goals();

    // if cancel future is null then throw an error
    if (cancel_future.get() == nullptr)
    {
      throw std::runtime_error("failed to cancel runner");
    }

    // check future for error
    if (cancel_future.get()->return_code != action_msgs::srv::CancelGoal_Response::ERROR_NONE)
    {
      throw std::runtime_error("failed to cancel runner");
    }

    // publish event
    if (on_stopped)
    {
      on_stopped(run_config_.interface);
    }
  }

private:
  std::string launch_file_path;
};

}  // namespace capabilities2_runner
