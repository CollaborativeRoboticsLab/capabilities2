#pragma once

#include <capabilities2_runner/action_runner.hpp>
#include <capabilities2_msgs/action/capability.hpp>

namespace capabilities2_runner
{

/**
 * @brief encapsulated capability runner
 *
 * Create an action client to run an action based capability
 * using and encapsulated capability action
 * this allows a system to run an action that is not a managed action
 *
 */
class EnCapRunner : public ActionRunner<capabilities2_msgs::action::Capability>
{
public:
  EnCapRunner() : ActionRunner()
  {
  }

  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& opts,
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr) override
  {
    // create action client
    init_action(node, opts, "capabilities2_msgs/action/Capability");
  }

  virtual void stop(std::function<void(const std::string&)> on_stopped = nullptr) override
  {
  }

private:
};

}  // namespace capabilities2_runner
