#pragma once

#include <capabilities2_runner/action_runner.hpp>
#include <capabilities2_msgs/action/capability.hpp>

namespace capabilities2_runner
{

/**
 * @brief encapsulated capability runner
 *
 * Create an action client to run an action based capability
 * using an encapsulated capability action
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
                     std::function<void(const std::string&)> on_terminated = nullptr,
                     std::function<void(const std::string&)> on_stopped = nullptr) override
  {
    // get action name for Capability type and init the action
    // create action client
    init_action(node, opts, get_action_name_by_type("capabilities2_msgs/action/Capability"), on_started, on_terminated,
                on_stopped);

    //
  }

private:
};

}  // namespace capabilities2_runner
