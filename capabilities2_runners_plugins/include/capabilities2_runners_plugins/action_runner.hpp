#pragma once

#include <capabilities2_runners_plugins/runner_base.hpp>

namespace capabilities2_runner
{

/**
 * @brief action runner base class
 *
 * Create an action client to run an action based capability
 *
 */
class ActionRunner : public RunnerBase
{
public:
  ActionRunner()
  {
  }

  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& opts,
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr) override
  {
    // store opts
    run_config_ = opts;

    // TODO: launch runner
  }

  virtual void stop(std::function<void(const std::string&)> on_stopped = nullptr) override
  {
    // TODO: stop runner
  }
};

}  // namespace capabilities2_runner
