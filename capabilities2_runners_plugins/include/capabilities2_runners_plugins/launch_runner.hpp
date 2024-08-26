#pragma once

#include <capabilities2_runners_plugins/runner_base.hpp>

namespace capabilities2_runner
{

/**
 * @brief launch runner base class
 *
 * Create a launch file runner to run a launch file based capability
 *
 */
class LaunchRunner : public RunnerBase
{
public:
  LaunchRunner() : RunnerBase()
  {
  }

  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& opts,
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr) override
  {
    // store opts
    run_config_ = opts;

    // TODO: launch runner

    // publish event
    if (on_started)
    {
      on_started(run_config_.interface);
    }
  }

  virtual void stop(std::function<void(const std::string&)> on_stopped = nullptr) override
  {
    // TODO: stop runner

    // publish event
    if (on_stopped)
    {
      on_stopped(run_config_.interface);
    }
  }
};

}  // namespace capabilities2_runner
