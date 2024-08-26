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

  virtual void start(const runner_opts& opts) override
  {
    // store opts
    run_config_ = opts;

    // TODO: launch runner
  }

  virtual void stop() override
  {
    // TODO: launch runner
  }
};

}  // namespace capabilities2_runner
