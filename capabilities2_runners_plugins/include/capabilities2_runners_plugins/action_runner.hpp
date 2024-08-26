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
