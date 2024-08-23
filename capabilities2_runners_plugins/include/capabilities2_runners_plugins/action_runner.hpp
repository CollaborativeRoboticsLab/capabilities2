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
  ActionRunner(/* args */);
  ~ActionRunner();
};

}  // namespace capabilities2_runner
