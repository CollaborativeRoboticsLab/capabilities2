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
  LaunchRunner(/* args */);
  ~LaunchRunner();
};

}  // namespace capabilities2_runner
