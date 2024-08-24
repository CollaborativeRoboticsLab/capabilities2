#pragma once

#include <iostream>
#include <string>

namespace capabilities2_runner
{

/** */
struct runner_opts
{
};

class RunnerBase
{
public:
  RunnerBase(/* args */)
  {
    // FIXME: temporary for testing
    std::cout << "running runner base" << std::endl;
  }

  ~RunnerBase() = default;

  // runner plugin api
  // virtual void start(const runner_opts& opts) = 0;
  // virtual void stop() = 0;
};

}  // namespace capabilities2_runner
