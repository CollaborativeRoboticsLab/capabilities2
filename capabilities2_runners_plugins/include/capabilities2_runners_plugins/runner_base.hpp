#pragma once

#include <iostream>
#include <string>

namespace capabilities2_runner
{
struct resource
{
  std::string name;
  std::string resource_type;
  std::string msg_type;
};

/**
 * @brief runner options
 *
 * Contains the options required to start and maintain a consistent runner
 *
 */
struct runner_opts
{
  std::string interface;
  std::string provider;
  std::vector<resource> resources;
  std::string global_namespace;
  std::string runner;
  std::string started_by;
  std::string pid;
};

class RunnerBase
{
public:
  RunnerBase() : run_config_()
  {
  }

  ~RunnerBase() = default;

  // runner plugin api
  virtual void start(const runner_opts& opts) = 0;
  virtual void stop() = 0;

  // getters
  const std::string& get_provider() const
  {
    return run_config_.provider;
  }

  const std::string& get_started_by() const
  {
    return run_config_.started_by;
  }

  const std::string& get_pid() const
  {
    return run_config_.pid;
  }

protected:
  runner_opts run_config_;
};

}  // namespace capabilities2_runner
