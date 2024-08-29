#pragma once

#include <string>
#include <functional>
#include <rclcpp/rclcpp.hpp>

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

  /** runner plugin api */

  // incorporates event callbacks

  /**
   * @brief start the runner
   *
   * @param node
   * @param opts
   * @param on_started
   * @param on_terminated
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& opts,
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr) = 0;

  /**
   * @brief stop the runner
   *
   * @param on_stopped
   */
  virtual void stop(std::function<void(const std::string&)> on_stopped = nullptr) = 0;

  // helpers
  // init base members
  void init_base(rclcpp::Node::SharedPtr node, const runner_opts& opts)
  {
    // store node pointer and opts
    node_ = node;
    run_config_ = opts;
  }

  const std::string get_package_name()
  {
    return run_config_.interface.substr(0, run_config_.interface.find("/"));
  }

  // getters
  const std::string& get_interface() const
  {
    return run_config_.interface;
  }

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
  rclcpp::Node::SharedPtr node_;
  runner_opts run_config_;
};

}  // namespace capabilities2_runner
