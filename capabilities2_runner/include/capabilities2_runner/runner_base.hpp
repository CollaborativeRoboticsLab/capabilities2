#pragma once

#include <stdexcept>
#include <string>
#include <functional>
#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>

namespace capabilities2_runner
{
/**
 * @brief runner exception
 *
 * Base class for runner exceptions
 *
 */
struct runner_exception : public std::runtime_error
{
  using std::runtime_error::runtime_error;

  runner_exception(const std::string& what) : std::runtime_error(what)
  {
  }

  virtual const char* what() const noexcept override
  {
    return std::runtime_error::what();
  }
};

/**
 * @brief resource definition
 *
 */
struct resource
{
  std::string name;
  std::string resource_type;
  std::string msg_type;
};

/**
 * @brief runner options
 *
 * Contains the options required to start and maintain a consistent runner. normally
 * loaded from the yaml file
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
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param on_started pointer to function to execute on starting the runner
   * @param on_terminated pointer to function to execute on terminating the runner
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr) = 0;

  /**
   * @brief stop the runner
   *
   * @param on_stopped pointer to function to execute on stopping the runner
   */
  virtual void stop(std::function<void(const std::string&)> on_stopped = nullptr) = 0;

  /**
   * @brief trigger the runner
   *
   * @param parameters pointer to tinyxml2::XMLElement that contains parameters
   */
  virtual void trigger(std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr) = 0;

  /**
   * @brief Initializer function for initializing the base runner in place of constructor due to plugin semantics
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   */
  void init_base(rclcpp::Node::SharedPtr node, const runner_opts& run_config)
  {
    // store node pointer and opts
    node_ = node;
    run_config_ = run_config;
  }

  /**
   * @brief get the package name to which the runner belong to.
   */
  const std::string get_package_name()
  {
    return run_config_.interface.substr(0, run_config_.interface.find("/"));
  }

  /**
   * @brief get the interface of the runner.
   */
  const std::string& get_interface() const
  {
    return run_config_.interface;
  }

  /**
   * @brief get the provider of the runner.
   */
  const std::string& get_provider() const
  {
    return run_config_.provider;
  }

  /**
   * @brief get the starter of the runner.
   */
  const std::string& get_started_by() const
  {
    return run_config_.started_by;
  }

  /**
   * @brief get the pid of the runner.
   */
  const std::string& get_pid() const
  {
    return run_config_.pid;
  }

protected:
  /**
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @param node runner configuration
   */
  runner_opts run_config_;
};

}  // namespace capabilities2_runner
