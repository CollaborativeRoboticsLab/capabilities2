#pragma once

#include <stdexcept>
#include <string>
#include <functional>
#include <optional>
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
   * @param on_stopped pointer to function to execute on stopping the runner
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr,
                     std::function<void(const std::string&)> on_stopped = nullptr) = 0;

  /**
   * @brief stop the runner
   *
   */
  virtual void stop() = 0;

  /**
   * @brief trigger the runner
   *
   * this method allows insertion of parameters in a runner after it has been initialized
   * it is an approach to parameterise capabilities
   *
   * @param parameters pointer to tinyxml2::XMLElement that contains parameters
   *
   * @return std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>> function pointer to invoke
   * elsewhere such as an event callback
   */
  virtual std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>>
  trigger(std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr) = 0;

  /**
   * @brief Initializer function for initializing the base runner in place of constructor due to plugin semantics
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param on_started pointer to function to execute on starting the runner
   * @param on_terminated pointer to function to execute on terminating the runner
   */
  void init_base(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                 std::function<void(const std::string&)> on_started = nullptr,
                 std::function<void(const std::string&)> on_terminated = nullptr,
                 std::function<void(const std::string&)> on_stopped = nullptr)
  {
    // store node pointer and opts
    node_ = node;
    run_config_ = run_config;
    on_started_ = on_started;
    on_terminated_ = on_terminated;
    on_stopped_ = on_stopped;
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
  // run config getters

  /**
   * @brief Get a resource name by data type from the config
   *
   * This helps to navigate remappings from the runner config
   *
   * WARNING: this only gets the first resource found of the given type
   *
   * @param resource_type
   * @param msg_type
   * @return const std::string
   */
  const std::string get_resource_name_by_type(const std::string& resource_type, const std::string& msg_type) const
  {
    for (const auto& resource : run_config_.resources)
    {
      if (resource.resource_type == resource_type)
      {
        if (resource.msg_type == msg_type)
        {
          return resource.name;
        }
      }
    }

    throw runner_exception("no resource found: " + msg_type);
  }

  /**
   * @brief Get a parameter name by type
   *
   * @param param_type
   * @return const std::string
   */
  const std::string get_parameter_name_by_type(const std::string& param_type) const
  {
    return get_resource_name_by_type("parameter", param_type);
  }

  /**
   * @brief Get a topic name by type
   *
   * @param topic_type
   * @return const std::string
   */
  const std::string get_topic_name_by_type(const std::string& topic_type) const
  {
    return get_resource_name_by_type("topic", topic_type);
  }

  /**
   * @brief Get a service name by type
   *
   * @param srv_type
   * @return const std::string
   */
  const std::string get_service_name_by_type(const std::string& srv_type) const
  {
    return get_resource_name_by_type("service", srv_type);
  }

  /**
   * @brief Get the action name by type object
   *
   * @param action_type
   * @return const std::string
   */
  const std::string get_action_name_by_type(const std::string& action_type) const
  {
    return get_resource_name_by_type("action", action_type);
  }

  /**
   * @brief get first name of a given resource
   *
   * This can be used to get the name of the first action resource in the runner config
   *
   * @return std::string
   */
  const std::string get_first_resource_name(const std::string& resource_type) const
  {
    for (const auto& resource : run_config_.resources)
    {
      if (resource.resource_type == resource_type)
      {
        return resource.name;
      }
    }

    throw runner_exception("no " + resource_type + " resource found for interface: " + run_config_.interface);
  }

  /**
   * @brief Get the first parameter name
   *
   * @return const std::string
   */
  const std::string get_first_parameter_name() const
  {
    return get_first_resource_name("parameter");
  }

  /**
   * @brief Get the first topic name
   *
   * @return const std::string
   */
  const std::string get_first_topic_name() const
  {
    return get_first_resource_name("topic");
  }

  /**
   * @brief Get the first service name
   *
   * @return const std::string
   */
  const std::string get_first_service_name() const
  {
    return get_first_resource_name("service");
  }

  /**
   * @brief Get the first action name
   *
   * @return const std::string
   */
  const std::string get_first_action_name() const
  {
    return get_first_resource_name("action");
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

  /**
   * @param node pointer to function to execute on starting the runner
   */
  std::function<void(const std::string&)> on_started_;

  /**
   * @param node pointer to function to execute on terminating the runner
   */
  std::function<void(const std::string&)> on_terminated_;

  /**
   * @param node pointer to function to execute on stopping the runner
   */
  std::function<void(const std::string&)> on_stopped_;

  /**
   * @param node pointer to function to execute on result
   */
  std::function<void(const std::string&)> on_result_;
};

}  // namespace capabilities2_runner
