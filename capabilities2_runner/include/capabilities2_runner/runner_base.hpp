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

/**
 * @brief event options
 *
 * keeps track of events that are related to runner instances at various points of the
 * plan
 * @param on_started pointer to function to execute on start
 * @param on_success pointer to function to execute on success
 * @param on_failure pointer to function to execute on failure
 * @param on_stopped pointer to function to execute on stop
 * @param on_started_param parameters for the function to execute on start
 * @param on_success_param parameters for the function to execute on success
 * @param on_failure_param parameters for the function to execute on failure
 * @param on_stopped_param parameters for the function to execute on stop
 */
struct event_opts
{
  std::function<void(tinyxml2::XMLElement*)> on_started;
  std::function<void(tinyxml2::XMLElement*)> on_success;
  std::function<void(tinyxml2::XMLElement*)> on_failure;
  std::function<void(tinyxml2::XMLElement*)> on_stopped;
  tinyxml2::XMLElement* on_started_param;
  tinyxml2::XMLElement* on_success_param;
  tinyxml2::XMLElement* on_failure_param;
  tinyxml2::XMLElement* on_stopped_param;
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
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config) = 0;

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
  virtual std::optional<std::function<void(tinyxml2::XMLElement*)>>
  trigger(tinyxml2::XMLElement* parameters = nullptr) = 0;

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
    insert_tracker_id = 0;
    execute_tracker_id = 0;
  }

  /**
   * @brief attach events to the runner
   *
   * @param on_started pointer to function to execute on starting the runner
   * @param on_started_param parameters to triggers the on_started event with
   * @param on_failure pointer to function to execute on failure of the runner
   * @param on_failure_param parameters to triggers the on_started event with
   * @param on_success pointer to function to execute on success of the runner
   * @param on_success_param parameters to triggers the on_started event with
   * @param on_stopped pointer to function to execute on stopping the runner
   * @param on_stopped_param parameters to triggers the on_started event with
   */
  void attach_events(
      std::function<void(tinyxml2::XMLElement*)> on_started = nullptr, tinyxml2::XMLElement* on_started_param = nullptr,
      std::function<void(tinyxml2::XMLElement*)> on_failure = nullptr, tinyxml2::XMLElement* on_failure_param = nullptr,
      std::function<void(tinyxml2::XMLElement*)> on_success = nullptr, tinyxml2::XMLElement* on_success_param = nullptr,
      std::function<void(tinyxml2::XMLElement*)> on_stopped = nullptr, tinyxml2::XMLElement* on_stopped_param = nullptr)
  {
    event_opts event;

    event.on_started = on_started;
    event.on_failure = on_failure;
    event.on_stopped = on_stopped;
    event.on_success = on_success;

    event.on_started_param = on_started_param;
    event.on_failure_param = on_failure_param;
    event.on_success_param = on_success_param;
    event.on_stopped_param = on_stopped_param;

    event_tracker[insert_tracker_id] = event;
    insert_tracker_id += 1;
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
   * @brief Update on_started event parameters with new data if avaible.
   *
   * This function is used to inject new data into the XMLElement containing
   * parameters related to the on_started trigger event
   *
   * A pattern needs to be implemented in the derived class
   *
   * @param parameters pointer to the XMLElement containing parameters
   * @return pointer to the XMLElement containing updated parameters
   */
  virtual tinyxml2::XMLElement* update_on_started(tinyxml2::XMLElement* parameters)
  {
    return parameters;
  };

  /**
   * @brief Update on_stopped event parameters with new data if avaible.
   *
   * This function is used to inject new data into the XMLElement containing
   * parameters related to the on_stopped trigger event
   *
   * A pattern needs to be implemented in the derived class
   *
   * @param parameters pointer to the XMLElement containing parameters
   * @return pointer to the XMLElement containing updated parameters
   */
  virtual tinyxml2::XMLElement* update_on_stopped(tinyxml2::XMLElement* parameters)
  {
    return parameters;
  };

  /**
   * @brief Update on_failure event parameters with new data if avaible.
   *
   * This function is used to inject new data into the XMLElement containing
   * parameters related to the on_failure trigger event
   *
   * A pattern needs to be implemented in the derived class
   *
   * @param parameters pointer to the XMLElement containing parameters
   * @return pointer to the XMLElement containing updated parameters
   */
  virtual tinyxml2::XMLElement* update_on_failure(tinyxml2::XMLElement* parameters)
  {
    return parameters;
  };

  /**
   * @brief Update on_success event parameters with new data if avaible.
   *
   * This function is used to inject new data into the XMLElement containing
   * parameters related to the on_success trigger event
   *
   * A pattern needs to be implemented in the derived class
   *
   * @param parameters pointer to the XMLElement containing parameters
   * @return pointer to the XMLElement containing updated parameters
   */
  virtual tinyxml2::XMLElement* update_on_success(tinyxml2::XMLElement* parameters)
  {
    return parameters;
  };

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
   * @brief shared pointer to the capabilities node. Allows to use ros node related functionalities
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief run_config_ runner configuration
   */
  runner_opts run_config_;

  /**
   * @brief dictionary of events
   */
  std::map<int, event_opts> event_tracker;

  /**
   * @brief Last tracker id to be inserted
   */
  int insert_tracker_id;

  /**
   * @brief Last tracker id to be executed
   */
  int execute_tracker_id;
};

}  // namespace capabilities2_runner
