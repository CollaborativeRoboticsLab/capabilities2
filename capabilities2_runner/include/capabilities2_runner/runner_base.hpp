#pragma once

#include <stdexcept>
#include <string>
#include <functional>
#include <mutex>
#include <thread>
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
 * @param on_started name of the capability to execute on start
 * @param on_success name of the capability to execute on success
 * @param on_failure name of the capability to execute on failure
 * @param on_stopped name of the capability to execute on stop
 * @param on_started_param parameters for the capability to execute on start
 * @param on_success_param parameters for the capability to execute on success
 * @param on_failure_param parameters for the capability to execute on failure
 * @param on_stopped_param parameters for the capability to execute on stop
 */
struct event_opts
{
  std::string on_started;
  std::string on_success;
  std::string on_failure;
  std::string on_stopped;
  std::string on_started_param;
  std::string on_success_param;
  std::string on_failure_param;
  std::string on_stopped_param;
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
   * @brief Trigger the runner
   *
   * This method allows insertion of parameters in a runner after it has been initialized. it is an approach
   * to parameterise capabilities. Internally starts up RunnerBase::triggerExecution in a thread
   *
   * @param parameters pointer to tinyxml2::XMLElement that contains parameters
   *
   */
  virtual void trigger(const std::string& parameters)
  {
    parameters_ = convert_to_xml(parameters);

    RCLCPP_INFO(node_->get_logger(), "[%s] received new parameters", run_config_.interface.c_str());

    new_parameter = true;

    if (!is_running)
    {
      RCLCPP_INFO(node_->get_logger(), "[%s] execution thread not active. Activating..",
                  run_config_.interface.c_str());
                  
      executionThread = std::thread(&RunnerBase::execution, this);
    } 
    else
    {
      RCLCPP_INFO(node_->get_logger(), "[%s] execution thread running. Avoiding reactivation",
                  run_config_.interface.c_str());
    }
  }

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
    insert_id = 0;
    execute_id = -1;
    is_running = false;
    new_parameter = false;
  }

  /**
   * @brief attach events to the runner
   *
   * @param event_option event_options related for the action
   *
   * @return number of attached events
   */
  int attach_events(capabilities2_runner::event_opts& event_option,
                    std::function<void(const std::string&, const std::string&)> triggerFunction)
  {
    RCLCPP_INFO(node_->get_logger(), "[%s] accepted event options with ID : %d ", run_config_.interface.c_str(),
                insert_id);

    triggerFunction_ = triggerFunction;

    events[insert_id] = event_option;
    insert_id += 1;

    return insert_id;
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
   * @brief Trigger process to be executed.
   *
   * This method utilizes paramters set via the trigger() function
   *
   * @param parameters pointer to tinyxml2::XMLElement that contains parameters
   *
   */
  virtual void execution() = 0;

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
  virtual std::string update_on_started(std::string& parameters)
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
  virtual std::string update_on_stopped(std::string& parameters)
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
  virtual std::string update_on_failure(std::string& parameters)
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
  virtual std::string update_on_success(std::string& parameters)
  {
    return parameters;
  };

  /**
   * @brief convert an XMLElement to std::string
   *
   * @param element XMLElement element to be converted
   * @param paramters parameter to hold std::string
   *
   * @return `true` if element is not nullptr and conversion successful, `false` if element is nullptr
   */
  std::string convert_to_string(tinyxml2::XMLElement* element)
  {
    if (element)
    {
      element->Accept(&printer);
      std::string parameters = printer.CStr();
      return parameters;
    }
    else
    {
      std::string parameters = "";
      return parameters;
    }
  }

  /**
   * @brief convert an XMLElement to std::string
   *
   * @param element XMLElement element to be converted
   * @param paramters parameter to hold std::string
   *
   * @return `true` if element is not nullptr and conversion successful, `false` if element is nullptr
   */
  tinyxml2::XMLElement* convert_to_xml(const std::string& parameters)
  {
    if (parameters != "")
    {
      doc.Parse(parameters.c_str());
      tinyxml2::XMLElement* element = doc.FirstChildElement();
      return element;
    }
    else
    {
      return nullptr;
    }
  }
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
  std::map<int, event_opts> events;

  /**
   * @brief Last event tracker id to be inserted
   */
  int insert_id;

  /**
   * @brief Last parameter tracker id to be executed
   */
  int execute_id;
  /**
   * @brief pointer to XMLElement which contain parameters
   */
  tinyxml2::XMLElement* parameters_;

  /**
   * @brief thread that executes the triggerExecution functionality
   */
  std::thread executionThread;

  /**
   * @brief boolean flag for status of the thread.
   */
  bool is_running;

  /**
   * @brief boolean flag for new parameter availability.
   */
  bool new_parameter;

  /**
   * @brief external function that triggers capability runners
   */
  std::function<void(const std::string, const std::string)> triggerFunction_;

  /**
   * @brief XMLElement that is used to convert xml strings to std::string
   */
  tinyxml2::XMLPrinter printer;

  /**
   * @brief XMLElement that is used to convert std::string to xml strings
   */
  tinyxml2::XMLDocument doc;
};

}  // namespace capabilities2_runner
