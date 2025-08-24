#pragma once

#include <stdexcept>
#include <string>
#include <functional>
#include <mutex>
#include <thread>
#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>

#include <capabilities2_utils/event_types.hpp>

#include <event_logger_msgs/msg/event.hpp>
#include <event_logger_msgs/msg/event_capability.hpp>
#include <event_logger/event_client.hpp>

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
  int input_count;
};

class RunnerBase
{
public:
  using Event = event_logger_msgs::msg::Event;
  using EventType = capabilities2::event_t;

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
   * @param print_
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
    // extract the unique id for the runner and use that as the thread id
    tinyxml2::XMLElement * element = nullptr;
    element = convert_to_xml(parameters);
    element->QueryIntAttribute("id", &runner_id);

    parameters_[runner_id] = element;

    info_("received new parameters with event id : " + std::to_string(runner_id), runner_id);

    executionThreadPool[runner_id] = std::thread(&RunnerBase::execution, this, runner_id);

    info_("started execution", runner_id);
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

    current_inputs_ = 0;
    runner_id = 0;

    event_client_ = std::make_shared<EventClient>(node_, "runner", "/events");
  }

  /**
   * @brief attach events to the runner
   *
   * @param event_option event_options related for the action
   * @param triggerFunction external function that triggers capability runners
   *
   * @return number of attached events
   */
  virtual void attach_events(capabilities2::event_opts& event_option,
                    std::function<void(const std::string&, const std::string&)> triggerFunction)
  {
    info_("accepted event options with ID : " + std::to_string(event_option.event_id));

    triggerFunction_ = triggerFunction;

    events[event_option.event_id] = event_option;
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

  /**
   * @brief Get the execution status of runner. 
   * 
   * @return `true` if execution is complete, `false` otherwise.
   */
  const bool get_completion_status() const
  {
    return execution_complete_;
  }

protected:
  /**
   * @brief Trigger process to be executed.
   *
   * This method utilizes paramters set via the trigger() function
   * 
   * @param id unique identifier for the runner id. used to track the correct 
   * triggers and subsequent events.
   *
   */
  virtual void execution(int id) = 0;

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
  void info_(const std::string text, int thread_id = -1)
  {
    auto message = Event();

    message.header.stamp = node_->now();
    message.origin_node = "runners";
    message.source.capability = run_config_.interface;
    message.source.provider = run_config_.provider;
    message.target.capability = "";
    message.target.provider = "";
    message.thread_id = thread_id;
    message.type = Event::INFO;
    message.content = text;
    message.pid = -1;
    message.event = Event::UNDEFINED;

    event_client_->publish(message);
  }

  void error_(const std::string text, int thread_id = -1)
  {
    auto message = Event();

    message.header.stamp = node_->now();
    message.origin_node = "runners";
    message.source.capability = run_config_.interface;
    message.source.provider = run_config_.provider;
    message.target.capability = "";
    message.target.provider = "";
    message.thread_id = thread_id;
    message.type = Event::ERROR;
    message.content = text;
    message.pid = -1;
    message.event = Event::UNDEFINED;

    event_client_->publish(message);
  }

  void output_(const std::string text, const std::string element, int thread_id = -1)
  {
    auto message = Event();

    message.header.stamp = node_->now();
    message.origin_node = "runners";
    message.source.capability = run_config_.interface;
    message.source.provider = run_config_.provider;
    message.target.capability = "";
    message.target.provider = "";
    message.thread_id = thread_id;
    message.type = Event::INFO;
    message.content = text + " : " + element;
    message.pid = -1;
    message.event = Event::UNDEFINED;

    event_client_->publish(message);
  }

  void event_(EventType event = EventType::IDLE, int thread_id = -1, const std::string& target_capability = "",
              const std::string& target_provider = "")
  {
    auto message = Event();

    message.header.stamp = node_->now();
    message.origin_node = "runners";
    message.source.capability = run_config_.interface;
    message.source.provider = run_config_.provider;
    message.target.capability = target_capability;
    message.target.provider = target_provider;
    message.thread_id = thread_id;
    message.type = Event::RUNNER_EVENT;
    message.pid = -1;

    switch (event)
    {
      case EventType::IDLE:
        message.event = Event::IDLE;
        break;
      case EventType::STARTED:
        message.event = Event::STARTED;
        break;
      case EventType::STOPPED:
        message.event = Event::STOPPED;
        break;
      case EventType::FAILED:
        message.event = Event::FAILED;
        break;
      case EventType::SUCCEEDED:
        message.event = Event::SUCCEEDED;
        break;
      default:
        message.event = Event::UNDEFINED;
        break;
    }

    event_client_->publish(message);
  }

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
  std::map<int, capabilities2::event_opts> events;

  /**
   * @brief unique id for the runner
   */
  int runner_id;

  /**
   * @brief curent number of trigger signals received
   */
  int current_inputs_;

  /**
   * @brief system runner completion tracking
   */
  bool execution_complete_;

  /**
   * @brief pointer to XMLElement which contain parameters
   */
  std::map<int, tinyxml2::XMLElement*> parameters_;

  /**
   * @brief dictionary of threads that executes the triggerExecution functionality
   */
  std::map<int, std::thread> executionThreadPool;

  /**
   * @brief mutex for threadpool synchronisation.
   */
  std::mutex mutex_;

  /**
   * @brief conditional variable for threadpool synchronisation.
   */
  std::condition_variable cv_;

  /**
   * @brief flag for threadpool synchronisation.
   */
  bool completed_;

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

  /**
   * @brief client for publishing events
   */
  std::shared_ptr<EventClient> event_client_;
};

}  // namespace capabilities2_runner
