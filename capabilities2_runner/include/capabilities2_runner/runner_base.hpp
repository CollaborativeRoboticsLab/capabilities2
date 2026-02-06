#pragma once

#include <stdexcept>
#include <string>
#include <functional>
#include <mutex>
#include <thread>
#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>

#include <capabilities2_events/event_node.hpp>

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

/**
 * @brief base class for all runners
 *
 * Defines the runner plugin api
 * inherits from EventNode to provide event emission
 *
 */
class RunnerBase : public capabilities2_events::EventNode
{
public:
  // static xml conversion

  /**
   * @brief convert an XMLElement to std::string
   *
   * @param element XMLElement element to be converted
   * @param parameters parameter to hold std::string
   *
   * @return `true` if element is not nullptr and conversion successful, `false` if element is nullptr
   */
  static const std::string convert_to_string(tinyxml2::XMLElement* element)
  {
    if (element)
    {
      tinyxml2::XMLPrinter printer;

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
   * @param parameters parameter to hold std::string
   *
   * @return `true` if element is not nullptr and conversion successful, `false` if element is nullptr
   */
  static tinyxml2::XMLElement* convert_to_xml(const std::string& parameters)
  {
    tinyxml2::XMLDocument doc;

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

public:
  RunnerBase() : run_config_(), execution_should_stop_(false)
  {
  }

  ~RunnerBase()
  {
    // clean up threads on destruction
    stop_execution(std::chrono::milliseconds(500));
  }

  /** runner plugin api */

  // incorporates event callbacks

  /**
   * @brief start the runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   *
   * NOTE: must call init_base in derived class implementation
   * NOTE: should call start event
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config) = 0;

  /**
   * @brief stop the runner
   *
   * NOTE: should clean up threads
   * NOTE: should call stop event
   */
  virtual void stop() = 0;

  /**
   * FIXME: implement new event subsystem
   *
   * @brief Trigger the runner
   *
   * This method allows insertion of parameters in a runner after it has been initialized. it is an approach
   * to parameterise capabilities. Internally starts up RunnerBase::triggerExecution in a thread
   *
   * @param parameters pointer to tinyxml2::XMLElement that contains parameters
   * @param bond_id unique identifier for the group of connections associated with this runner trigger event
   *
   */
  virtual void trigger(const std::string& parameters, const std::string& bond_id)
  {
    // TODO: verify parameter formatting (safe xml string or other format)
    // TODO: minimum parameter set?

    // DEPRECATED: extract trigger id from parameters
    // // extract the unique id for the runner and use that as the thread id
    // tinyxml2::XMLElement* element = nullptr;
    // element = convert_to_xml(parameters);
    // if (!element)
    // {
    //   // when this is empty it means that the trigger activation was performed
    //   // by a registered user
    //   // this is valid but we need to get a unique trigger id for the runner
    //   // to proceed safely
    //   RCLCPP_WARN(node_->get_logger(), "no trigger parameters provided");
    // }

    // std::string trigger_id = "";
    // element->QueryStringAttribute("id", &trigger_id);

    // parameters_[trigger_id] = element;

    // create a thread id
    std::string trigger_id = rclcpp::create_uuid_string();
    // namespace the thread id with bond id for later
    // could list all threads related to a bond if needed
    std::string thread_id = bond_id + "/" + trigger_id;

    // start execution thread
    {
      std::scoped_lock lock(mutex_);
      // TODO: consider emitting on start event here
      // emit_event(bond_id, capabilities2_msgs::msg::CapabilityEventCode::ON_STARTED, updated_on_started(parameters));
      execution_thread_pool_[thread_id] = std::thread(&RunnerBase::execution, this, parameters, bond_id);
    }

    // emit trigger event
    emit_event(bond_id, capabilities2_msgs::msg::CapabilityEventCode::TRIGGERED, (""));

    // BUG: thread management?

    // TODO: consider emitting on stop event here
    // emit_event(bond_id, capabilities2_msgs::msg::CapabilityEventCode::ON_STOPPED, updated_on_stopped(parameters));

    RCLCPP_DEBUG(node_->get_logger(), "started execution thread for runner id: %s", trigger_id.c_str());
  }

  /**
   * @brief Initializer function for initializing the base runner in place of constructor due to plugin semantics
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   */
  void init_base(rclcpp::Node::SharedPtr node, const runner_opts& run_config)
  {
    // store node connection source
    capabilities2_msgs::msg::Capability source_capability;
    source_capability.capability = run_config.interface;
    source_capability.provider = run_config.provider;
    set_source(source_capability);

    // store node pointer and opts
    node_ = node;
    run_config_ = run_config;

    // FIXME: should be in derived class?
    // current_inputs_ = 0;
    // DEPRECATED: event node keeps a unique id
    // runner_id = 0;
  }

  /**
   * @brief enable events system for this runner
   *
   * @param events event emitter to be used by this runner
   */
  void enable_events(std::shared_ptr<capabilities2_events::EventBase> events)
  {
    if (!is_event_emitter_set())
    {
      set_event_emitter(events);
    }
  }

  // FIXME: implement new event subsystem
  /**
   * @brief attach events to the runner
   *
   * @param event_option event_options related for the action
   * @param triggerFunction external function that triggers capability runners
   *
   * @return number of attached events
   */
  // MOVED TO: capabilities2_events::EventNode
  // virtual void attach_events(capabilities2::event_opts& event_option,
  //                            std::function<void(const std::string&, const std::string&)> triggerFunction)
  // {
  //   // info_("accepted event options with ID : " + std::to_string(event_option.event_id));

  //   triggerFunction_ = triggerFunction;

  //   events[event_option.event_id] = event_option;
  // }
  /**
   * @brief make EventNode::add_connection public method on runner api
   *
   * @param connection_id
   * @param type
   * @param target
   * @param callback
   */
  void add_connection(const std::string& connection_id, const capabilities2_msgs::msg::CapabilityEventCode& type,
                      const capabilities2_msgs::msg::Capability& target,
                      std::function<void(const std::string&, const std::string&, const std::string&)> callback)
  {
    // add connection to this runner to target capability
    // this allows the runner to emit events on state changes
    // to the target capability
    // the connection ID format is: "bond_id/trigger_id"
    // which allows event emission to extract bond_id for access control
    EventNode::add_connection(connection_id, type, target, callback);
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

  // FIXME: not used?
  /**
   * @brief Get the execution status of runner.
   *
   * @return `true` if execution is complete, `false` otherwise.
   */
  // const bool get_completion_status() const
  // {
  //   return execution_complete_;
  // }

protected:
  /**
   * @brief Trigger process to be executed.
   *
   * This method utilizes parameters set via the trigger() function
   *
   * @param parameters parameters for the execution
   *
   * NOTE: should call success and failure events appropriately
   */
  virtual void execution(const std::string& parameters, const std::string& bond_id) = 0;

  /** */
  void stop_execution(std::chrono::milliseconds timeout = std::chrono::milliseconds(500))
  {
    // signal listening threads to stop
    execution_should_stop_ = true;

    // try join in a non-blocking way and log if threads are not cleaned up properly
    {
      std::scoped_lock lock(mutex_);
      for (auto& [thread_id, exec_thread] : execution_thread_pool_)
      {
        if (exec_thread.joinable())
        {
          if (exec_thread.try_join_for(timeout))
          {
            RCLCPP_DEBUG(node_->get_logger(), "execution %s joined successfully", thread_id.c_str());
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "execution %s did not stop in time, detaching", thread_id.c_str());
            exec_thread.detach();  // don't block, but log the issue
          }
        }
      }

      // clear the thread pool
      execution_thread_pool_.clear();
    }
  }

  // FIXME: implement new event subsystem
  // STATE CHANGE PARAMETER HELPERS

  /**
   * @brief Update on_started event parameters with new data if available.
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
   * @brief Update on_stopped event parameters with new data if available.
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
   * @brief Update on_failure event parameters with new data if available.
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
   * @brief Update on_success event parameters with new data if available.
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
  // FIXME: implement new event subsystem
  // STATE CHANGE HELPERS

  /**
   * @brief emit STARTED event
   *
   * @param bond_id
   * @param parameters
   */
  void emit_started(const std::string& bond_id, const std::string& parameters = "")
  {
    capabilities2_msgs::msg::CapabilityEventCode event_type;
    event_type.code = capabilities2_msgs::msg::CapabilityEventCode::STARTED;
    emit_event(bond_id, event_type, parameters);
  }

  /**
   * @brief emit STOPPED event
   *
   * @param bond_id
   * @param parameters
   */
  void emit_stopped(const std::string& bond_id, const std::string& parameters = "")
  {
    capabilities2_msgs::msg::CapabilityEventCode event_type;
    event_type.code = capabilities2_msgs::msg::CapabilityEventCode::STOPPED;
    emit_event(bond_id, event_type, parameters);
  }

  /**
   * @brief emit SUCCEEDED event
   *
   * @param bond_id
   * @param parameters
   */
  void emit_succeeded(const std::string& bond_id, const std::string& parameters = "")
  {
    capabilities2_msgs::msg::CapabilityEventCode event_type;
    event_type.code = capabilities2_msgs::msg::CapabilityEventCode::SUCCEEDED;
    emit_event(bond_id, event_type, parameters);
  }

  /**
   * @brief emit FAILED event
   *
   * @param bond_id
   * @param parameters
   */
  void emit_failed(const std::string& bond_id, const std::string& parameters = "")
  {
    capabilities2_msgs::msg::CapabilityEventCode event_type;
    event_type.code = capabilities2_msgs::msg::CapabilityEventCode::FAILED;
    emit_event(bond_id, event_type, parameters);
  }

  /**
   * @brief shared pointer to the capabilities node
   * Allows to use ros node related functionalities
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief run_config_ runner configuration
   */
  runner_opts run_config_;

  /**
   * @brief dictionary of events
   */
  // DEPRECATED: moved to capabilities2_events::EventNode
  // std::map<int, capabilities2::event_opts> events;

  /**
   * @brief unique id for the runner
   */
  // DEPRECATED: moved eventNode which uniquely identifies itself
  // int runner_id;

  // FIXME: should be moved to derived class
  /**
   * @brief current number of trigger signals received
   */
  // int current_inputs_;

  // FIXME: should be moved to derived class
  /**
   * @brief system runner completion tracking
   */
  // bool execution_complete_;

  /**
   * @brief pointer to XMLElement which contain parameters
   * NOTE: std::string is thread/runner id
   */
  // DEPRECATED: moved to execution function parameter since events track elsewhere
  // just need to insert parameters into event emission during execution
  // std::map<std::string, tinyxml2::XMLElement*> parameters_;

  /**
   * @brief dictionary of threads that executes the execute function
   */
  std::map<std::string, std::thread> execution_thread_pool_;

  /**
   * @brief mutex for threadpool synchronisation.
   */
  std::mutex mutex_;

  /**
   * @brief flag to signal execution threads to stop.
   */
  std::atomic<bool> execution_should_stop_;

  /**
   * @brief conditional variable for threadpool synchronisation.
   */
  // FIXME: move for derived class use
  // std::condition_variable cv_;

  /**
   * @brief flag for threadpool synchronisation.
   */
  // FIXME: move for derived class use
  // bool completed_;

  // FIXME: implement new event subsystem
  /**
   * @brief external function that triggers capability runners
   */
  // DEPRECATED: moved to capabilities2_events::EventNode
  // std::function<void(const std::string, const std::string)> triggerFunction_;

  // TODO: try make static helper class for xml conversion
  /**
   * @brief XMLElement that is used to convert xml strings to std::string
   */
  // tinyxml2::XMLPrinter printer;

  /**
   * @brief XMLElement that is used to convert std::string to xml strings
   */
  // tinyxml2::XMLDocument doc;

  /**
   * @brief client api for event emission
   */
  // DEPRECATED: moved to capabilities2_events::EventNode
  // std::shared_ptr<capabilities2_events::EventBase> event_;
};

}  // namespace capabilities2_runner
