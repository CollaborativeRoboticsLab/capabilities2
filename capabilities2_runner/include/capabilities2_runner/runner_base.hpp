#pragma once

#include <stdexcept>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <capabilities2_events/event_node.hpp>
#include <capabilities2_events/event_parameters.hpp>

#include <capabilities2_msgs/msg/capability.hpp>
#include <capabilities2_msgs/msg/capability_event_code.hpp>

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
 * Defines the runner plugin api. Inherits from EventNode to provide event emission
 *
 */
class RunnerBase : public capabilities2_events::EventNode
{
public:
  RunnerBase() : node_(nullptr), run_config_()
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
   *
   * @attention Must call init_base in derived class implementation and should call start event
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& bond_id) = 0;

  /**
   * @brief stop the runner
   *
   * @attention should clean up threads and should call stop event
   */
  virtual void stop(const std::string& bond_id, const std::string& instance_id = "") = 0;

  /**
   * FIXME: implement new event subsystem
   * @brief Trigger the runner
   *
   * This method allows insertion of parameters in a runner after it has been initialized. it is an approach
   * to parameterise capabilities. Internally starts up RunnerBase::triggerExecution in a thread
   *
   * @param parameters capability options that contain parameters for the trigger
   * @param bond_id unique identifier for the group of connections associated with this runner trigger event
   * @param instance_id unique identifier for the instance of the capability
   * @attention should call success and failure events with parameters and bond_id when the trigger process
   * completes.
   *
   */
  virtual void trigger(capabilities2_events::EventParameters& parameters, const std::string& bond_id,
                       const std::string& instance_id) = 0;

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

  /**
   * @brief make EventNode::add_connection public method on runner api.
   *
   * add connection to this runner to target capability this allows the runner to emit events on state changes
   * to the target capability the connection ID format is: "bond_id/trigger_id"  which allows event emission to
   * extract bond_id for access control
   *
   * @param connection_id unique identifier for the connection (format: "bond_id/trigger_id")
   * @param type type of event to connect to
   * @param target target capability to connect to
   * @param callback callback to trigger target capability with (capability, parameters, bond_id)
   */
  void add_connection(const std::string& connection_id, const capabilities2_msgs::msg::CapabilityEventCode& type,
                      const capabilities2_msgs::msg::Capability& target,
                      std::function<void(const std::string&, const std::string&, const std::string&,
                                         capabilities2_events::EventParameters)>
                          callback)
  {
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

protected:
  // FIXME: implement new event subsystem

  /**
   * @brief Update on_started event parameters with new data if available.
   *
   * This function is used to inject new data into the CapabilityOptions containing
   * parameters related to the on_started trigger event
   *
   * A pattern needs to be implemented in the derived class
   *
   * @return CapabilityOptions containing new parameters
   */
  virtual capabilities2_events::EventParameters param_on_started()
  {
    return capabilities2_events::EventParameters();
  };

  /**
   * @brief Update on_stopped event parameters with new data if available.
   *
   * This function is used to inject new data into the CapabilityOptions containing
   * parameters related to the on_stopped trigger event
   *
   * A pattern needs to be implemented in the derived class
   *
   * @return CapabilityOptions containing new parameters
   */
  virtual capabilities2_events::EventParameters param_on_stopped()
  {
    return capabilities2_events::EventParameters();
  };

  /**
   * @brief Update on_failure event parameters with new data if available.
   *
   * This function is used to inject new data into the CapabilityOptions containing
   * parameters related to the on_failure trigger event
   *
   * A pattern needs to be implemented in the derived class
   *
   * @return CapabilityOptions containing new parameters
   */
  virtual capabilities2_events::EventParameters param_on_failure()
  {
    return capabilities2_events::EventParameters();
  };

  /**
   * @brief Update on_success event parameters with new data if available.
   *
   * This function is used to inject new data into the CapabilityOptions containing
   * parameters related to the on_success trigger event
   *
   * A pattern needs to be implemented in the derived class
   *
   * @return CapabilityOptions containing new parameters
   */
  virtual capabilities2_events::EventParameters param_on_success()
  {
    return capabilities2_events::EventParameters();
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

  // FIXME: implement new event subsystem
  // STATE CHANGE HELPERS

  /**
   * @brief emit STARTED event
   *
   * @param bond_id
   * @param parameters
   */
  void emit_started(const std::string& bond_id, const std::string& instance_id,
                    capabilities2_events::EventParameters parameters = capabilities2_events::EventParameters())
  {
    RCLCPP_INFO(node_->get_logger(), "emitting STARTED event with bond_id: %s", bond_id.c_str());
    emit_event(bond_id, instance_id, capabilities2_msgs::msg::CapabilityEventCode::STARTED, parameters);
  }

  /**
   * @brief emit STOPPED event
   *
   * @param bond_id
   * @param parameters
   */
  void emit_stopped(const std::string& bond_id, const std::string& instance_id,
                    capabilities2_events::EventParameters parameters = capabilities2_events::EventParameters())
  {
    RCLCPP_INFO(node_->get_logger(), "emitting STOPPED event with bond_id: %s", bond_id.c_str());
    emit_event(bond_id, instance_id, capabilities2_msgs::msg::CapabilityEventCode::STOPPED, parameters);
  }

  /**
   * @brief emit SUCCEEDED event
   *
   * @param bond_id
   * @param parameters
   */
  void emit_succeeded(const std::string& bond_id, const std::string& instance_id,
                      capabilities2_events::EventParameters parameters = capabilities2_events::EventParameters())
  {
    RCLCPP_INFO(node_->get_logger(), "emitting SUCCEEDED event with bond_id: %s", bond_id.c_str());
    emit_event(bond_id, instance_id, capabilities2_msgs::msg::CapabilityEventCode::SUCCEEDED, parameters);
  }

  /**
   * @brief emit FAILED event
   *
   * @param bond_id
   * @param parameters
   */
  void emit_failed(const std::string& bond_id, const std::string& instance_id,
                   capabilities2_events::EventParameters parameters = capabilities2_events::EventParameters())
  {
    RCLCPP_INFO(node_->get_logger(), "emitting FAILED event with bond_id: %s", bond_id.c_str());
    emit_event(bond_id, instance_id, capabilities2_msgs::msg::CapabilityEventCode::FAILED, parameters);
  }

protected:
  /**
   * @brief shared pointer to the capabilities node
   * Allows to use ros node related functionalities
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief run_config_ runner configuration
   */
  runner_opts run_config_;
};

}  // namespace capabilities2_runner
