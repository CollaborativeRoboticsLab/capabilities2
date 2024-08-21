#pragma once

#include <memory>
#include <map>
#include <string>
#include <vector>
#include <functional>

#include <uuid/uuid.h>

#include <rclcpp/rclcpp.hpp>

#include <capabilities2_server/capabilities_db.hpp>
#include <capabilities2_server/bond_cache.hpp>
#include <capabilities2_server/runner_cache.hpp>

#include <capabilities2_msgs/msg/remapping.hpp>
#include <capabilities2_msgs/msg/capability_spec.hpp>
#include <capabilities2_msgs/msg/capability_event.hpp>
#include <capabilities2_msgs/srv/get_remappings.hpp>
#include <capabilities2_msgs/msg/running_capability.hpp>

namespace capabilities2_server
{

/**
 * @brief capabilities api
 * mapping message based logic of capabilities to database models
 * handles message conversion and db access
 * also tracks cache for bonds and runners
 *
 */
class CapabilitiesAPI
{
public:
  CapabilitiesAPI()
  {
  }

  // connect db
  void connect(const std::string& db_file,
               rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_ptr)
  {
    // set logger
    node_logging_interface_ptr_ = node_logging_interface_ptr;

    // connect db
    cap_db_ = std::make_unique<CapabilitiesDB>(db_file);
  }

  // control api
  void start_capability(const std::string& capability, const std::string& provider)
  {
    // create a runner id
    std::string runner_id = "runner_" + capability + "_" + provider;

    // create a new runner
    runner_cache_.add_runner(runner_id, capability);

    // XXX TODO: start the runner
  }

  void stop_capability(const std::string& capability)
  {
    // get runners
    std::vector<std::string> runners = runner_cache_.get_runners(capability);

    // stop runners
    for (const auto& runner : runners)
    {
      // XXX TODO: stop the runner
      runner_cache_.remove_runner(runner);
    }
  }

  void free_capability(const std::string& capability, const std::string& bond_id)
  {
    // remove bond from cache for capability
    bond_cache_.remove_bond(capability, bond_id);

    // stop the capability if no more bonds
    if (bond_cache_.get_bonds(capability).empty())
    {
      stop_capability(capability);
    }
  }

  void use_capability(const std::string& capability, const std::string& provider, const std::string& bond_id)
  {
    // add bond to cache for capability
    bond_cache_.add_bond(capability, bond_id);

    // start the capability with the provider if not already running
    if (runner_cache_.get_runners(capability).empty())
    {
      start_capability(capability, provider);
    }
  }

  // capability api
  void add_capability(const capabilities2_msgs::msg::CapabilitySpec& spec)
  {
    // peak at the spec header
    models::header_model_t header;
    header.from_yaml(YAML::Load(spec.content));

    // XXX TODO: check for existing capability

    // check type, create model and add to db
    if (spec.type == capabilities2_msgs::msg::CapabilitySpec::CAPABILITY_INTERFACE)
    {
      // convert message to model and add to db
      models::interface_model_t model;
      model.from_yaml(YAML::Load(spec.content));
      cap_db_->insert_interface(model);
    }
    else if (spec.type == capabilities2_msgs::msg::CapabilitySpec::SEMANTIC_CAPABILITY_INTERFACE)
    {
      models::semantic_interface_model_t model;
      model.from_yaml(YAML::Load(spec.content));
      cap_db_->insert_semantic_interface(model);
    }
    else if (spec.type == capabilities2_msgs::msg::CapabilitySpec::CAPABILITY_PROVIDER)
    {
      models::provider_model_t model;
      model.from_yaml(YAML::Load(spec.content));
      cap_db_->insert_provider(model);
    }
    else
    {
      RCLCPP_ERROR(node_logging_interface_ptr_->get_logger(), "unknown capability type: %s", spec.type.c_str());
    }
  }

  // query api
  std::vector<std::string> get_interfaces()
  {
    std::vector<std::string> interfaces;

    for (const auto& interface : cap_db_->get_interfaces())
    {
      interfaces.push_back(interface.header.name);
    }

    return interfaces;
  }

  std::vector<std::string> get_sematic_interfaces(const std::string& interface)
  {
    std::vector<std::string> semantic_interfaces;

    for (const auto& semantic_interface : cap_db_->get_semantic_interfaces_by_interface(interface))
    {
      semantic_interfaces.push_back(semantic_interface.header.name);
    }

    return semantic_interfaces;
  }

  std::vector<std::string> get_providers(const std::string& interface, const bool& include_semantic)
  {
    std::vector<std::string> providers;

    for (const auto& provider : cap_db_->get_providers_by_interface(interface))
    {
      providers.push_back(provider.header.name);
    }

    // if include_semantic is true, add providers from semantic interfaces
    if (include_semantic)
    {
      for (const auto& semantic_interface : cap_db_->get_semantic_interfaces_by_interface(interface))
      {
        for (const auto& provider : cap_db_->get_providers_by_interface(semantic_interface.header.name))
        {
          providers.push_back(provider.header.name);
        }
      }
    }

    return providers;
  }

  /**
   * @brief Get the capability spec object for a given resource
   * return the spec in yaml string format
   *
   * @param resource
   * @return const capabilities2_msgs::msg::CapabilitySpec
   */
  const capabilities2_msgs::msg::CapabilitySpec get_capability_spec(const std::string& resource)
  {
    capabilities2_msgs::msg::CapabilitySpec msg;

    // search db for resource
    // interfaces
    models::interface_model_t interface = cap_db_->get_interface(resource);

    if (!interface.header.name.empty())
    {
      msg.package = interface.header.name;
      msg.type = interface.header.type;
      // get spec convert to yaml then to string
      msg.content = YAML::Dump(interface.to_yaml());
    }

    // semantic interfaces
    models::semantic_interface_model_t semantic_interface = cap_db_->get_semantic_interface(resource);

    if (!semantic_interface.header.name.empty())
    {
      msg.package = semantic_interface.header.name;
      msg.type = semantic_interface.header.type;
      msg.content = YAML::Dump(semantic_interface.to_yaml());
    }

    // providers
    models::provider_model_t provider = cap_db_->get_provider(resource);

    if (!provider.header.name.empty())
    {
      msg.package = provider.header.name;
      msg.type = provider.header.type;
      msg.content = YAML::Dump(provider.to_yaml());
    }

    return msg;
  }

  std::vector<capabilities2_msgs::msg::CapabilitySpec> get_capability_specs()
  {
    // get all capability specs from db
    std::vector<capabilities2_msgs::msg::CapabilitySpec> specs;

    // interfaces
    for (const auto& interface : cap_db_->get_interfaces())
    {
      capabilities2_msgs::msg::CapabilitySpec msg;
      msg.package = interface.header.name;
      msg.type = interface.header.type;
      msg.content = YAML::Dump(interface.to_yaml());
      specs.push_back(msg);
    }

    // semantic interfaces
    for (const auto& semantic_interface : cap_db_->get_semantic_interfaces())
    {
      capabilities2_msgs::msg::CapabilitySpec msg;
      msg.package = semantic_interface.header.name;
      msg.type = semantic_interface.header.type;
      msg.content = YAML::Dump(semantic_interface.to_yaml());
      specs.push_back(msg);
    }

    // providers
    for (const auto& provider : cap_db_->get_providers())
    {
      capabilities2_msgs::msg::CapabilitySpec msg;
      msg.package = provider.header.name;
      msg.type = provider.header.type;
      msg.content = YAML::Dump(provider.to_yaml());
      specs.push_back(msg);
    }

    return specs;
  }

  void get_remappings(const std::string& capability_spec,
                      std::shared_ptr<capabilities2_msgs::srv::GetRemappings::Response> res)
  {
    // get remappings for a capability spec

    // get re-map-able specs from db, these are semantic interfaces and providers
    models::remappable_base_t remappable = cap_db_->get_remappable(capability_spec);

    // get remappings
    for (const auto& topic : remappable.remappings.topics)
    {
      capabilities2_msgs::msg::Remapping msg;
      msg.key = topic.from;
      msg.value = topic.to;
      res->topics.push_back(msg);
    }

    for (const auto& service : remappable.remappings.services)
    {
      capabilities2_msgs::msg::Remapping msg;
      msg.key = service.from;
      msg.value = service.to;
      res->services.push_back(msg);
    }

    for (const auto& action : remappable.remappings.actions)
    {
      capabilities2_msgs::msg::Remapping msg;
      msg.key = action.from;
      msg.value = action.to;
      res->actions.push_back(msg);
    }

    for (const auto& parameter : remappable.remappings.parameters)
    {
      capabilities2_msgs::msg::Remapping msg;
      msg.key = parameter.from;
      msg.value = parameter.to;
      res->parameters.push_back(msg);
    }
  }

  // runner api
  std::vector<capabilities2_msgs::msg::RunningCapability> get_running_capabilities()
  {
    std::vector<capabilities2_msgs::msg::RunningCapability> running_capabilities;

    // XXX TODO: get running capabilities from runners

    return running_capabilities;
  }

  // event api
  const capabilities2_msgs::msg::CapabilityEvent on_capability_started();
  const capabilities2_msgs::msg::CapabilityEvent on_capability_stopped();
  const capabilities2_msgs::msg::CapabilityEvent on_capability_used();
  const capabilities2_msgs::msg::CapabilityEvent on_capability_freed();
  const capabilities2_msgs::msg::CapabilityEvent on_capability_error();

  // bond api
  const std::string establish_bond(rclcpp::Node::SharedPtr node)
  {
    // create a new unique bond id
    std::string bond_id = CapabilitiesAPI::gen_bond_id();

    // establish a bond and start liveness functions (on_broken, on_formed)
    // bind the bond id to the callback functions
    bond_cache_.start(bond_id, node, std::bind(&CapabilitiesAPI::on_bond_broken, this, bond_id),
                      std::bind(&CapabilitiesAPI::on_bond_established, this, bond_id));

    // return bond id
    return bond_id;
  }

  void on_bond_established(const std::string& bond_id)
  {
    // log bond established event
    RCLCPP_INFO(node_logging_interface_ptr_->get_logger(), "bond established with id: %s", bond_id.c_str());
  }

  void on_bond_broken(const std::string& bond_id)
  {
    // log warning
    RCLCPP_WARN(node_logging_interface_ptr_->get_logger(), "bond broken with id: %s", bond_id.c_str());

    // get capabilities requested by the bond
    std::vector<std::string> capabilities = bond_cache_.get_capabilities(bond_id);

    // free capabilities
    for (const auto& capability : capabilities)
    {
      free_capability(capability, bond_id);
    }
  }

public:
  /**
   * @brief generate a unique bond id
   *
   * @return const std::string
   */
  static const std::string gen_bond_id()
  {
    // create a new uuid for bond id
    uuid_t uuid;
    uuid_generate_random(uuid);
    char uuid_str[40];
    uuid_unparse(uuid, uuid_str);
    return std::string(uuid_str);
  }

private:
  // logger
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_ptr_;

  // db
  std::unique_ptr<CapabilitiesDB> cap_db_;

  // caches
  BondCache bond_cache_;
  RunnerCache runner_cache_;
};

}  // namespace capabilities2_server
