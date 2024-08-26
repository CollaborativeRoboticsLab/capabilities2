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

    // log
    RCLCPP_INFO(node_logging_interface_ptr_->get_logger(), "CAPAPI connected to db: %s", db_file.c_str());
  }

  // init runner events
  void init_events(std::function<void(const std::string&)> on_started,
                   std::function<void(const std::string&)> on_stopped,
                   std::function<void(const std::string&)> on_terminated)
  {
    runner_cache_.set_on_started(on_started);
    runner_cache_.set_on_stopped(on_stopped);
    runner_cache_.set_on_terminated(on_terminated);
  }

  // control api
  void start_capability(rclcpp::Node::SharedPtr node, const std::string& capability, const std::string& provider)
  {
    // get the running model from the db
    models::running_model_t running = cap_db_->get_running(provider);

    // start all dependencies
    // go through the running model and start the necessary dependencies
    for (const auto& run : running.dependencies)
    {
      RCLCPP_INFO(node_logging_interface_ptr_->get_logger(), "found dependency: %s", run.interface.c_str());

      // make an internal 'use' bond for the capability dependency
      bind_dependency(run.interface);

      // add the runner to the cache
      start_capability(node, run.interface, run.provider);
    }

    // get the provider specification for the capability
    models::run_config_model_t run_config = cap_db_->get_run_config(provider);

    // create a new runner
    // this call implicitly starts the runner
    // create a runner id which is the cap name to uniquely identify the runner
    // this means only one runner per capability name
    // TODO: consider the logic for multiple runners per capability
    try
    {
      runner_cache_.add_runner(node, capability, run_config);

      // log
      RCLCPP_INFO(node_logging_interface_ptr_->get_logger(), "started capability: %s with provider: %s",
                  capability.c_str(), provider.c_str());
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_WARN(node_logging_interface_ptr_->get_logger(), "could not start runner: %s", e.what());
    }
  }

  void stop_capability(const std::string& capability)
  {
    // get the provider from runner
    std::string provider = runner_cache_.provider(capability);
    // get the running model from the db
    models::running_model_t running = cap_db_->get_running(provider);

    // unbind and stop dependencies
    // FIXME: this unrolls the dependency tree from the bottom up but should probably be top down
    for (const auto& run : running.dependencies)
    {
      RCLCPP_INFO(node_logging_interface_ptr_->get_logger(), "freeing dependency: %s", run.interface.c_str());

      // remove the internal 'use' bond for the capability dependency
      unbind_dependency(run.interface);

      // stop the dependency if no more bonds
      if (!bond_cache_.exists(run.interface))
      {
        stop_capability(run.interface);
      }
    }

    // remove the runner
    // this will implicitly stop the runner
    runner_cache_.remove_runner(capability);

    // log
    RCLCPP_INFO(node_logging_interface_ptr_->get_logger(), "stopped capability: %s", capability.c_str());
  }

  void free_capability(const std::string& capability, const std::string& bond_id)
  {
    // remove bond from cache for capability
    bond_cache_.remove_bond(capability, bond_id);

    // stop the capability if no more bonds
    if (!bond_cache_.exists(capability))
    {
      // stop the capability
      RCLCPP_INFO(node_logging_interface_ptr_->get_logger(), "stopping freed capability: %s", capability.c_str());
      stop_capability(capability);
    }
  }

  void use_capability(rclcpp::Node::SharedPtr node, const std::string& capability, const std::string& provider,
                      const std::string& bond_id)
  {
    // add bond to cache for capability
    bond_cache_.add_bond(capability, bond_id);

    // start the capability with the provider
    start_capability(node, capability, provider);
  }

  // capability api
  void add_capability(const capabilities2_msgs::msg::CapabilitySpec& spec)
  {
    // peak at the spec header
    models::header_model_t header;
    header.from_yaml(YAML::Load(spec.content));
    header.name = spec.package + "/" + header.name;

    // check type and exists, create model and add to db
    if (spec.type == capabilities2_msgs::msg::CapabilitySpec::CAPABILITY_INTERFACE)
    {
      // exists guard
      if (cap_db_->exists<models::interface_model_t>(header.name))
      {
        RCLCPP_WARN(node_logging_interface_ptr_->get_logger(), "interface already exists");
        return;
      }

      // convert message to model and add to db
      models::interface_model_t model;
      model.from_yaml(YAML::Load(spec.content));
      // add package to name probably to avoid collisions (this was previous convention)
      model.header.name = spec.package + "/" + model.header.name;
      cap_db_->insert_interface(model);
      return;
    }

    if (spec.type == capabilities2_msgs::msg::CapabilitySpec::SEMANTIC_CAPABILITY_INTERFACE)
    {
      if (cap_db_->exists<models::semantic_interface_model_t>(header.name))
      {
        RCLCPP_WARN(node_logging_interface_ptr_->get_logger(), "semantic interface already exists");
        return;
      }

      models::semantic_interface_model_t model;
      model.from_yaml(YAML::Load(spec.content));
      model.header.name = spec.package + "/" + model.header.name;
      cap_db_->insert_semantic_interface(model);
      return;
    }

    if (spec.type == capabilities2_msgs::msg::CapabilitySpec::CAPABILITY_PROVIDER)
    {
      if (cap_db_->exists<models::provider_model_t>(header.name))
      {
        RCLCPP_WARN(node_logging_interface_ptr_->get_logger(), "provider already exists");
        return;
      }

      models::provider_model_t model;
      model.from_yaml(YAML::Load(spec.content));
      model.header.name = spec.package + "/" + model.header.name;
      cap_db_->insert_provider(model);
      return;
    }

    // couldn't parse unknown capability type
    RCLCPP_ERROR(node_logging_interface_ptr_->get_logger(), "unknown capability type: %s", spec.type.c_str());
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

    // if not include_semantic, remove providers for semantic interfaces
    if (!include_semantic)
    {
      // TODO: implement
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
      // package name is first part of name separated by /
      msg.package = interface.header.name.substr(0, interface.header.name.find_first_of('/'));
      msg.type = interface.header.type;
      // get spec convert to yaml then to string
      YAML::Node spec = interface.to_yaml();
      // fix name to remove package name
      spec["name"] = spec["name"].as<std::string>().substr(interface.header.name.find_first_of('/') + 1);
      msg.content = YAML::Dump(spec);
    }

    // semantic interfaces
    models::semantic_interface_model_t semantic_interface = cap_db_->get_semantic_interface(resource);

    if (!semantic_interface.header.name.empty())
    {
      msg.package = semantic_interface.header.name.substr(0, semantic_interface.header.name.find_first_of('/'));
      msg.type = semantic_interface.header.type;
      YAML::Node spec = semantic_interface.to_yaml();
      spec["name"] = spec["name"].as<std::string>().substr(semantic_interface.header.name.find_first_of('/') + 1);
      msg.content = YAML::Dump(spec);
    }

    // providers
    models::provider_model_t provider = cap_db_->get_provider(resource);

    if (!provider.header.name.empty())
    {
      msg.package = provider.header.name.substr(0, provider.header.name.find_first_of('/'));
      msg.type = provider.header.type;
      YAML::Node spec = provider.to_yaml();
      spec["name"] = spec["name"].as<std::string>().substr(provider.header.name.find_first_of('/') + 1);
      msg.content = YAML::Dump(spec);
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
      msg.package = interface.header.name.substr(0, interface.header.name.find_first_of('/'));
      msg.type = interface.header.type;
      YAML::Node spec = interface.to_yaml();
      spec["name"] = spec["name"].as<std::string>().substr(interface.header.name.find_first_of('/') + 1);
      msg.content = YAML::Dump(spec);
      specs.push_back(msg);
    }

    // semantic interfaces
    for (const auto& semantic_interface : cap_db_->get_semantic_interfaces())
    {
      capabilities2_msgs::msg::CapabilitySpec msg;
      msg.package = semantic_interface.header.name.substr(0, semantic_interface.header.name.find_first_of('/'));
      msg.type = semantic_interface.header.type;
      YAML::Node spec = semantic_interface.to_yaml();
      spec["name"] = spec["name"].as<std::string>().substr(semantic_interface.header.name.find_first_of('/') + 1);
      msg.content = YAML::Dump(spec);
      specs.push_back(msg);
    }

    // providers
    for (const auto& provider : cap_db_->get_providers())
    {
      capabilities2_msgs::msg::CapabilitySpec msg;
      msg.package = provider.header.name.substr(0, provider.header.name.find_first_of('/'));
      msg.type = provider.header.type;
      YAML::Node spec = provider.to_yaml();
      spec["name"] = spec["name"].as<std::string>().substr(provider.header.name.find_first_of('/') + 1);
      msg.content = YAML::Dump(spec);
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
    // create message container
    std::vector<capabilities2_msgs::msg::RunningCapability> running_capabilities;

    // get a list of running caps
    std::vector<std::string> caps = runner_cache_.get_running_capabilities();

    // for each cap get the running model from db
    for (const std::string& cap : caps)
    {
      // create a running cap msg
      capabilities2_msgs::msg::RunningCapability rc;

      // get cap provider from runner
      std::string provider = runner_cache_.provider(cap);

      // get running model for provider
      models::running_model_t running = cap_db_->get_running(provider);

      // fill running cap msg
      rc.capability.capability = running.interface;
      rc.capability.provider = running.provider;
      rc.started_by = runner_cache_.started_by(cap);
      rc.pid = atoi(runner_cache_.pid(cap).c_str());

      // get dependencies
      for (auto& dep : running.dependencies)
      {
        capabilities2_msgs::msg::Capability cap_msg;
        cap_msg.capability = dep.interface;
        cap_msg.provider = dep.provider;
        rc.dependent_capabilities.push_back(cap_msg);
      }

      // add running cap msg
      running_capabilities.push_back(rc);
    }

    return running_capabilities;
  }

  // event api
  // related to runner api
  const capabilities2_msgs::msg::CapabilityEvent on_capability_started(const std::string& capability)
  {
    // create event msg
    capabilities2_msgs::msg::CapabilityEvent event;
    event.header.frame_id = "capabilities";
    event.header.stamp = rclcpp::Clock().now();

    // started event
    event.type = capabilities2_msgs::msg::CapabilityEvent::LAUNCHED;

    // set cap, prov, pid
    event.capability = capability;
    event.provider = runner_cache_.provider(capability);
    event.pid = atoi(runner_cache_.pid(capability).c_str());

    return event;
  }

  const capabilities2_msgs::msg::CapabilityEvent on_capability_stopped(const std::string& capability)
  {
    // create event msg
    capabilities2_msgs::msg::CapabilityEvent event;
    event.header.frame_id = "capabilities";
    event.header.stamp = rclcpp::Clock().now();

    // terminated event
    event.type = capabilities2_msgs::msg::CapabilityEvent::STOPPED;

    // set cap, prov, pid
    event.capability = capability;
    event.provider = runner_cache_.provider(capability);
    event.pid = atoi(runner_cache_.pid(capability).c_str());

    return event;
  }

  const capabilities2_msgs::msg::CapabilityEvent on_capability_terminated(const std::string& capability)
  {
    // create event msg
    capabilities2_msgs::msg::CapabilityEvent event;
    event.header.frame_id = "capabilities";
    event.header.stamp = rclcpp::Clock().now();

    // terminated event
    event.type = capabilities2_msgs::msg::CapabilityEvent::TERMINATED;

    // set cap, prov, pid
    event.capability = capability;
    event.provider = runner_cache_.provider(capability);
    event.pid = atoi(runner_cache_.pid(capability).c_str());

    return event;
  }

  const capabilities2_msgs::msg::CapabilityEvent on_server_ready()
  {
    // create event msg
    capabilities2_msgs::msg::CapabilityEvent event;
    event.header.frame_id = "capabilities";
    event.header.stamp = rclcpp::Clock().now();

    // started event
    event.type = capabilities2_msgs::msg::CapabilityEvent::SERVER_READY;

    return event;
  }

  // bond api
  // establish bond
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
    RCLCPP_WARN(node_logging_interface_ptr_->get_logger(), "bond broken for id: %s", bond_id.c_str());

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
  // bind a dependency to an internal bond
  // this is a bond without a live external connection
  // this will help keep the capability runners scoped to a reference
  // only when all the bonds (including internal) are broken the capability is freed
  // this is specifically needed for running dependencies of a capability
  void bind_dependency(const std::string& capability)
  {
    // create a new unique bond id
    std::string bond_id = CapabilitiesAPI::gen_bond_id();

    // add the bond id to the bond cache
    bond_cache_.add_bond(capability, bond_id);

    // keep the bond id in the internal bond ids
    internal_bond_ids_.push_back(bond_id);
  }

  void unbind_dependency(const std::string& capability)
  {
    // do we have a bond id for this capability

    // get cap bonds
    std::vector<std::string> cap_bonds = bond_cache_.get_bonds(capability);

    // if any internal bonds in cap_bonds
    for (const auto& bond : cap_bonds)
    {
      auto it = std::find(internal_bond_ids_.begin(), internal_bond_ids_.end(), bond);
      if (it != internal_bond_ids_.end())
      {
        // remove just the first bond id found in cap_bonds and internal_bond_ids_
        bond_cache_.remove_bond(capability, bond);

        // delete internal bond
        internal_bond_ids_.erase(it);

        return;
      }
    }
  }

private:
  // logger
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_ptr_;

  // db
  std::unique_ptr<CapabilitiesDB> cap_db_;

  // caches
  BondCache bond_cache_;
  RunnerCache runner_cache_;

  // internal bindings
  std::vector<std::string> internal_bond_ids_;
};

}  // namespace capabilities2_server
