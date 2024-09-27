#pragma once

#include <string>

#include <capabilities2_server/models/header.hpp>
#include <capabilities2_server/models/interface.hpp>
#include <capabilities2_server/models/defineable_base.hpp>
#include <capabilities2_runner/runner_base.hpp>

namespace capabilities2_server
{
namespace models
{

/**
 * @brief run configuration type
 *
 * This type represents the configuration required to actually run a capability
 * this includes the name of the capability, the provider, and fully qualified
 * resource names
 *
 */
struct run_config_model_t : defineable_base_t
{
  header_model_t provider;
  interface_model_t interface;
  std::string global_namespace;
  std::string runner;
  std::string started_by;
  std::string pid;

  const capabilities2_runner::runner_opts to_runner_opts() const
  {
    capabilities2_runner::runner_opts opts;

    opts.interface = interface.header.name;
    opts.provider = provider.name;
    opts.global_namespace = global_namespace;
    opts.runner = runner;
    opts.started_by = started_by;
    opts.pid = pid;

    // lambda for pushing resources by type
    auto push_by_type = [&](const std::map<std::string, resource_model_t>& resources, const std::string& type) {
      for (const auto& resource : resources)
      {
        opts.resources.push_back({ resource.second.name, type, resource.second.type });
      }
    };

    // get all resources
    push_by_type(interface.interface.parameters, "parameter");
    push_by_type(interface.interface.topics, "topic");
    push_by_type(interface.interface.services, "service");
    push_by_type(interface.interface.actions, "action");

    return opts;
  }

  const bool is_valid() const
  {
    return !interface.header.name.empty() && !provider.name.empty();
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;

    node["provider"] = provider.to_yaml();
    node["interface"] = interface.to_yaml();
    if (defined())
    {
      node["definition"] = definition_str;
    }
    node["global_namespace"] = global_namespace;
    node["runner"] = runner;
    node["started_by"] = started_by;
    node["pid"] = pid;

    return node;
  }
};

}  // namespace models
}  // namespace capabilities2_server
