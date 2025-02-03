#pragma once

#include <map>
#include <yaml-cpp/yaml.h>
#include <capabilities2_server/models/header.hpp>
#include <capabilities2_server/models/resource.hpp>
#include <capabilities2_server/models/predicateable_base.hpp>
#include <capabilities2_server/utils/sql_safe.hpp>

namespace capabilities2_server
{
namespace models
{

/**
 * @brief interface specification type
 * the specification model is the computation graph blueprint of a capability
 *
 */
struct specification_model_t
{
  std::map<std::string, resource_model_t> parameters;
  std::map<std::string, resource_model_t> topics;
  std::map<std::string, resource_model_t> services;
  std::map<std::string, resource_model_t> actions;

  void from_yaml(const YAML::Node& node)
  {
    if (node["parameters"])
    {
      for (const auto& parameter : node["parameters"])
      {
        resource_model_t r;
        r.name = parameter.first.as<std::string>();
        r.type = parameter.second["type"].as<std::string>();
        r.description = parameter.second["description"].as<std::string>();
        parameters[r.name] = r;
      }
    }
    if (node["topics"])
    {
      for (const auto& topic : node["topics"])
      {
        resource_model_t r;
        r.name = topic.first.as<std::string>();
        r.type = topic.second["type"].as<std::string>();
        r.description = topic.second["description"].as<std::string>();
        topics[r.name] = r;
      }
    }
    if (node["services"])
    {
      for (const auto& service : node["services"])
      {
        resource_model_t r;
        r.name = service.first.as<std::string>();
        r.type = service.second["type"].as<std::string>();
        r.description = service.second["description"].as<std::string>();
        services[r.name] = r;
      }
    }
    if (node["actions"])
    {
      for (const auto& action : node["actions"])
      {
        resource_model_t r;
        r.name = action.first.as<std::string>();
        r.type = action.second["type"].as<std::string>();
        r.description = action.second["description"].as<std::string>();
        actions[r.name] = r;
      }
    }
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;

    // parameters
    YAML::Node p;
    for (const auto& parameter : parameters)
    {
      p[parameter.first] = parameter.second.to_yaml();
    }
    node["parameters"] = p;

    // topics
    YAML::Node t;
    for (const auto& topic : topics)
    {
      t[topic.first] = topic.second.to_yaml();
    }
    node["topics"] = t;

    // services
    YAML::Node s;
    for (const auto& service : services)
    {
      s[service.first] = service.second.to_yaml();
    }
    node["services"] = s;

    // actions
    YAML::Node a;
    for (const auto& action : actions)
    {
      a[action.first] = action.second.to_yaml();
    }
    node["actions"] = a;

    // return node
    return node;
  }
};

/**
 * @brief interface model
 *
 * the main representation of a capability
 * mapping skill-like concepts to ROS-resources that usually make up a logical robot subsystem
 *
 */
struct interface_model_t : public predicateable_base_t
{
  header_model_t header;
  specification_model_t interface;

  void from_yaml(const YAML::Node& node)
  {
    header.from_yaml(node);
    interface.from_yaml(node["interface"]);
    // if relations exist
    if (node["relations"])
    {
      for (const auto& relation : node["relations"])
      {
        predicate_model_t p;
        p.from_yaml(relation);
        relations.push_back(p);
      }
    }
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node = header.to_yaml();
    node["interface"] = interface.to_yaml();
    // if relations exist
    if (relations.size() > 0)
    {
      YAML::Node r;
      for (const auto& relation : relations)
      {
        r.push_back(relation.to_yaml());
      }
      node["relations"] = r;
    }
    return node;
  }

  static const std::string to_sql_table()
  {
    return header_model_t::to_sql_table() + ", interface TEXT";
  }

  const std::string to_sql_values() const
  {
    return header.to_sql_values() + ", '" + to_sql_safe(YAML::Dump(interface.to_yaml())) + "'";
  }
};

}  // namespace models
}  // namespace capabilities2_server
