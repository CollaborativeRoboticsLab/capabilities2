#pragma once

#include <map>
#include <yaml-cpp/yaml.h>
#include <capabilities2_server/models/header.hpp>
#include <capabilities2_server/models/resource.hpp>

namespace capabilities2_server
{
namespace models
{

// interface specification type
struct specification
{
  std::map<std::string, resource> parameters;
  std::map<std::string, resource> topics;
  std::map<std::string, resource> services;
  std::map<std::string, resource> actions;

  void from_yaml(const YAML::Node& node)
  {
    if (node["parameters"])
    {
      for (const auto& parameter : node["parameters"])
      {
        resource r;
        r.name = parameter.first.as<std::string>();
        r.type = parameter.second["type"].as<std::string>();
        r.description = parameter.second["description"].as<std::string>();
        parameters[r.name] = r;
      }
    }
  }
  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node["parameters"] = parameters;
    node["topics"] = topics;
    node["services"] = services;
    node["actions"] = actions;
    return node;
  }
}

// interface model definition
struct interface
{
  header header;
  specification interface;

  void from_yaml(const YAML::Node& node)
  {
    header.from_yaml(node["header"]);
    interface.from_yaml(node["interface"]);
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node["header"] = header.to_yaml();
    node["interface"] = interface.to_yaml();
    return node;
  }

  static std::string to_sql_table() const
  {
    return header.to_sql_table() + ", interface TEXT";
  }

  std::string to_sql_values() const
  {
    return header.to_sql_values() + ", '" + interface.to_yaml().to_string() + "'";
  }
};

}  // namespace models
}  // namespace capabilities2_server
