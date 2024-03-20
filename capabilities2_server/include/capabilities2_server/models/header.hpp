#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace capabilities2_server
{
namespace models
{

// header type definition
struct header_model_t
{
  std::string name;
  int version;
  std::string type;
  std::string description;

  void from_yaml(const YAML::Node& node)
  {
    name = node["name"].as<std::string>();
    version = node["version"].as<int>();
    type = node["type"].as<std::string>();
    description = node["description"].as<std::string>();
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node["name"] = name;
    node["version"] = version;
    node["type"] = type;
    node["description"] = description;
    return node;
  }

  static std::string to_sql_table() const
  {
    return "name TEXT NOT NULL, version INTEGER, type TEXT NOT NULL, description TEXT";
  }

  std::string to_sql_values() const
  {
    return "'" + h.name + "', " + std::to_string(h.version) + ", '" + h.type + "', '" + h.description + "'";
  }
};

}  // namespace models
}  // namespace capabilities2_server
