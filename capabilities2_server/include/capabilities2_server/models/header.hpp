#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace capabilities2_server
{
namespace models
{

/**
 * @brief the header model is a simple model for the header of a capability type
 * it includes the name, version, type, and description of the capability
 *
 */
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

  static const std::string to_sql_table()
  {
    return "name TEXT NOT NULL, version INTEGER, type TEXT NOT NULL, description TEXT";
  }

  const std::string to_sql_values() const
  {
    return "'" + name + "', " + std::to_string(version) + ", '" + type + "', '" + description + "'";
  }
};

}  // namespace models
}  // namespace capabilities2_server
