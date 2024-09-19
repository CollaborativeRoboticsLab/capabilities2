#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace capabilities2_server
{
namespace models
{

/** */
// TODO: implement hidden unique id
// struct identifiable_base_t
// {
//   int id;
// };

/** @brief base model for all models that can be modified */
struct modifiable_base_t
{
  std::string date_created;
  std::string date_modified;
};

/** @brief base model for all models that can be soft deleted */
struct soft_deleteable_base_t
{
  bool is_deleted = false;
};

/**
 * @brief the header model is a simple model for the header of a capability type
 * it includes the name, version, type, and description of the capability
 *
 */
struct header_model_t : soft_deleteable_base_t, modifiable_base_t
{
  std::string name;
  int version;
  std::string type;
  std::string description;

  void from_yaml(const YAML::Node& node)
  {
    name = node["name"].as<std::string>();
    version = node["spec_version"].as<int>();
    type = node["spec_type"].as<std::string>();
    description = node["description"].as<std::string>();
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node["name"] = name;
    node["spec_version"] = version;
    node["spec_type"] = type;
    node["description"] = description;
    return node;
  }

  static const std::string to_sql_table()
  {
    // name, version, type, description
    // name is the primary key
    return "name TEXT NOT NULL PRIMARY KEY, version INTEGER, type TEXT NOT NULL, description TEXT";
  }

  const std::string to_sql_values() const
  {
    return "'" + name + "', " + std::to_string(version) + ", '" + type + "', '" + description + "'";
  }
};

}  // namespace models
}  // namespace capabilities2_server
