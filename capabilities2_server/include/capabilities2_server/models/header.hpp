#pragma once

#include <string>
#include <yaml-cpp/yaml.h>
#include <capabilities2_server/utils/sql_safe.hpp>

namespace capabilities2_server
{
namespace models
{

/** @brief unique key for a row */
// implement hidden unique id
struct identifiable_base_t
{
  // TODO: add autoincrementing col in DB
  int id;
};

/** @brief base model for all models that can be modified */
struct modifiable_base_t
{
  // TODO: implement creation and modified logic in db handler
  std::string date_created;
  std::string date_modified;
};

/** @brief base model for all models that can be soft deleted */
struct soft_deleteable_base_t
{
  // TODO: implement soft delete logic in db handler
  bool is_deleted = false;
};

/**
 * @brief the header model is a simple model for the header of a capability type
 * it includes the name, version, type, and description of the capability
 *
 */
struct header_model_t : identifiable_base_t, soft_deleteable_base_t, modifiable_base_t
{
  std::string name;
  std::string version;
  std::string type;
  std::string description;

  void from_yaml(const YAML::Node& node)
  {
    name = node["name"].as<std::string>();
    version = node["spec_version"].as<std::string>();
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
    return "name TEXT NOT NULL PRIMARY KEY, version TEXT NOT NULL, type TEXT NOT NULL, description TEXT";
  }

  const std::string to_sql_values() const
  {
    return "'" + name + "', '" + version + "', '" + type + "', '" + to_sql_safe(description) + "'";
  }
};

}  // namespace models
}  // namespace capabilities2_server
