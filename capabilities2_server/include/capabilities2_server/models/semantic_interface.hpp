#pragma once

#include <yaml-cpp/yaml.h>
#include <capabilities2_server/models/header.hpp>
#include <capabilities2_server/models/remapping.hpp>

namespace capabilities2_server
{
namespace models
{

// semantic interface model definition
struct semantic_interface_model_t
{
  header_model_t header;
  std::string redefines;
  std::string global_namespace;
  remappings_model_t remappings;

  void from_yaml(const YAML::Node& node)
  {
    header.from_yaml(node["header"]);
    redefines = node["redefines"].as<std::string>();
    global_namespace = node["global_namespace"].as<std::string>();
    remappings.from_yaml(node["remappings"]);
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node["header"] = header.to_yaml();
    node["redefines"] = redefines;
    node["global_namespace"] = global_namespace;
    node["remappings"] = remappings.to_yaml();
    return node;
  };

  static const std::string to_sql_table()
  {
    return header_model_t::to_sql_table() + ", redefines TEXT NOT NULL, global_namespace TEXT, remappings TEXT";
  }

  std::string to_sql_values() const
  {
    return header.to_sql_values() + ", '" + redefines + "', '" + global_namespace + "', '" +
           YAML::Dump(remappings.to_yaml()) + "'";
  }
};

}  // namespace models
}  // namespace capabilities2_server
