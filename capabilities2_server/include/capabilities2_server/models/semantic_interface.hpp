#pragma once

#include <yaml-cpp/yaml.h>
#include <capabilities2_server/models/header.hpp>
#include <capabilities2_server/models/remappable_base.hpp>
#include <capabilities2_server/models/predicateable_base.hpp>
#include <capabilities2_utils/sql_safe.hpp>

namespace capabilities2_server
{
namespace models
{

// semantic interface model definition
/**
 * @brief semantic interface model
 *
 * a semantic interface redefines an interface in a semantic way
 * this helps avoid resource collisions in the computation graph
 * when using abstract capabilities such as when capabilities are shared
 * between different robot implementations
 *
 */
struct semantic_interface_model_t : public remappable_base_t, predicateable_base_t
{
  header_model_t header;
  std::string redefines;
  std::string global_namespace;
  remappings_model_t remappings;

  void from_yaml(const YAML::Node& node)
  {
    header.from_yaml(node);
    redefines = node["redefines"].as<std::string>();
    global_namespace = node["global_namespace"].as<std::string>();
    // if remappings exist
    if (node["remappings"])
    {
      remappings.from_yaml(node["remappings"]);
    }
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node = header.to_yaml();
    node["redefines"] = redefines;
    node["global_namespace"] = global_namespace;
    node["remappings"] = remappings.to_yaml();
    return node;
  };

  static const std::string to_sql_table()
  {
    // redefines is foreign key to interfaces
    return header_model_t::to_sql_table() + ", redefines TEXT NOT NULL, global_namespace TEXT, remappings TEXT, "
                                            "FOREIGN KEY(redefines) REFERENCES interfaces(name)";
  }

  std::string to_sql_values() const
  {
    return header.to_sql_values() + ", '" + redefines + "', '" + global_namespace + "', '" +
           to_sql_safe(YAML::Dump(remappings.to_yaml())) + "'";
  }
};

}  // namespace models
}  // namespace capabilities2_server
