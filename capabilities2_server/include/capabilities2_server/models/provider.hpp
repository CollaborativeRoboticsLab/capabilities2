#pragma once

#include <map>
#include <yaml-cpp/yaml.h>
#include <capabilities2_server/models/header.hpp>
#include <capabilities2_server/models/remappable_base.hpp>

namespace capabilities2_server
{
namespace models
{

// provider model definition
struct provider_model_t : public remappable_base_t
{
  header_model_t header;
  std::string implements;
  std::map<std::string, std::string> depends_on;
  remappings_model_t remappings;
  std::string runner;

  void from_yaml(const YAML::Node& node)
  {
    header.from_yaml(node);
    implements = node["implements"].as<std::string>();
    for (const auto& dependency : node["depends_on"])
    {
      depends_on[dependency.first.as<std::string>()] = dependency.second.as<std::string>();
    }
    remappings.from_yaml(node["remappings"]);
    runner = node["runner"].as<std::string>();
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node = header.to_yaml();
    node["implements"] = implements;
    node["depends_on"] = depends_on;
    node["remappings"] = remappings.to_yaml();
    node["runner"] = runner;
    return node;
  }

  static const std::string to_sql_table()
  {
    return header_model_t::to_sql_table() + ", implements TEXT NOT NULL, depends_on TEXT, remappings TEXT, runner TEXT";
  }

  const std::string to_sql_values() const
  {
    return header.to_sql_values() + ", '" + implements + "', '" + YAML::Dump(remappings.to_yaml()) + "', '" + runner +
           "'";
  }
};

}  // namespace models
}  // namespace capabilities2_server
