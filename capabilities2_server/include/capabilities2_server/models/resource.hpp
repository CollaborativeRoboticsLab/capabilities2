#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace capabilities2_server
{
namespace models
{

// computation graph resource type definition
struct resource
{
  std::string name;
  std::string type;
  std::string description;

  void from_yaml(const YAML::Node& node)
  {
    name = node["name"].as<std::string>();
    type = node["type"].as<std::string>();
    description = node["description"].as<std::string>();
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node["name"] = name;
    node["type"] = type;
    node["description"] = description;
    return node;
  }
};

}  // namespace models
}  // namespace capabilities2_server
