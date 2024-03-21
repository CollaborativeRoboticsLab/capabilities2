#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace capabilities2_server
{
namespace models
{

/**
 * @brief computation graph resource type definition
 * this could be a parameter, topic, service, action, or node
 * the name is the unique identifier for the resource within the graph for example /foo/bar
 * the type is the data type for the resource for example std_msgs/String
 * the description is a human readable description of the resource which can be used to generate documentation
 * or understand the resource in the context of the graph
 *
 */
struct resource_model_t
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
