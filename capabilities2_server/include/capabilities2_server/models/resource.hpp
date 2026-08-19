#pragma once

#include <string>
#include <vector>

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
  std::string semantic_key;
  bool required = false;
  std::vector<std::string> satisfiable_from;
  std::string fallback_parameter;
  std::vector<std::string> aliases;
  bool has_default = false;
  std::string default_value;

  void from_yaml(const YAML::Node& node)
  {
    name = node["name"].as<std::string>();
    type = node["type"].as<std::string>();
    description = node["description"].as<std::string>();

    if (node["semantic_key"])
    {
      semantic_key = node["semantic_key"].as<std::string>();
    }

    if (node["required"])
    {
      required = node["required"].as<bool>();
    }

    if (node["satisfiable_from"] && node["satisfiable_from"].IsSequence())
    {
      satisfiable_from.clear();
      for (const auto& item : node["satisfiable_from"])
      {
        satisfiable_from.push_back(item.as<std::string>());
      }
    }

    if (node["fallback_parameter"])
    {
      fallback_parameter = node["fallback_parameter"].as<std::string>();
    }

    if (node["aliases"] && node["aliases"].IsSequence())
    {
      aliases.clear();
      for (const auto& item : node["aliases"])
      {
        aliases.push_back(item.as<std::string>());
      }
    }

    if (node["default"])
    {
      has_default = true;
      default_value = YAML::Dump(node["default"]);

      if (!default_value.empty() && default_value.back() == '\n')
      {
        default_value.pop_back();
      }
    }
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node["name"] = name;
    node["type"] = type;
    node["description"] = description;

    if (!semantic_key.empty())
    {
      node["semantic_key"] = semantic_key;
    }

    if (required)
    {
      node["required"] = required;
    }

    if (!satisfiable_from.empty())
    {
      YAML::Node satisfiable_from_node;
      for (const auto& source : satisfiable_from)
      {
        satisfiable_from_node.push_back(source);
      }
      node["satisfiable_from"] = satisfiable_from_node;
    }

    if (!fallback_parameter.empty())
    {
      node["fallback_parameter"] = fallback_parameter;
    }

    if (!aliases.empty())
    {
      YAML::Node aliases_node;
      for (const auto& alias : aliases)
      {
        aliases_node.push_back(alias);
      }
      node["aliases"] = aliases_node;
    }

    if (has_default)
    {
      node["default"] = YAML::Load(default_value);
    }

    return node;
  }
};

}  // namespace models
}  // namespace capabilities2_server
