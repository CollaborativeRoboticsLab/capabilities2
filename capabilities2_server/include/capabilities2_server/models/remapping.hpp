#pragma once

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace capabilities2_server
{
namespace models
{

struct remapping
{
  std::string from;
  std::string to;

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node["from"] = from;
    node["to"] = to;
    return node;
  }

  bool operator==(const remapping& other) const
  {
    return from == other.from && to == other.to;
  }

  bool operator!=(const remapping& other) const
  {
    return !(*this == other);
  }
};

struct remappings
{
  std::vector<remapping> parameters;
  std::vector<remapping> topics;
  std::vector<remapping> services;
  std::vector<remapping> actions;

  void from_yaml(const YAML::Node& node)
  {
    if (node["parameters"])
    {
      for (const auto& parameter : node["parameters"])
      {
        remapping r;
        r.from = parameter["from"].as<std::string>();
        r.to = parameter["to"].as<std::string>();
        parameters.push_back(r);
      }
    }

    if (node["topics"])
    {
      for (const auto& topic : node["topics"])
      {
        remapping r;
        r.from = topic["from"].as<std::string>();
        r.to = topic["to"].as<std::string>();
        topics.push_back(r);
      }
    }

    if (node["services"])
    {
      for (const auto& service : node["services"])
      {
        remapping r;
        r.from = service["from"].as<std::string>();
        r.to = service["to"].as<std::string>();
        services.push_back(r);
      }
    }

    if (node["actions"])
    {
      for (const auto& action : node["actions"])
      {
        remapping r;
        r.from = action["from"].as<std::string>();
        r.to = action["to"].as<std::string>();
        actions.push_back(r);
      }
    }
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node["parameters"] = parameters;
    node["topics"] = topics;
    node["services"] = services;
    node["actions"] = actions;
    return node;
  }
};

}  // namespace models
}  // namespace capabilities2_server
