#pragma once

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace capabilities2_server
{
namespace models
{

struct remapping_model_t
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

  bool operator==(const remapping_model_t& other) const
  {
    return from == other.from && to == other.to;
  }

  bool operator!=(const remapping_model_t& other) const
  {
    return !(*this == other);
  }
};

struct remappings_model_t
{
  std::vector<remapping_model_t> parameters;
  std::vector<remapping_model_t> topics;
  std::vector<remapping_model_t> services;
  std::vector<remapping_model_t> actions;

  void from_yaml(const YAML::Node& node)
  {
    if (node["parameters"])
    {
      for (const auto& parameter : node["parameters"])
      {
        remapping_model_t r;
        r.from = parameter["from"].as<std::string>();
        r.to = parameter["to"].as<std::string>();
        parameters.push_back(r);
      }
    }

    if (node["topics"])
    {
      for (const auto& topic : node["topics"])
      {
        remapping_model_t r;
        r.from = topic["from"].as<std::string>();
        r.to = topic["to"].as<std::string>();
        topics.push_back(r);
      }
    }

    if (node["services"])
    {
      for (const auto& service : node["services"])
      {
        remapping_model_t r;
        r.from = service["from"].as<std::string>();
        r.to = service["to"].as<std::string>();
        services.push_back(r);
      }
    }

    if (node["actions"])
    {
      for (const auto& action : node["actions"])
      {
        remapping_model_t r;
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
