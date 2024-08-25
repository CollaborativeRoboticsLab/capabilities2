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

  void from_yaml_detail_iterator_value(const YAML::detail::iterator_value& iter)
  {
    // from is yaml key, to is yaml value
    from = iter.first.as<std::string>();
    to = iter.second.as<std::string>();
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node[from] = to;
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
        r.from_yaml_detail_iterator_value(parameter);
        parameters.push_back(r);
      }
    }

    if (node["topics"])
    {
      for (const auto& topic : node["topics"])
      {
        remapping_model_t r;
        r.from_yaml_detail_iterator_value(topic);
        topics.push_back(r);
      }
    }

    if (node["services"])
    {
      for (const auto& service : node["services"])
      {
        remapping_model_t r;
        r.from_yaml_detail_iterator_value(service);
        services.push_back(r);
      }
    }

    if (node["actions"])
    {
      for (const auto& action : node["actions"])
      {
        remapping_model_t r;
        r.from_yaml_detail_iterator_value(action);
        actions.push_back(r);
      }
    }
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;

    // parameters
    for (const auto& parameter : parameters)
    {
      node["parameters"][parameter.from] = parameter.to;
    }

    // topics
    YAML::Node t;
    for (const auto& topic : topics)
    {
      node["topics"][topic.from] = topic.to;
    }

    // services
    YAML::Node s;
    for (const auto& service : services)
    {
      node["services"][service.from] = service.to;
    }

    // actions
    YAML::Node a;
    for (const auto& action : actions)
    {
      node["actions"][action.from] = action.to;
    }

    // return node
    return node;
  }
};

}  // namespace models
}  // namespace capabilities2_server
