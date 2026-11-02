#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace capabilities2_server
{

namespace models
{

struct launch_parameter_model_t
{
  std::string key;
  std::string value;

  void from_yaml(const YAML::Node& node)
  {
    key = node["key"].as<std::string>();
    value = node["value"].as<std::string>();
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node["key"] = key;
    node["value"] = value;
    return node;
  }
};

struct readiness_endpoint_model_t
{
  std::string name;
  std::string type;
  double timeout{ 60.0 };

  void from_yaml(const YAML::Node& node)
  {
    if (node["name"] && !node["name"].IsNull())
      name = node["name"].as<std::string>();
    if (node["type"] && !node["type"].IsNull())
      type = node["type"].as<std::string>();
    if (node["timeout"] && !node["timeout"].IsNull())
      timeout = node["timeout"].as<double>();
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node["name"] = name;
    node["type"] = type;
    node["timeout"] = timeout;
    return node;
  }
};

/**
 * @brief defineable base
 *
 * store the definition of an implementation of a capability
 * the definition is stored in a character format but
 * may contain a specific behaviour language to define a provider
 * specialise the definition later when the capability is run
 *
 */
struct defineable_base_t
{
  std::string command;
  std::vector<resource_model_t> configuration_parameters;
  std::vector<launch_parameter_model_t> launch_parameters;
  std::vector<std::string> lifecycle_nodes;
  double lifecycle_timeout_sec{ 60.0 };
  readiness_endpoint_model_t action_readiness;
  readiness_endpoint_model_t service_readiness;
  readiness_endpoint_model_t topic_readiness;
  std::vector<resource_model_t> runtime__input_parameters;
  std::vector<resource_model_t> runtime__output_parameters;

  // is the definition valid
  bool valid = false;

  void from_yaml(const YAML::Node& node)
  {
    // get the definition node
    YAML::Node def_node = node["definition"];

    // if no definition exists, return
    if (!def_node)
    {
      return;
    }

    // try get the command as a string
    if (def_node["command"])
    {
      command = def_node["command"].as<std::string>();
      valid = true;
    }

    // try get the configuration parameters
    if (def_node["configuration_parameters"])
    {
      valid = true;
      for (const auto& param : def_node["configuration_parameters"])
      {
        resource_model_t r;
        r.from_yaml(param);
        configuration_parameters.push_back(r);
      }
    }

    if (def_node["launch_parameters"])
    {
      valid = true;
      for (const auto& param : def_node["launch_parameters"])
      {
        launch_parameter_model_t launch_parameter;
        launch_parameter.from_yaml(param);
        launch_parameters.push_back(launch_parameter);
      }
    }

    const auto readiness = def_node["launch_readiness"];
    if (readiness && readiness["lifecycle"])
    {
      valid = true;
      const auto lifecycle = readiness["lifecycle"];
      if (lifecycle["nodes"])
      {
        lifecycle_nodes = lifecycle["nodes"].as<std::vector<std::string>>();
      }
      if (lifecycle["timeout_sec"])
      {
        lifecycle_timeout_sec = lifecycle["timeout_sec"].as<double>();
      }
    }
    if (readiness)
    {
      if (readiness["action"] && !readiness["action"].IsNull())
        action_readiness.from_yaml(readiness["action"]);
      if (readiness["service"] && !readiness["service"].IsNull())
        service_readiness.from_yaml(readiness["service"]);
      if (readiness["topic"] && !readiness["topic"].IsNull())
        topic_readiness.from_yaml(readiness["topic"]);
      valid = true;
    }

    // get the runtime parameters
    YAML::Node runtime_node = def_node["runtime_parameters"];

    // if no runtime parameters exist, return
    if (!runtime_node)
    {
      return;
    }

    // try get the runtime input parameters
    if (runtime_node["input"])
    {
      valid = true;
      for (const auto& param : runtime_node["input"])
      {
        resource_model_t r;
        r.from_yaml(param);
        runtime__input_parameters.push_back(r);
      }
    }

    // try get the runtime output parameters
    if (runtime_node["output"])
    {
      valid = true;
      for (const auto& param : runtime_node["output"])
      {
        resource_model_t r;
        r.from_yaml(param);
        runtime__output_parameters.push_back(r);
      }
    }
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;

    // if command exists
    if (!command.empty())
    {
      node["definition"]["command"] = command;
    }

    // if configuration parameters exist
    YAML::Node config_params;
    if (!configuration_parameters.empty())
    {
      for (const auto& param : configuration_parameters)
      {
        config_params.push_back(param.to_yaml());
      }
      node["definition"]["configuration_parameters"] = config_params;
    }

    YAML::Node launch_params;
    if (!launch_parameters.empty())
    {
      for (const auto& param : launch_parameters)
      {
        launch_params.push_back(param.to_yaml());
      }
      node["definition"]["launch_parameters"] = launch_params;
    }

    if (!lifecycle_nodes.empty())
    {
      node["definition"]["launch_readiness"]["lifecycle"]["nodes"] = lifecycle_nodes;
      node["definition"]["launch_readiness"]["lifecycle"]["timeout_sec"] = lifecycle_timeout_sec;
    }
    if (!action_readiness.name.empty() || !action_readiness.type.empty())
      node["definition"]["launch_readiness"]["action"] = action_readiness.to_yaml();
    if (!service_readiness.name.empty() || !service_readiness.type.empty())
      node["definition"]["launch_readiness"]["service"] = service_readiness.to_yaml();
    if (!topic_readiness.name.empty() || !topic_readiness.type.empty())
      node["definition"]["launch_readiness"]["topic"] = topic_readiness.to_yaml();

    // if runtime input parameters exist
    YAML::Node runtime_input_params;
    if (!runtime__input_parameters.empty())
    {
      for (const auto& param : runtime__input_parameters)
      {
        runtime_input_params.push_back(param.to_yaml());
      }
      node["definition"]["runtime_parameters"]["input"] = runtime_input_params;
    }

    // if runtime output parameters exist
    YAML::Node runtime_output_params;
    if (!runtime__output_parameters.empty())
    {
      for (const auto& param : runtime__output_parameters)
      {
        runtime_output_params.push_back(param.to_yaml());
      }
      node["definition"]["runtime_parameters"]["output"] = runtime_output_params;
    }

    return node;
  }

  // is the definition valid
  bool defined() const
  {
    return valid;
  }
};

}  // namespace models
}  // namespace capabilities2_server
