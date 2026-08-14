#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace capabilities2_server
{

namespace models
{

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
      for (const auto& param : def_node["configuration_parameters"])
      {
        resource_model_t r;
        r.from_yaml(param);
        configuration_parameters.push_back(r);
      }
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
