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
  // the definition stored as a string
  std::string definition_str = "";  // optional property
  // is the definition valid
  bool valid = false;

  void from_yaml(const YAML::Node& node)
  {
    // try get the definition as a string
    if (node["definition"])
    {
      definition_str = node["definition"].as<std::string>();
      valid = true;
    }
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node["definition"] = definition_str;
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
