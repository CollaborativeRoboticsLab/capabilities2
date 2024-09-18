#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace capabilities2_server
{
namespace models
{

/**
 * @brief predicate model type
 *
 * the predicate model is used to specify a relation between two capability resources
 * subject -> predicate -> object
 *
 */
struct predicate_model_t
{
  std::string subject;
  std::string predicate;
  std::string object;

  void from_yaml(const YAML::Node& node)
  {
    subject = node["subject"].as<std::string>();
    predicate = node["predicate"].as<std::string>();
    object = node["object"].as<std::string>();
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    node["subject"] = subject;
    node["predicate"] = predicate;
    node["object"] = object;
    return node;
  }
};

}  // namespace models

}  // namespace capabilities2_server
