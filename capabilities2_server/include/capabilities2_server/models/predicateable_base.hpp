#pragma once

#include <vector>
#include <capabilities2_server/models/predicate.hpp>

namespace capabilities2_server
{
namespace models
{

/**
 * @brief predicateable model base type
 *
 * base class for all models with relations
 * contains a vector of relations (see predicate_model_t)
 *
 */
struct predicateable_base_t
{
  std::vector<predicate_model_t> relations;

  void from_yaml(const YAML::Node& node)
  {
    if (!node["relations"])
    {
      return;
    }

    for (const auto& relation : node["relations"])
    {
      predicate_model_t predicate;
      predicate.from_yaml(relation);
      relations.push_back(predicate);
    }
  }

  const bool predicated() const
  {
    return !relations.empty();
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;
    for (const auto& relation : relations)
    {
      node.push_back(relation.to_yaml());
    }
    return node;
  }
};

}  // namespace models

}  // namespace capabilities2_server
