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
};

}  // namespace models

}  // namespace capabilities2_server
