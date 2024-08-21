#pragma once

#include <capabilities2_server/models/remapping.hpp>

namespace capabilities2_server
{
namespace models
{

/**
 * @brief remappable base model definition
 *
 * base class for remappable models to inherit from
 *
 */
struct remappable_base_t
{
  remappings_model_t remappings;
};

}  // namespace models
}  // namespace capabilities2_server
