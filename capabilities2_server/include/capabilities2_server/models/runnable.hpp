#pragma once

#include <string>

#include <capabilities2_server/models/header.hpp>
#include <capabilities2_server/models/defineable_base.hpp>
#include <capabilities2_server/models/predicateable_base.hpp>
#include <capabilities2_runner/runner_base.hpp>

namespace capabilities2_server
{
namespace models
{

/**
 * @brief runnable type
 *
 * This type represents a runnable instance of a capability with a specific provider 
 * description and interface. It includes the necessary information to run the capability, 
 * such as the provider, interface, trigger command and descriptions.
 *
 */
struct runnable_model_t : defineable_base_t, predicateable_base_t
{
  header_model_t provider;
  header_model_t interface;

  const bool is_valid() const
  {
    return !interface.name.empty() && !provider.name.empty();
  }

  YAML::Node to_yaml() const
  {
    YAML::Node node;

    node["provider"] = provider.to_yaml();
    node["interface"] = interface.to_yaml();
    
    if (defined())
    {
      node["definition"] = defineable_base_t::to_yaml();
    }

    if (predicated())
    {
      node["relations"] = predicateable_base_t::to_yaml();
    }

    return node;
  }
};

}  // namespace models
}  // namespace capabilities2_server
