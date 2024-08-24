#pragma once

#include <string>

#include <capabilities2_server/models/header.hpp>
#include <capabilities2_server/models/interface.hpp>

namespace capabilities2_server
{
namespace models
{

/**
 * @brief run configuration type
 *
 * This type represents the configuration required to actually run a capability
 * this includes the name of the capability, the provider, and fully qualified
 * resource names
 *
 */
struct run_config_model_t
{
  header_model_t header;
  interface_model_t interface;
  // FIXME: this is not properly implemented yet
  std::string global_namespace;
  std::string runner;
  std::string started_by;
  std::string pid;
};

}  // namespace models
}  // namespace capabilities2_server
