#pragma once

#include <string>
#include <vector>

namespace capabilities2_server
{
namespace models
{

/** */
struct capability_model_t
{
  std::string interface;
  std::string provider;
};

/**
 * @brief running capability model
 *
 * the running model describes the run-time requirements of a capability.
 * this is a recursive model that can have dependencies on other run specs.
 * it is derived from interfaces, semantic interfaces, and providers.
 *
 */
struct running_model_t
{
  std::string interface;
  std::string provider;
  std::vector<capability_model_t> dependencies;
  std::string started_by;
  std::string pid;
};

}  // namespace models
}  // namespace capabilities2_server
