#pragma once

#include <capabilities2_msgs/msg/CapabilitySpec.hpp>

namespace capabilities2_server
{
class CapabilitiesDB
{
public:
  CapabilitiesDB();
  ~CapabilitiesDB();

  void load_capability_models();
  void get_capability_model();
};

}  // namespace capabilities2_server
