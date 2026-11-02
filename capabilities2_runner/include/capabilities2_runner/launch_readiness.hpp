#pragma once

#include <string>
#include <vector>

namespace capabilities2_runner
{
struct readiness_endpoint
{
  std::string name;
  std::string type;
  double timeout{ 60.0 };
};

struct launch_readiness_config
{
  std::vector<std::string> lifecycle_nodes;
  double lifecycle_timeout_sec{ 60.0 };
  readiness_endpoint action;
  readiness_endpoint service;
  readiness_endpoint topic;
};
}  // namespace capabilities2_runner