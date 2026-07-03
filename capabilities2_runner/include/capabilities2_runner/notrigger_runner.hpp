#pragma once

#include <capabilities2_runner/runner_base.hpp>

namespace capabilities2_runner
{

/**
 * @brief no-trigger runner
 *
 * provides a no trigger runner implementation for the runner
 * use for child runner classes that do not require a trigger
 *
 */
class NoTriggerRunner : public RunnerBase
{
public:
  // throw on trigger function
  void trigger(capabilities2_events::EventParameters& parameters, const std::string& bond_id,
               const std::string& instance_id = "") override
  {
    // emit failed event
    emit_failed(bond_id, instance_id, param_on_failure());

    throw runner_exception("cannot trigger this is a no-trigger runner");
  }
};

}  // namespace capabilities2_runner
