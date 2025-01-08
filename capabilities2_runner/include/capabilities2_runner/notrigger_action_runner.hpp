#pragma once

#include <capabilities2_runner/action_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief no-trigger action runner
 *
 * provides a no trigger runner implementation for the action runner
 * use for child action classes that do not require a trigger
 *
 * @tparam ActionT
 */
template <typename ActionT>
class NoTriggerActionRunner : public ActionRunner<ActionT>
{
public:
  // throw on trigger function
  void trigger(tinyxml2::XMLElement* parameters) override
  {
    throw runner_exception("cannot trigger this is a no-trigger action runner");
  }

protected:

  // throw on triggerExecution function
  void execution() override
  {
    throw runner_exception("no triggerExecution() this is a no-trigger action runner");
  }

  // throw on xml conversion functions
  typename ActionT::Goal generate_goal(tinyxml2::XMLElement*) override
  {
    throw runner_exception("cannot generate goal this is a no-trigger action runner");
  }
};

}  // namespace capabilities2_runner
