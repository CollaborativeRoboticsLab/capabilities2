#pragma once

#include <capabilities2_runner/action_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief no trigger action runner mixin
 *
 * provides a no trigger runner implementation for the action runner
 * mix with child classes that do not require a trigger
 *
 * @tparam ActionT
 */
template <class ActionT>
class NoTriggerRunnerMixin : public ActionRunner<ActionT>
{
public:
  // delete trigger function
  virtual std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>>
  trigger(std::shared_ptr<tinyxml2::XMLElement> parameters) override = delete;
};

}  // namespace capabilities2_runner
