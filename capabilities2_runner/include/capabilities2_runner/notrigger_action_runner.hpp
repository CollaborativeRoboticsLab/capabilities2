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
  std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>>
  trigger(std::shared_ptr<tinyxml2::XMLElement> parameters) override
  {
    throw runner_exception("cannot trigger this is a no-trigger action runner");
  }

protected:
  // throw on xml conversion functions
  typename ActionT::Goal generate_goal(std::shared_ptr<tinyxml2::XMLElement>) override
  {
    throw runner_exception("cannot generate goal this is a no-trigger action runner");
  }

  std::shared_ptr<tinyxml2::XMLElement> generate_result(const typename ActionT::Result::SharedPtr& result) override
  {
    throw runner_exception("cannot generate result this is a no-trigger action runner");
  }
};

}  // namespace capabilities2_runner
