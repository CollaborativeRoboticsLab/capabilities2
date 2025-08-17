#pragma once

#include <capabilities2_runner_system/multiplex_base_runner.hpp>

namespace capabilities2_runner
{
class InputMultiplexAllRunner : public MultiplexBaseRunner
{
public:
  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  InputMultiplexAllRunner() : MultiplexBaseRunner()
  {
  }

  /**
   * @brief trigger function to handle multiplexing of all inputs based on ALL condition
   *
   * @param parameters not used in this runner
   */
  virtual void trigger(const std::string& parameters) override
  {
    tinyxml2::XMLElement* parameters_ = convert_to_xml(parameters);

    int input_count = 0;

    parameters_->QueryIntAttribute("input_count", &input_count);
    parameters_->QueryIntAttribute("id", &runner_id);

    if (input_count_tracker.find(runner_id) == input_count_tracker.end())
    {
      input_count_tracker[runner_id] = 1;
      expected_input_count[runner_id] = input_count;

      info_("has started the All condition with " + std::to_string(input_count_tracker[runner_id]) + " inputs.");
    }
    else
    {
      input_count_tracker[runner_id] += 1;

      info_("has received " + std::to_string(input_count_tracker[runner_id]) + "/" +
            std::to_string(expected_input_count[runner_id]) + " inputs for ALL condition.");
    }

    if (input_count_tracker[runner_id] == expected_input_count[runner_id])
    {
      info_("has fullfilled the All condition with " + std::to_string(input_count_tracker[runner_id]) + " inputs.");

      executionThreadPool[runner_id] = std::thread(&InputMultiplexAllRunner::execution, this, runner_id);
    }
  }
};

}  // namespace capabilities2_runner
