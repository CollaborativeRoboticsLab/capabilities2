#pragma once

#include <capabilities2_runner_system/multiplex_base_runner.hpp>

namespace capabilities2_runner
{
class InputMultiplexAnyRunner : public MultiplexBaseRunner
{
public:
  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  InputMultiplexAnyRunner() : MultiplexBaseRunner()
  {
  }

  /**
   * @brief trigger function to handle multiplexing of all inputs based on ANY condition
   *
   * @param parameters not used in this runner
   */
  virtual void trigger(const std::string& parameters) override
  {
    info_("received new parameters for InputMultiplexAnyRunner : " + parameters);

    tinyxml2::XMLElement* parameters_ = convert_to_xml(parameters);

    int input_count = -1;

    parameters_->QueryIntAttribute("input_count", &input_count);
    parameters_->QueryIntAttribute("id", &runner_id);

    if (runner_id < 0 || input_count < 0)
    {
      throw runner_exception("UID or input_count not found in parameters");
    }
    else
    {
      info_("triggered with UID: " + std::to_string(runner_id) + " and input_count: " + std::to_string(input_count));
    }

    if (input_count_tracker.find(runner_id) == input_count_tracker.end())
    {
      input_count_tracker[runner_id] = 1;
      expected_input_count[runner_id] = input_count;

      info_("has started the ANY condition with " + std::to_string(input_count_tracker[runner_id]) + " inputs.");
    }
    else
    {
      input_count_tracker[runner_id] += 1;

      info_("has received " + std::to_string(input_count_tracker[runner_id]) + "/" +
            std::to_string(expected_input_count[runner_id]) + " inputs for ANY condition.");
    }

    if (input_count_tracker[runner_id] > 0)
    {
      info_("has fullfilled the ANY condition with " + std::to_string(input_count_tracker[runner_id]) + " inputs.");

      executionThreadPool[runner_id] = std::thread(&InputMultiplexAnyRunner::execution, this, runner_id);
    }
  }
};

}  // namespace capabilities2_runner
