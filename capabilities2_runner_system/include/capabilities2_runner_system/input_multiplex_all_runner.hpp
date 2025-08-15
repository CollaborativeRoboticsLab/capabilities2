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

    int uid = 0;
    int input_count = 0;

    parameters_->QueryIntAttribute("input_count", &input_count);
    parameters_->QueryIntAttribute("uid", &uid);

    if (input_count_tracker.find(uid) == input_count_tracker.end())
    {
      input_count_tracker[uid] = 1;
      expected_input_count[uid] = input_count;

      info_("has started the All condition with " + std::to_string(input_count_tracker[uid]) + " inputs.");
    }
    else
    {
      input_count_tracker[uid] += 1;

      info_("has received " + std::to_string(input_count_tracker[uid]) + "/" +
            std::to_string(expected_input_count[uid]) + " inputs for ALL condition.");
    }

    if (input_count_tracker[uid] == expected_input_count[uid])
    {
      info_("has fullfilled the All condition with " + std::to_string(input_count_tracker[uid]) + " inputs.");

      executionThreadPool[uid] = std::thread(&InputMultiplexAllRunner::execution, this, uid);
    }
  }

};

}  // namespace capabilities2_runner
