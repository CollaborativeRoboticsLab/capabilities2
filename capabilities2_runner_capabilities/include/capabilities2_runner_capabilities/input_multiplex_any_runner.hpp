#pragma once

#include <thread>
#include <capabilities2_runner/runner_base.hpp>

namespace capabilities2_runner
{
class InputMultiplexAnyRunner : public RunnerBase
{
public:
  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  InputMultiplexAnyRunner() : RunnerBase()
  {
  }

  /**
   * @brief Starter function for starting the action runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config) override
  {
    init_base(node, run_config);

    info_("Starting InputMultiplexAnyRunner with " + std::to_string(run_config.input_count) + " inputs.");

    expected_inputs_ = run_config.input_count;

    current_inputs_ = 0;
    not_triggered_ = true;
  }

  virtual void trigger(const std::string& parameters) override
  {
    current_inputs_ += 1;

    if (not_triggered_)
      not_triggered_ = false;

    if (current_inputs_ >= 0)
    {
      info_("InputMultiplexAnyRunner is has fullfilled the ANY condition with " + std::to_string(current_inputs_) +
            " inputs.");

      executionThread = std::thread(&InputMultiplexAnyRunner::execution, this, thread_id);
      thread_id += 1;
    }
    else
    {
      info_("InputMultiplexAnyRunner waiting. Only got " + std::to_string(current_inputs_ + 1) + "/" +
            std::to_string(expected_inputs_) + " inputs.");
    }
  }

  virtual void execution(int id)
  {
    // trigger the events related to on_success state
    if (events[execute_id].on_success.interface != "")
    {
      event_(EventType::SUCCEEDED, id, events[execute_id].on_success.interface, events[execute_id].on_success.provider);
      triggerFunction_(events[execute_id].on_success.interface,
                       update_on_success(events[execute_id].on_success.parameters));
    }
    // trigger the events related to on_failure state
    else if (events[execute_id].on_failure.interface != "")
    {
      event_(EventType::FAILED, id, events[execute_id].on_failure.interface, events[execute_id].on_failure.provider);
      triggerFunction_(events[execute_id].on_failure.interface,
                       update_on_failure(events[execute_id].on_failure.parameters));
    }
  }

  /**
   * @brief stop function to cease functionality and shutdown
   *
   */
  virtual void stop() override
  {
    // if the node pointer is empty then throw an error
    // this means that the runner was not started and is being used out of order

    if (!node_)
      throw runner_exception("cannot stop runner that was not started");

    info_("stopping runner");
  }

  /**
   * @brief Destructor
   *
   * Cleans up the thread if it is still running
   */
  ~InputMultiplexAnyRunner();

private:
  bool not_triggered_;

  int expected_inputs_;

  int current_inputs_;

  std::thread executionThread;
};

}  // namespace capabilities2_runner
