#pragma once

#include <thread>
#include <string>
#include <map>
#include <capabilities2_runner/runner_base.hpp>

namespace capabilities2_runner
{
class MultiplexBaseRunner : public RunnerBase
{
public:
  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  MultiplexBaseRunner() : RunnerBase()
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
  }

  /**
   * @brief Trigger process to be executed.
   *
   * @param id unique identifier for the execution
   */
  virtual void execution(int id)
  {
    info_("execution started for id: " + std::to_string(id));

    // check if the id is already completed
    if (completed_executions.find(id) != completed_executions.end() && completed_executions[id])
    {
      info_("execution already completed for id: " + std::to_string(id));
      return;
    }
    else
    {
      // trigger the events related to on_success state
      if (events[id].on_success.interface != "")
      {
        event_(EventType::SUCCEEDED, id, events[id].on_success.interface, events[id].on_success.provider);
        triggerFunction_(events[id].on_success.interface, update_on_success(events[id].on_success.parameters));
      }
      // trigger the events related to on_failure state
      else if (events[id].on_failure.interface != "")
      {
        event_(EventType::FAILED, id, events[id].on_failure.interface, events[id].on_failure.provider);
        triggerFunction_(events[id].on_failure.interface, update_on_failure(events[id].on_failure.parameters));
      }
    }

    // track the execution as completed
    completed_executions[id] = true;

    info_("multiplexing complete. Thread closing.", id);
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

protected:
  // input count tracker
  std::map<int, int> input_count_tracker;

  // expected input count
  std::map<int, int> expected_input_count;

  // completed executions
  std::map<int, bool> completed_executions;
};
}  // namespace capabilities2_runner