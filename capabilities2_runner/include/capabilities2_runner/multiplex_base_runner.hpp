#pragma once

#include <capabilities2_runner/threadtrigger_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief Multiplex Base Runner
 *
 * Base class for inter-runner connections that require multiplexing of inputs
 *
 */
class MultiplexBaseRunner : public ThreadTriggerRunner
{
public:
  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  MultiplexBaseRunner() : ThreadTriggerRunner()
  {
  }

  /**
   * @brief Starter function for starting the action runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& bond_id) override
  {
    init_base(node, run_config);

    // emit started event
    emit_started(bond_id, "");
  }

  /**
   * @brief stop function to cease functionality and shutdown
   *
   */
  virtual void stop(const std::string& bond_id) override
  {
    // if the node pointer is empty then throw an error
    // this means that the runner was not started and is being used out of order

    if (!node_)
      throw runner_exception("cannot stop runner that was not started");

    // emit stopped event
    emit_stopped(bond_id, update_on_stopped(events[runner_id].on_stopped.parameters));

    RCLCPP_INFO(node_->get_logger(), "stopping runner");
  }

protected:
  /**
   * @brief Trigger process to be executed.
   *
   * @param id unique identifier for the execution
   */
  virtual void execution(const std::string& parameters, const std::string& thread_id) override
  {
    // extract trigger_id from thread_id (format: "bond_id/trigger_id")
    size_t slash_pos = thread_id.find('/');
    std::string trigger_id = (slash_pos != std::string::npos) ? thread_id.substr(slash_pos + 1) : "";

    RCLCPP_INFO(node_->get_logger(), "execution started for trigger_id: %s", trigger_id.c_str());

    // check if the id is already completed
    if (completed_executions.find(trigger_id) != completed_executions.end() && completed_executions[trigger_id])
    {
      RCLCPP_INFO(node_->get_logger(), "execution already completed for trigger_id: %s", trigger_id.c_str());
      return;
    }
    else
    {
      // emit events based on the execution result
      // TODO: determine execution result and emit appropriate events, currently emits success for demonstration
      // used to emit success and failure?
    }

    // track the execution as completed
    completed_executions[trigger_id] = true;

    RCLCPP_INFO(node_->get_logger(), "multiplexing complete. Thread closing for trigger_id: %s", trigger_id.c_str());
  }

protected:
  // input count tracker
  std::map<int, int> input_count_tracker;

  // expected input count
  std::map<int, int> expected_input_count;

  // completed executions
  std::map<std::string, bool> completed_executions;
};

}  // namespace capabilities2_runner
