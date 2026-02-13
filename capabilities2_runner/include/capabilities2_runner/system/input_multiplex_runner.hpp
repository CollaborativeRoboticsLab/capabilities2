#pragma once

#include <capabilities2_runner/threadtrigger_runner.hpp>

namespace capabilities2_runner
{
class InputMultiplexRunner : public ThreadTriggerRunner
{
public:
  /**
   * @brief Constructor which needs to be empty due to plugin semantics
   */
  InputMultiplexRunner() : ThreadTriggerRunner()
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
    emit_started(bond_id, param_on_started());
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
    emit_stopped(bond_id, param_on_stopped());

    RCLCPP_INFO(node_->get_logger(), "stopping runner");
  }

protected:
  /**
   * @brief Trigger process to be executed.
   *
   * @param id unique identifier for the execution
   */
  virtual void execution(capabilities2_events::EventParameters parameters, const std::string& thread_id) override
  {
    // split thread_id to get bond_id and trigger_id (format: "bond_id/trigger_id")
    std::string bond_id = ThreadTriggerRunner::bond_from_thread_id(thread_id);
    std::string trigger_id = ThreadTriggerRunner::trigger_from_thread_id(thread_id);

    int input_count = 1;
    int multiplex_id = 0;

    if (parameters.has_value("input_count"))
      input_count = std::any_cast<int>(parameters.get_value("input_count"));
    else
      RCLCPP_WARN(node_->get_logger(), "No 'input_count' parameter found in event parameters. Defaulting to 1.");

    if (parameters.has_value("multiplex_id"))
      multiplex_id = std::any_cast<int>(parameters.get_value("multiplex_id"));
    else
      RCLCPP_WARN(node_->get_logger(), "No 'multiplex_id' parameter found in event parameters. Defaulting to 0.");

    // track the input count for the runner_id
    if (input_count_tracker.find(multiplex_id) == input_count_tracker.end())
    {
      input_count_tracker[multiplex_id] = 1;
      expected_input_count[multiplex_id] = input_count;
    }
    else
    {
      input_count_tracker[multiplex_id] += 1;
    }

    // check if the input count has reached the expected input count for the multiplex_id and if so execute the process
    if (input_count_tracker[multiplex_id] == expected_input_count[multiplex_id])
    {
      RCLCPP_INFO(node_->get_logger(),
                  "multiplex_id: %d has received all expected inputs. Executing process for trigger_id: %s",
                  multiplex_id, trigger_id.c_str());

      // If on_success is defined, emit success event will trigger it. If not defined, it will be a no-op.
      emit_succeeded(bond_id, param_on_success());

      RCLCPP_INFO(node_->get_logger(), "execution successful for trigger_id: %s", trigger_id.c_str());
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(),
                  "multiplex_id: %d pending expected inputs. Current count: %d/%d for trigger_id: %s", multiplex_id,
                  input_count_tracker[multiplex_id], expected_input_count[multiplex_id], trigger_id.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "multiplexing complete. Thread closing for trigger_id: %s", thread_id.c_str());
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
