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
    emit_started(bond_id, "", param_on_started());
  }

  /**
   * @brief stop function to cease functionality and shutdown
   *
   */
  virtual void stop(const std::string& bond_id, const std::string& instance_id = "") override
  {
    // if the node pointer is empty then throw an error
    // this means that the runner was not started and is being used out of order

    if (!node_)
      throw runner_exception("cannot stop runner that was not started");

    // emit stopped event
    emit_stopped(bond_id, instance_id, param_on_stopped());

    RCLCPP_INFO(node_->get_logger(), "stopping runner");
  }

protected:
  /**
   * @brief Trigger process to be executed.
   *
   * @param parameters pointer to tinyxml2::XMLElement that contains parameters
   * @param thread_id unique identifier for the execution thread
   */
  virtual void execution(capabilities2_events::EventParameters parameters, const std::string& thread_id) override
  {
    // split thread_id to get bond_id and instance_id (format: "bond_id/instance_id")
    std::string bond_id = ThreadTriggerRunner::bond_from_thread_id(thread_id);
    std::string instance_id = ThreadTriggerRunner::instance_from_thread_id(thread_id);

    int input_count = std::any_cast<int>(parameters.get_value("input_count", 1));

    // track the input count for the runner_id
    if (input_count_tracker.find(instance_id) == input_count_tracker.end())
    {
      input_count_tracker[instance_id] = 1;
      expected_input_count[instance_id] = input_count;
    }
    else
    {
      input_count_tracker[instance_id] += 1;
    }

    // check if the input count has reached the expected input count for the instance_id and if so execute the process
    if (input_count_tracker[instance_id] == expected_input_count[instance_id])
    {
      RCLCPP_INFO(node_->get_logger(),
                  "instance_id: %s has received all expected inputs. Executing process for instance_id: %s",
                  instance_id.c_str(), instance_id.c_str());

      // If on_success is defined, emit success event will trigger it. If not defined, it will be a no-op.
      emit_succeeded(bond_id, instance_id, param_on_success());

      RCLCPP_INFO(node_->get_logger(), "execution successful for instance_id: %s", instance_id.c_str());
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(),
                  "instance_id: %s pending expected inputs. Current count: %d/%d for instance_id: %s", instance_id.c_str(),
                  input_count_tracker[instance_id], expected_input_count[instance_id], instance_id.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "multiplexing complete. Thread closing for instance_id: %s", instance_id.c_str());
  }

protected:
  // input count tracker
  std::map<std::string, int> input_count_tracker;

  // expected input count
  std::map<std::string, int> expected_input_count;

  // completed executions
  std::map<std::string, bool> completed_executions;
};

}  // namespace capabilities2_runner
