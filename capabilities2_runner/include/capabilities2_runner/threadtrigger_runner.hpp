#pragma once

#include <chrono>
#include <map>
#include <atomic>
#include <mutex>
#include <thread>

#include <capabilities2_runner/runner_base.hpp>
#include <capabilities2_events/uuid_generator.hpp>
#include <capabilities2_msgs/msg/capability_event_code.hpp>

namespace capabilities2_runner
{

/**
 * @brief add threaded trigger execution to the runner
 *
 */
class ThreadTriggerRunner : public RunnerBase
{
public:
  /**
   * @brief helper function to extract bond_id from thread_id
   *
   * @param thread_id  unique identifier for the execution thread, format: "bond_id/trigger_id"
   *
   * @return const std::string
   */
  static const std::string bond_from_thread_id(const std::string& thread_id)
  {
    // extract bond_id from thread_id (format: "bond_id/trigger_id")
    size_t slash_pos = thread_id.find('/');
    std::string bond_id = (slash_pos != std::string::npos) ? thread_id.substr(0, slash_pos) : thread_id;
    return bond_id;
  }

  /**
   * @brief helper function to extract instance_id from thread_id
   *
   * @param thread_id unique identifier for the execution thread, format: "bond_id/instance_id"
   * @return const std::string
   */
  static const std::string instance_from_thread_id(const std::string& thread_id)
  {
    // extract instance_id from thread_id (format: "bond_id/instance_id")
    size_t slash_pos = thread_id.find('/');
    std::string instance_id = (slash_pos != std::string::npos) ? thread_id.substr(slash_pos + 1) : "";
    return instance_id;
  }

public:
  ThreadTriggerRunner() : RunnerBase(), execution_thread_pool_(), mutex_(), execution_should_stop_(false)
  {
  }

  ~ThreadTriggerRunner()
  {
    // stop any running threads on destruction
    stop_execution(std::chrono::milliseconds(500));
  }

  /**
   * @brief Trigger the runner
   *
   * This method allows insertion of parameters in a runner after it has been initialized. it is an approach
   * to parameterise capabilities. Internally starts up RunnerBase::triggerExecution in a thread
   *
   * @param parameters pointer to tinyxml2::XMLElement that contains parameters
   * @param bond_id unique identifier for the group of connections associated with this runner trigger event
   * @param instance_id unique identifier for the instance associated with this runner trigger event
   */
  virtual void trigger(capabilities2_events::EventParameters& parameters, const std::string& bond_id,
                       const std::string& instance_id) override
  {
    // namespace the thread id with bond id for later
    // could list all threads related to a bond if needed
    std::string thread_id = bond_id + "/" + instance_id;

    // start execution thread
    {
      std::scoped_lock lock(mutex_);
      // TODO: consider emitting on start event here
      // emit_event(bond_id, capabilities2_msgs::msg::CapabilityEventCode::ON_STARTED, updated_on_started(parameters));
      execution_thread_pool_[thread_id] = std::thread(&ThreadTriggerRunner::execution, this, parameters, thread_id);
    }

    // emit trigger event
    emit_event(bond_id, instance_id, capabilities2_msgs::msg::CapabilityEventCode::TRIGGERED);

    // BUG: thread management?

    // TODO: consider emitting on stop event here
    // emit_event(bond_id, capabilities2_msgs::msg::CapabilityEventCode::ON_STOPPED, updated_on_stopped(parameters));

    RCLCPP_DEBUG(node_->get_logger(), "started execution thread for thread id: %s", thread_id.c_str());
  }

protected:
  /**
   * @brief Trigger process to be executed.
   *
   * This method utilizes parameters set via the trigger() function
   *
   * @param parameters parameters for the execution
   * @param thread_id unique identifier for the execution thread, can be used for tracking and cleanup
   *
   * @attention: Should be implemented on derieved classes and should call success and failure events appropriately
   */
  virtual void execution(capabilities2_events::EventParameters parameters, const std::string& thread_id) = 0;

private:
  /** */
  void stop_execution(std::chrono::milliseconds timeout = std::chrono::milliseconds(500))
  {
    // signal listening threads to stop
    execution_should_stop_ = true;

    // try join in a non-blocking way and log if threads are not cleaned up properly
    {
      std::scoped_lock lock(mutex_);
      for (auto& [thread_id, exec_thread] : execution_thread_pool_)
      {
        if (exec_thread.joinable())
        {
          // TODO: FIX THIS FUTURE MICHAEL
          exec_thread.join();
        }
      }

      // clear the thread pool
      execution_thread_pool_.clear();
    }
  }

protected:
  /**
   * @brief dictionary of threads that executes the execute function
   */
  std::map<std::string, std::thread> execution_thread_pool_;

  /**
   * @brief mutex for threadpool synchronisation.
   */
  std::mutex mutex_;

  /**
   * @brief flag to signal execution threads to stop.
   */
  std::atomic<bool> execution_should_stop_;
};

}  // namespace capabilities2_runner
