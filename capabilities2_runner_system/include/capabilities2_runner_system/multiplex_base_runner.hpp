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
   * @param uid unique identifier for the execution
   */
  virtual void execution(int uid)
  {
    info_("execution started for uid: " + std::to_string(uid));

    // trigger the events related to on_success state
    if (events[uid].on_success.interface != "")
    {
      event_(EventType::SUCCEEDED, uid, events[uid].on_success.interface, events[uid].on_success.provider);
      triggerFunction_(events[uid].on_success.interface, update_on_success(events[uid].on_success.parameters));
    }
    // trigger the events related to on_failure state
    else if (events[uid].on_failure.interface != "")
    {
      event_(EventType::FAILED, uid, events[uid].on_failure.interface, events[uid].on_failure.provider);
      triggerFunction_(events[uid].on_failure.interface, update_on_failure(events[uid].on_failure.parameters));
    }
  }

  /**
   * @brief attach events to the runner
   *
   * @param event_option event_options related for the action
   * @param triggerFunction external function that triggers capability runners
   *
   * @return number of attached events
   */
  virtual int attach_events(event_logger::event_opts& event_option,
                            std::function<void(const std::string&, const std::string&)> triggerFunction) override
  {
    info_("accepted event options with ID : " + std::to_string(insert_id));

    triggerFunction_ = triggerFunction;

    tinyxml2::XMLElement* on_success_params = convert_to_xml(event_option.on_success.parameters);

    int uid = NULL;

    // extract the uid from the event options from whatever runner is present by looping
    if (event_option.on_success.interface != "")
    {
      tinyxml2::XMLElement* on_success_params = convert_to_xml(event_option.on_success.parameters);
      on_success_params->QueryIntAttribute("uid", &uid);
    }
    else if (event_option.on_failure.interface != "")
    {
      tinyxml2::XMLElement* on_failure_params = convert_to_xml(event_option.on_failure.parameters);
      on_failure_params->QueryIntAttribute("uid", &uid);
    }
    else if (event_option.on_started.interface != "")
    {
      tinyxml2::XMLElement* on_started_params = convert_to_xml(event_option.on_started.parameters);
      on_started_params->QueryIntAttribute("uid", &uid);
    }
    else if (event_option.on_stopped.interface != "")
    {
      tinyxml2::XMLElement* on_stopped_params = convert_to_xml(event_option.on_stopped.parameters);
      on_stopped_params->QueryIntAttribute("uid", &uid);
    }

    events[uid] = event_option;

    return uid;
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
};
}  // namespace capabilities2_runner