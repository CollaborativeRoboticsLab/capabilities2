#pragma once

#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <capabilities2_msgs/msg/capability.hpp>
#include <capabilities2_msgs/msg/capability_event.hpp>
#include <capabilities2_msgs/msg/capability_event_code.hpp>

namespace capabilities2_events
{
/**
 * @brief event exception
 *
 * Base class for event exceptions
 *
 */
struct event_exception : public std::runtime_error
{
  using std::runtime_error::runtime_error;

  event_exception(const std::string& what) : std::runtime_error(what)
  {
  }

  virtual const char* what() const noexcept override
  {
    return std::runtime_error::what();
  }
};

/**
 * @brief event base class
 *
 * Represents an event in the capabilities framework.
 * An event can be triggered to notify other components
 * about changes in running capability (runners) states.
 */
class EventBase
{
public:
  EventBase()
  {
  }

  ~EventBase();

  /**
   * @brief emit an event
   *
   * an event emission uses a parameterised callback
   * which lets loosely-coupled capabilities propogate state changes
   * when a source capability emits an event to a target capability
   *
   * @param trigger_id
   * @param event_code
   * @param source
   * @param target
   * @param callback
   */
  virtual void emit(const std::string& trigger_id, const capabilities2_msgs::msg::CapabilityEventCode& event_code,
                    const capabilities2_msgs::msg::Capability& source,
                    const capabilities2_msgs::msg::Capability& target,
                    std::function<void(const std::string&, const std::string&)> callback)
  {
    // do callback
    if (callback)
    {
      callback(target.capability, target.parameters);
    }
  }
};

}  // namespace capabilities2_events
