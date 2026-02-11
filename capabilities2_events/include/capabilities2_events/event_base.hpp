#pragma once

#include <stdexcept>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <capabilities2_events/event_parameters.hpp>
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
  typedef std::string capability_str_t;
  typedef capabilities2_events::EventParameters parameter_t;
  typedef std::string access_id_t;
  typedef std::function<void(const capability_str_t&, const parameter_t&, const access_id_t&)> event_callback_t;

public:
  EventBase()
  {
  }

  ~EventBase() = default;

  /**
   * @brief emit an event
   *
   * an event emission uses a parameterised callback
   * which lets loosely-coupled capabilities propogate state changes
   * when a source capability emits an event to a target capability
   *
   * @param connection_id connection identifier (format: "bond_id/trigger_id")
   * @param event_code type of event being emitted
   * @param source source capability emitting the event
   * @param target target capability receiving the event
   * @param callback function to trigger target capability with (capability, parameters, bond_id)
   */
  virtual void emit(const std::string& connection_id, const uint8_t& event_code,
                    const capabilities2_msgs::msg::Capability& source,
                    const capabilities2_msgs::msg::Capability& target, event_callback_t callback)
  {
    // extract bond_id from connection_id (format: "bond_id/trigger_id")
    size_t slash_pos = connection_id.find('/');
    std::string bond_id = (slash_pos != std::string::npos) ? connection_id.substr(0, slash_pos) : connection_id;

    // do callback with bond_id for access control
    if (callback)
    {
      callback(target.capability, capabilities2_events::EventParameters(target), bond_id);  
    }
  }
};

}  // namespace capabilities2_events
