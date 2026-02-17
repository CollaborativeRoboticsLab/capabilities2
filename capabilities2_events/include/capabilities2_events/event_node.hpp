#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <capabilities2_events/uuid_generator.hpp>
#include <capabilities2_events/event_base.hpp>

#include <capabilities2_msgs/msg/capability.hpp>
#include <capabilities2_msgs/msg/capability_event_code.hpp>
#include <capabilities2_msgs/msg/capability_connection.hpp>

namespace capabilities2_events
{
/**
 * @brief an event node is a source of an event
 *
 * events when emitted will trigger interactions with connected nodes
 *
 */
class EventNode
{
private:
  /**
   * @brief event pipe structure
   *
   * represents a connection from this event node to a target capability
   * contains the callback to be invoked when the event is emitted
   *
   */
  struct EventPipe
  {
    // connection type
    capabilities2_msgs::msg::CapabilityEventCode type;

    // connection target
    capabilities2_msgs::msg::Capability target;

    // event callback with signature: (capability, parameters, bond_id)
    EventBase::event_callback_t callback;
  };

public:
  EventNode(std::shared_ptr<EventBase> event_emitter = nullptr)
    : id_(UUIDGenerator::gen_uuid_str()), source_(), event_emitter_(event_emitter), connections_()
  {
  }

  virtual ~EventNode() = default;

  /** event handling */

  /**
   * @brief emit an event from this event node to all matching connections
   *
   * @param bond_id the bond_id to match connections with (extracted from connection_id) for access control
   * @param event_type the type of event being emitted
   * @param msg_parameters the new parameters to emit with the event
   */
  void emit_event(const std::string& bond_id, const std::string& instance_id, const uint8_t& event_type,
                  capabilities2_events::EventParameters parameters = capabilities2_events::EventParameters())
  {
    // check if event emitter is set
    if (!event_emitter_)
    {
      // No event emitter configured - silently skip
      // This allows runners to work without event system if needed
      return;
    }

    // check each connection
    for (const auto& [conn_id, connection] : connections_)
    {
      // extract bond_id from connection_id (format: "bond_id/instance_id/target_instance_id")
      size_t first_pos = conn_id.find('/');
      size_t second_pos = conn_id.find('/', first_pos + 1);

      std::string conn_bond_id = (first_pos != std::string::npos) ? conn_id.substr(0, first_pos) : conn_id;
      std::string conn_instance_id =
          (second_pos != std::string::npos) ? conn_id.substr(first_pos + 1, second_pos - first_pos - 1) : "";

      // get targets for this event type and id namespace
      if (connection.type.code == event_type && bond_id == conn_bond_id && instance_id == conn_instance_id)
      {
        // parameterise target capability with parameters from the trigger
        auto old_parameters = capabilities2_events::EventParameters(connection.target);

        // extend or replace parameters of the target capability if any non empty parameters are provided
        if (!parameters.is_empty())
          for (auto& option : parameters.options)
            old_parameters.set_value(option.key, option.get_value(), option.type);

        // create a new target capability message with updated parameters to emit with the event
        auto target_with_params = old_parameters.toMsg();

        // copy capability and provider from original target as parameters only contains the options
        target_with_params.capability = connection.target.capability;
        target_with_params.provider = connection.target.provider;
        target_with_params.instance_id = connection.target.instance_id;

        // emit event via event api
        // NOTE: callback invocation is handled by event api
        // this allows the nodes to be decoupled
        // nodes just keep track of connections
        // modifying execution of other nodes is not owned by this class
        // this lets the execution flow be made thread-safe
        // in the scope where the thread is owned
        event_emitter_->emit(conn_id, event_type, source_, target_with_params, connection.callback);
      }
    }
  }

protected:
  /**
   * @brief Set the source object
   *
   * @param src
   */
  void set_source(const capabilities2_msgs::msg::Capability& src)
  {
    source_ = src;
  }

  /**
   * @brief event emitter for this event node
   *
   * allows late binding of event emitter (when a node is initialized)
   *
   * @param event_emitter
   */
  void set_event_emitter(std::shared_ptr<EventBase> event_emitter)
  {
    event_emitter_ = event_emitter;
  }

  /**
   * @brief make EventNode::add_connection public method on runner api.
   *
   * add connection to this runner to target capability this allows the runner to emit events on state changes
   * to the target capability the connection ID format is: "bond_id/trigger_id"  which allows event emission to
   * extract bond_id for access control
   *
   * @param connection_id unique identifier for the connection (format: "bond_id/trigger_id")
   * @param type type of event to connect to
   * @param target target capability to connect to
   * @param event_cb callback to trigger target capability with (capability, parameters, bond_id, target_instance_id)
   *
   * @throws event_exception if connection with given id already exists
   */
  void add_connection(const std::string& connection_id, const capabilities2_msgs::msg::CapabilityEventCode& type,
                      const capabilities2_msgs::msg::Capability& target, EventBase::event_callback_t event_cb)
  {
    // validate connection id
    if (connections_.find(connection_id) != connections_.end())
    {
      throw event_exception("connection with id: " + connection_id + " already exists");
    }

    // set up an event pipe
    EventPipe conn;
    conn.type = type;
    conn.target = target;
    conn.callback = event_cb;

    // add connection
    connections_[connection_id] = conn;

    if (event_emitter_)
    {
      // emit connected event
      event_emitter_->on_connected(source_.capability, target.capability);
    }
  }

  /**
   * @brief Remove a connection by its id
   *
   * @param connection_id
   */
  void remove_connection(const std::string& connection_id)
  {
    // remove connection
    auto it = connections_.find(connection_id);
    if (it == connections_.end())
    {
      throw event_exception("connection with id: " + connection_id + " does not exist");
    }
    auto target = it->second.target;
    connections_.erase(connection_id);

    // emit disconnected event
    if (event_emitter_)
    {
      event_emitter_->on_disconnected(source_.capability, target.capability);
    }
  }

  // helper members

  /**
   * @brief clear all connections from this event node
   */
  void clear_connections()
  {
    connections_.clear();
  }

  /**
   * @brief List all connections from this event node
   *
   * @return std::vector<capabilities2_msgs::msg::CapabilityConnection>
   */
  std::vector<capabilities2_msgs::msg::CapabilityConnection> list_connections() const
  {
    std::vector<capabilities2_msgs::msg::CapabilityConnection> conns;

    for (const auto& [conn_id, connection] : connections_)
    {
      capabilities2_msgs::msg::CapabilityConnection c;
      c.type = connection.type;
      c.source = source_;
      c.target = connection.target;
      conns.push_back(c);
    }

    return conns;
  }

  /**
   * @brief Check if a connection exists
   *
   * @param connection_id
   * @return true
   * @return false
   */
  bool has_connection(const std::string& connection_id) const
  {
    return connections_.find(connection_id) != connections_.end();
  }

  /**
   * @brief current connection count
   *
   * @return size_t
   */
  size_t connection_count() const
  {
    return connections_.size();
  }

  /**
   * @brief unique id of this event node
   *
   * @return const std::string&
   */
  const std::string& get_id() const
  {
    return id_;
  }

  /**
   * @brief source capability of this event node
   *
   * @return const capabilities2_msgs::msg::Capability&
   */
  const capabilities2_msgs::msg::Capability& get_source() const
  {
    return source_;
  }

  /**
   * @brief event emitter is set on this node
   */
  bool is_event_emitter_set() const
  {
    return event_emitter_ != nullptr;
  }

private:
  // unique id of the event node
  // using uuid string
  std::string id_;

  // source capability of this event node
  capabilities2_msgs::msg::Capability source_;

  // event emitter used to emit events
  // store as member to avoid passing around
  std::shared_ptr<EventBase> event_emitter_;

  // connections from this event node to target capabilities
  // Key: connection_id (format: "bond_id/trigger_id")
  // Value: EventPipe with type, target, callback
  std::map<std::string, EventPipe> connections_;
};
}  // namespace capabilities2_events
