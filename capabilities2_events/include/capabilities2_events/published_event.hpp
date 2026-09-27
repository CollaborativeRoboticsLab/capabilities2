#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include <capabilities2_events/event_base.hpp>

#include <rclcpp/rclcpp.hpp>
#include <capabilities2_msgs/msg/capability_event_stamped.hpp>

namespace capabilities2_events
{

class PublishedEvent : public EventBase
{
public:
  struct SnapshotResult
  {
    rclcpp::Time stamp;
    uint64_t oldest_sequence{ 0 };
    uint64_t latest_sequence{ 0 };
    uint32_t max_events{ 0 };
    bool truncated{ false };
    std::vector<capabilities2_msgs::msg::CapabilityEventStamped> events;
  };

  PublishedEvent(rclcpp::Publisher<capabilities2_msgs::msg::CapabilityEventStamped>::SharedPtr event_pub,
                 size_t max_history_size = 100)
    : EventBase(), event_pub_(event_pub), max_history_size_(max_history_size)
  {
  }

  ~PublishedEvent() = default;

  /**
   * @brief pverrides EventBase::emit to publish the event to a ROS2 topic
   * 
   * @param connection_id connection identifier (format: "bond_id/instance_id/target_instance_id")
   * @param ownership_id ownership identifier for the connection (used for access control)
   * @param event_code type of event being emitted
   * @param source source capability emitting the event
   * @param target target capability receiving the event
   * @param callback function to trigger target capability with (capability, parameters, bond_id, target_instance_id)
   */
  void emit(const std::string& connection_id, const std::string& ownership_id, const uint8_t& event_code,
            const capabilities2_msgs::msg::Capability& source, const capabilities2_msgs::msg::Capability& target,
            EventBase::event_callback_t callback) override
  {
    // split connection id to get trigger id. Assuming connection_id format is "bond_id/instance_id/target_instance_id"
    size_t first_pos = connection_id.find('/');
    size_t second_pos = connection_id.find('/', first_pos + 1);

    std::string instance_id =
        (second_pos != std::string::npos) ? connection_id.substr(first_pos + 1, second_pos - first_pos - 1) : "";

    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.event.trigger_id = instance_id;
    event_msg.event.code.code = event_code;
    event_msg.event.connection.source = source;
    event_msg.event.connection.target = target;
    event_msg.event.connection.ownership_id = ownership_id;

    publish_event(event_msg);

    // call super
    EventBase::emit(connection_id, ownership_id, event_code, source, target, callback);
  }

  void publish_observed_event(const std::string& ownership_id, const uint8_t& event_code,
                              const capabilities2_msgs::msg::Capability& source,
                              const std::string& instance_id) override
  {
    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.event.trigger_id = instance_id;
    event_msg.event.code.code = event_code;
    event_msg.event.connection.source = source;
    event_msg.event.connection.ownership_id = ownership_id;

    publish_event(event_msg);
  }

  /**
   * @brief on server ready event
   *
   * @param msg
   */
  void on_server_ready(const std::string& msg)
  {
    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.event.connection.source.capability = "capabilities2_server";
    event_msg.event.connection.source.provider = "capabilities2_server";
    event_msg.event.connection.ownership_id = "";
    event_msg.event.code.code = capabilities2_msgs::msg::CapabilityEventCode::SERVER_READY;
    event_msg.event.description = msg;

    publish_event(event_msg);
  }

  /**
   * @brief on process launched event
   *
   * @param pid
   */
  void on_process_launched(const std::string& pid)
  {
    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.event.connection.source.capability = "capabilities2_server";
    event_msg.event.connection.source.provider = "capabilities2_server";
    event_msg.event.connection.ownership_id = "";
    event_msg.event.code.code = capabilities2_msgs::msg::CapabilityEventCode::LAUNCHED;
    event_msg.event.description = "Process launched with PID: " + pid;

    publish_event(event_msg);
  }

  /**
   * @brief on process terminated event
   *
   * @param pid
   */
  void on_process_terminated(const std::string& pid)
  {
    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.event.connection.source.capability = "capabilities2_server";
    event_msg.event.connection.source.provider = "capabilities2_server";
    event_msg.event.connection.ownership_id = "";
    event_msg.event.code.code = capabilities2_msgs::msg::CapabilityEventCode::TERMINATED;
    event_msg.event.description = "Process terminated with PID: " + pid;

    publish_event(event_msg);
  }

  void on_triggered(const std::string& trigger_id)
  {
    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.event.connection.source.capability = "capabilities2_server";
    event_msg.event.connection.source.provider = "capabilities2_server";
    event_msg.event.connection.ownership_id = "";
    event_msg.event.code.code = capabilities2_msgs::msg::CapabilityEventCode::TRIGGERED;
    event_msg.event.description = "Triggered event with ID: " + trigger_id;

    publish_event(event_msg);
  }

  void on_connected(const std::string& source, const std::string& target)
  {
    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.event.connection.source.capability = "capabilities2_server";
    event_msg.event.connection.source.provider = "capabilities2_server";
    event_msg.event.connection.ownership_id = "";
    event_msg.event.code.code = capabilities2_msgs::msg::CapabilityEventCode::CONNECTED;
    event_msg.event.description = "Connected event from " + source + " to " + target;

    publish_event(event_msg);
  }

  void on_disconnected(const std::string& source, const std::string& target)
  {
    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.event.connection.source.capability = "capabilities2_server";
    event_msg.event.connection.source.provider = "capabilities2_server";
    event_msg.event.connection.ownership_id = "";
    event_msg.event.code.code = capabilities2_msgs::msg::CapabilityEventCode::DISCONNECTED;
    event_msg.event.description = "Disconnected event from " + source + " to " + target;

    publish_event(event_msg);
  }

  SnapshotResult get_event_snapshot(uint64_t after_sequence, size_t requested_max_events) const
  {
    SnapshotResult snapshot;
    snapshot.max_events = static_cast<uint32_t>(max_history_size_);

    std::lock_guard<std::mutex> lock(history_mutex_);

    if (event_history_.empty())
    {
      snapshot.stamp = rclcpp::Clock().now();
      return snapshot;
    }

    snapshot.stamp = event_history_.back().event.header.stamp;
    snapshot.oldest_sequence = event_history_.front().sequence;
    snapshot.latest_sequence = event_history_.back().sequence;

    const size_t bounded_request_count = requested_max_events > 0
      ? std::min(requested_max_events, max_history_size_)
      : max_history_size_;

    snapshot.truncated = after_sequence > 0 && after_sequence + 1 < snapshot.oldest_sequence;

    for (const auto& stored_event : event_history_)
    {
      if (stored_event.sequence <= after_sequence)
      {
        continue;
      }

      snapshot.events.push_back(stored_event.event);
      if (snapshot.events.size() >= bounded_request_count)
      {
        break;
      }
    }

    return snapshot;
  }

private:
  struct SequencedEvent
  {
    uint64_t sequence{ 0 };
    capabilities2_msgs::msg::CapabilityEventStamped event;
  };

  void publish_event(const capabilities2_msgs::msg::CapabilityEventStamped& event_msg)
  {
    if (event_pub_)
    {
      event_pub_->publish(event_msg);
    }

    SequencedEvent stored_event;

    {
      std::lock_guard<std::mutex> lock(history_mutex_);
      stored_event.sequence = next_sequence_++;
      stored_event.event = event_msg;
      event_history_.push_back(stored_event);
      while (event_history_.size() > max_history_size_)
      {
        event_history_.pop_front();
      }
    }
  }

  // event publisher
  rclcpp::Publisher<capabilities2_msgs::msg::CapabilityEventStamped>::SharedPtr event_pub_;
  std::deque<SequencedEvent> event_history_;
  mutable std::mutex history_mutex_;
  uint64_t next_sequence_{ 1 };
  size_t max_history_size_;
};

}  // namespace capabilities2_events
