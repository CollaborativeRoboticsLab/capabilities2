#pragma once

#include <capabilities2_events/event_base.hpp>

#include <rclcpp/rclcpp.hpp>
#include <capabilities2_msgs/msg/capability_event_stamped.hpp>

namespace capabilities2_events
{

class PublishedEvent : public EventBase
{
public:
  PublishedEvent(rclcpp::Publisher<capabilities2_msgs::msg::CapabilityEventStamped>::SharedPtr event_pub)
    : EventBase(), event_pub_(event_pub)
  {
  }

  ~PublishedEvent() = default;

  /**
   * @brief see EventBase::emit, specialised to publish events
   */
  void emit(const std::string& connection_id, const uint8_t& event_code,
            const capabilities2_msgs::msg::Capability& source, const capabilities2_msgs::msg::Capability& target,
            EventBase::event_callback_t callback) override
  {
    // split connection id to get trigger id. Assuming connection_id format is "bond_id/instance_id/target_instance_id"
    size_t first_pos = connection_id.find('/');
    size_t second_pos = connection_id.find('/', first_pos + 1);

    std::string instance_id = (second_pos != std::string::npos) ? connection_id.substr(first_pos + 1, second_pos - first_pos - 1) : "";

    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.event.trigger_id = instance_id;
    event_msg.event.code.code = event_code;
    event_msg.event.connection.source = source;
    event_msg.event.connection.target = target;

    // publish event
    event_pub_->publish(event_msg);

    // call super
    EventBase::emit(connection_id, event_code, source, target, callback);
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
    event_msg.event.code.code = capabilities2_msgs::msg::CapabilityEventCode::SERVER_READY;
    event_msg.event.description = msg;

    event_pub_->publish(event_msg);
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
    event_msg.event.code.code = capabilities2_msgs::msg::CapabilityEventCode::LAUNCHED;
    event_msg.event.description = "Process launched with PID: " + pid;

    event_pub_->publish(event_msg);
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
    event_msg.event.code.code = capabilities2_msgs::msg::CapabilityEventCode::TERMINATED;
    event_msg.event.description = "Process terminated with PID: " + pid;

    event_pub_->publish(event_msg);
  }

  void on_triggered(const std::string& trigger_id)
  {
    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.event.connection.source.capability = "capabilities2_server";
    event_msg.event.connection.source.provider = "capabilities2_server";
    event_msg.event.code.code = capabilities2_msgs::msg::CapabilityEventCode::TRIGGERED;
    event_msg.event.description = "Triggered event with ID: " + trigger_id;

    event_pub_->publish(event_msg);
  }

  void on_connected(const std::string& source, const std::string& target)
  {
    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.event.connection.source.capability = "capabilities2_server";
    event_msg.event.connection.source.provider = "capabilities2_server";
    event_msg.event.code.code = capabilities2_msgs::msg::CapabilityEventCode::CONNECTED;
    event_msg.event.description = "Connected event from " + source + " to " + target;

    event_pub_->publish(event_msg);
  }

  void on_disconnected(const std::string& source, const std::string& target)
  {
    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.event.connection.source.capability = "capabilities2_server";
    event_msg.event.connection.source.provider = "capabilities2_server";
    event_msg.event.code.code = capabilities2_msgs::msg::CapabilityEventCode::DISCONNECTED;
    event_msg.event.description = "Disconnected event from " + source + " to " + target;

    event_pub_->publish(event_msg);
  }

private:
  // event publisher
  rclcpp::Publisher<capabilities2_msgs::msg::CapabilityEventStamped>::SharedPtr event_pub_;
};

}  // namespace capabilities2_events
