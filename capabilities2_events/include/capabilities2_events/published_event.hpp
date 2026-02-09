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
    // split connection id to get trigger id
    std::string trigger_id = connection_id;
    size_t slash_pos = connection_id.find('/');
    if (slash_pos != std::string::npos)
    {
      trigger_id = connection_id.substr(slash_pos + 1);
    }

    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.trigger_id = trigger_id;
    event_msg.event_code.code = event_code;
    event_msg.source = source;
    event_msg.target = target;

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
    event_msg.event_code.code = capabilities2_msgs::msg::CapabilityEventCode::SERVER_READY;
    event_msg.source.capability = "capabilities2_server";
    event_msg.source.provider = "capabilities2_server";
    event_msg.description = msg;

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
    event_msg.event_code.code = capabilities2_msgs::msg::CapabilityEventCode::PROCESS_LAUNCHED;
    event_msg.source.capability = "capabilities2_server";
    event_msg.source.provider = "capabilities2_server";
    event_msg.description = "Process launched with PID: " + pid;

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
    event_msg.event_code.code = capabilities2_msgs::msg::CapabilityEventCode::PROCESS_TERMINATED;
    event_msg.source.capability = "capabilities2_server";
    event_msg.source.provider = "capabilities2_server";
    event_msg.description = "Process terminated with PID: " + pid;

    event_pub_->publish(event_msg);
  }

private:
  // event publisher
  rclcpp::Publisher<capabilities2_msgs::msg::CapabilityEventStamped>::SharedPtr event_pub_;
};

}  // namespace capabilities2_events
