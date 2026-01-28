#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <capabilities2_events/event_base.hpp>
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

  ~PublishedEvent();

  /** */
  void emit(const std::string& trigger_id, const capabilities2_msgs::msg::CapabilityEventCode& event_code,
            const capabilities2_msgs::msg::Capability& source, const capabilities2_msgs::msg::Capability& target,
            std::function<void(const std::string&, const std::string&)> callback) override
  {
    capabilities2_msgs::msg::CapabilityEventStamped event_msg;
    event_msg.header.stamp = rclcpp::Clock().now();
    event_msg.trigger_id = trigger_id;
    event_msg.event_code = event_code;
    event_msg.source = source;
    event_msg.target = target;

    // publish event
    event_pub_->publish(event_msg);

    // call super
    EventBase::emit(trigger_id, event_code, source, target, callback);
  }

private:
  // event publisher
  rclcpp::Publisher<capabilities2_msgs::msg::CapabilityEventStamped>::SharedPtr event_pub_;
};

}  // namespace capabilities2_events
