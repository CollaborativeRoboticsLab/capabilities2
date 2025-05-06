#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <capabilities2_msgs/msg/capability_event.hpp>

class CapabilitiesEventListener : public rclcpp::Node
{
public:
  using Event = capabilities2_msgs::msg::CapabilityEvent;

  CapabilitiesEventListener(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("capabilities2_events_listener", options)
  {
    // Create a subscription to the "topic" topic
    RCLCPP_INFO(this->get_logger(), "Creating subscription to topic");

    subscription_ = this->create_subscription<Event>("/events", 10, std::bind(&CapabilitiesEventListener::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const Event& msg) const
  {
    std::string text;

    if (msg.is_failed_element)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "/" + std::to_string(msg.thread_id) + "] " + msg.text + " : " + msg.element;
    }
    else if (msg.thread_id >= 0 and msg.target.capability == "" and msg.source.capability == "")
    {
      text = "[" + msg.origin_node + "]" + "[" + std::to_string(msg.thread_id) + "] " + msg.text;
    }
    else if (msg.thread_id < 0 and msg.target.capability == "" and msg.source.capability == "")
    {
      text = "[" + msg.origin_node + "] " + msg.text;
    }
    else if (msg.thread_id >= 0 and msg.target.capability == "" and msg.source.capability != "")
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "/" + std::to_string(msg.thread_id) + "] " + msg.text;
    }
    else if (msg.thread_id < 0 and msg.target.capability == "" and msg.source.capability != "")
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "] " + msg.text;
    }
    else if (msg.thread_id >= 0 and msg.target.capability != "")
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "/" + std::to_string(msg.thread_id) + "] triggering " +
             msg.target.capability + " " + msg.text;
    }
    else if (msg.thread_id < 0 and msg.target.capability != "")
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "] triggering " + msg.target.capability + " " + msg.text;
    }

    if (msg.error)
      RCLCPP_ERROR(get_logger(), text.c_str());
    else
      RCLCPP_INFO(get_logger(), text.c_str());
  }

  rclcpp::Subscription<Event>::SharedPtr subscription_;
};