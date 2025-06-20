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

    subscription_ =
        this->create_subscription<Event>("/events", 10, std::bind(&CapabilitiesEventListener::topic_callback, this, std::placeholders::_1));
  }

private:
  /**
   * @brief Constructs a message from the event and logs it
   *
   * @param msg
   */
  void topic_callback(const Event& msg) const
  {
    std::string text;

    // Construct the text based on the event type and content

    // if the event contains and errornous element, we log it as an error
    if (msg.type == Event::ERROR_ELEMENT)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "/" + std::to_string(msg.thread_id) + "] " + msg.content;
    }
    // If the event is at runner definition and logs on start event
    else if (msg.type == Event::RUNNER_DEFINE and msg.event == Event::STARTED)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "] will trigger [" + msg.target.capability + "] on start";
    }
    // If the event is at runner definition and logs on stop event
    else if (msg.type == Event::RUNNER_DEFINE and msg.event == Event::STOPPED)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "] will trigger [" + msg.target.capability + "] on stop";
    }
    // If the event is at runner definition and logs on failure event
    else if (msg.type == Event::RUNNER_DEFINE and msg.event == Event::FAILED)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "] will trigger [" + msg.target.capability + "] on failure";
    }
    // If the event is at runner definition and logs on success event
    else if (msg.type == Event::RUNNER_DEFINE and msg.event == Event::SUCCEEDED)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "] will trigger [" + msg.target.capability + "] on success";
    }
    // If the event is at runner execution and logs on start event from main thread
    else if (msg.type == Event::RUNNER_EVENT and msg.event == Event::STARTED and msg.thread_id < 0)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "] triggering [" + msg.target.capability + "] on start";
    }
    // If the event is at runner execution and logs on start event from worker thread
    else if (msg.type == Event::RUNNER_EVENT and msg.event == Event::STARTED and msg.thread_id >= 0)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "/" + std::to_string(msg.thread_id) + "] triggering " +
             msg.target.capability + "] on start";
    }
    // If the event is at runner execution and logs on stop event from main thread
    else if (msg.type == Event::RUNNER_EVENT and msg.event == Event::STOPPED and msg.thread_id < 0)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "] triggering [" + msg.target.capability + "] on stop";
    }
    // If the event is at runner execution and logs on stop event from worker thread
    else if (msg.type == Event::RUNNER_EVENT and msg.event == Event::STOPPED and msg.thread_id >= 0)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "/" + std::to_string(msg.thread_id) + "] triggering " +
             msg.target.capability + "] on stop";
    }
    // If the event is at runner execution and logs on failure event from main thread
    else if (msg.type == Event::RUNNER_EVENT and msg.event == Event::FAILED and msg.thread_id < 0)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "] triggering [" + msg.target.capability + "] on failure";
    }
    // If the event is at runner execution and logs on failure event from worker thread
    else if (msg.type == Event::RUNNER_EVENT and msg.event == Event::FAILED and msg.thread_id >= 0)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "/" + std::to_string(msg.thread_id) + "] triggering " +
             msg.target.capability + "] on failure";
    }
    // If the event is at runner execution and logs on success event from main thread
    else if (msg.type == Event::RUNNER_EVENT and msg.event == Event::SUCCEEDED and msg.thread_id < 0)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "] triggering [" + msg.target.capability + "] on success";
    }
    // If the event is at runner execution and logs on success event from worker thread
    else if (msg.type == Event::RUNNER_EVENT and msg.event == Event::SUCCEEDED and msg.thread_id >= 0)
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "/" + std::to_string(msg.thread_id) + "] triggering " +
             msg.target.capability + "] on success";
    }
    // no capabilities means running on main node, thread id is -1 means on main thread
    else if (msg.thread_id < 0 and msg.target.capability == "" and msg.source.capability == "")
    {
      text = "[" + msg.origin_node + "] " + msg.content;
    }
    // no capabilities means running on main node, thread id is >0 means on worker thread. 
    else if (msg.thread_id >= 0 and msg.target.capability == "" and msg.source.capability == "")
    {
      text = "[" + msg.origin_node + "]" + "[" + std::to_string(msg.thread_id) + "] " + msg.content;
    }
    // source capability is set, target capability is not set means msg is from within a capability, thread id is -1 means on main thread
    else if (msg.thread_id < 0 and msg.target.capability == "" and msg.source.capability != "")
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "] " + msg.content;
    }
    // source capability is set, target capability is not set means msg is from within a capability, thread id is >0 means on worker thread. 
    else if (msg.thread_id >= 0 and msg.target.capability == "" and msg.source.capability != "")
    {
      text = "[" + msg.origin_node + "]" + "[" + msg.source.capability + "/" + std::to_string(msg.thread_id) + "] " + msg.content;
    }

    if (msg.type == Event::ERROR)
      RCLCPP_ERROR(get_logger(), text.c_str());
    else if (msg.type == Event::ERROR_ELEMENT)
      RCLCPP_ERROR(get_logger(), text.c_str());
    else if (msg.type == Event::DEBUG)
      RCLCPP_DEBUG(get_logger(), text.c_str());
    else if (msg.type == Event::INFO)
      RCLCPP_INFO(get_logger(), text.c_str());
    else if (msg.type == Event::RUNNER_DEFINE)
      RCLCPP_INFO(get_logger(), text.c_str());
    else if (msg.type == Event::RUNNER_EVENT)
      RCLCPP_INFO(get_logger(), text.c_str());
    else
      RCLCPP_INFO(get_logger(), text.c_str());
  }

  rclcpp::Subscription<Event>::SharedPtr subscription_;
};