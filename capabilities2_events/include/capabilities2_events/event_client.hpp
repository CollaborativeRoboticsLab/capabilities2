#pragma once
#include <rclcpp/rclcpp.hpp>
#include <capabilities2_msgs/msg/capability_event.hpp>

/**
 * @brief A class to publish events to a given topic
 *
 */
class EventClient
{
public:
  using Event = capabilities2_msgs::msg::CapabilityEvent;

  /**
   * @brief Construct a new Status Client object
   *
   * @param node Pointer to the node
   * @param node_name node name to be used for status message
   * @param topic_name topic name to publish the message
   */
  EventClient(rclcpp::Node::SharedPtr node, const std::string& node_name, const std::string& topic_name)
  {
    node_ = node;
    node_name_ = node_name;
    event_publisher_ = node_->create_publisher<Event>(topic_name, 10);
  }

  /**
   * @brief publishes status information to the given topic
   *
   * @param message Message to be published
   */
  void publish(const Event& message)
  {
    event_publisher_->publish(message);
  }

  /**
   * @brief publishes status information to the given topic as info
   *
   * @param text Text to be published
   */

  void info(const std::string& text, int thread_id = -1)
  {
    auto message = Event();

    message.header.stamp = node_->now();
    message.origin_node = node_name_;
    message.source.capability = "";
    message.source.provider = "";
    message.target.capability = "";
    message.target.provider = "";
    message.thread_id = thread_id;
    message.event = Event::UNDEFINED;
    message.type = Event::INFO;
    message.content = text;
    message.pid = -1;

    publish(message);
  }

  /**
   * @brief publishes status information to the given topic as debug
   *
   * @param text Text to be published
   */

  void debug(const std::string& text, int thread_id = -1)
  {
    auto message = Event();

    message.header.stamp = node_->now();
    message.origin_node = node_name_;
    message.source.capability = "";
    message.source.provider = "";
    message.target.capability = "";
    message.target.provider = "";
    message.thread_id = thread_id;
    message.event = Event::UNDEFINED;
    message.type = Event::DEBUG;
    message.content = text;
    message.pid = -1;

    publish(message);
  }

  /**
   * @brief publishes status information to the given topic as error
   *
   * @param text Text to be published
   */
  void error(const std::string& text, int thread_id = -1)
  {
    auto message = Event();

    message.header.stamp = node_->now();
    message.origin_node = node_name_;
    message.source.capability = "";
    message.source.provider = "";
    message.target.capability = "";
    message.target.provider = "";
    message.thread_id = thread_id;
    message.event = Event::UNDEFINED;
    message.type = Event::ERROR;
    message.content = text;
    message.pid = -1;

    publish(message);
  }

  /**
   * @brief publishes element information to the given topic as error
   *
   * @param element element information to be published
   */
  void error_element(const std::string& element, int thread_id = -1)
  {
    auto message = Event();

    message.header.stamp = node_->now();
    message.origin_node = node_name_;
    message.source.capability = "";
    message.source.provider = "";
    message.target.capability = "";
    message.target.provider = "";
    message.thread_id = thread_id;
    message.event = Event::UNDEFINED;
    message.type = Event::ERROR_ELEMENT;
    message.content = element;
    message.pid = -1;

    publish(message);
  }

  /**
   * @brief publishes element information to the given topic as event
   *
   * @param element element information to be published
   */
  void runner_define(const std::string& src_capability, const std::string& src_provider, const std::string& tgt_capability,
                     const std::string& tgt_provider, uint8_t event = Event::UNDEFINED)
  {
    auto message = Event();

    message.header.stamp = node_->now();
    message.origin_node = node_name_;
    message.source.capability = src_capability;
    message.source.provider = src_provider;
    message.target.capability = tgt_capability;
    message.target.provider = tgt_provider;
    message.thread_id = -1;
    message.event = event;
    message.type = Event::RUNNER_DEFINE;
    message.content = "";
    message.pid = -1;

    publish(message);
  }

protected:
  /**
   * @brief Node pointer to access logging interface
   *
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief publisher to publish execution status
   *
   */
  rclcpp::Publisher<Event>::SharedPtr event_publisher_;

  /**
   * @brief Node name
   *
   */
  std::string node_name_;
};