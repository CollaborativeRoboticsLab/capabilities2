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
   * @brief publishes status information to the given topic as info
   *
   * @param text Text to be published
   */

  void info(const std::string& text)
  {
    auto message = Event();

    message.header.stamp = node_->now();
    message.origin_node = node_name_;
    message.source.capability = "";
    message.source.provider = "";
    message.target.capability = "";
    message.target.provider = "";
    message.thread_id = 0;
    message.event = Event::UNDEFINED;
    message.error = false;
    message.text = text;
    message.is_failed_element = false;
    message.element = "";
    message.pid = -1;

    event_publisher_->publish(message);
  }

  /**
   * @brief publishes status information to the given topic as info
   *
   * @param message Message to be published
   */
  void info(const Event& message)
  {
    event_publisher_->publish(message);
  }

  /**
   * @brief publishes status information to the given topic as error
   *
   * @param text Text to be published
   */
  void error(const std::string& text)
  {
    auto message = Event();

    message.header.stamp = node_->now();
    message.origin_node = node_name_;
    message.source.capability = "";
    message.source.provider = "";
    message.target.capability = "";
    message.target.provider = "";
    message.thread_id = 0;
    message.event = Event::UNDEFINED;
    message.error = true;
    message.text = text;
    message.is_failed_element = false;
    message.element = "";
    message.pid = -1;

    event_publisher_->publish(message);
  }

  /**
   * @brief publishes status information to the given topic as error
   *
   * @param message Message to be published
   */
  void error(const Event& message)
  {
    event_publisher_->publish(message);
  }

  /**
   * @brief publishes element information to the given topic as error
   *
   * @param element element information to be published
   */
  void error_element(const std::string& element)
  {
    auto message = Event();

    message.header.stamp = node_->now();
    message.origin_node = node_name_;
    message.source.capability = "";
    message.source.provider = "";
    message.target.capability = "";
    message.target.provider = "";
    message.thread_id = 0;
    message.event = Event::UNDEFINED;
    message.error = true;
    message.text = "Failed element";
    message.is_failed_element = true;
    message.element = element;
    message.pid = -1;

    event_publisher_->publish(message);
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