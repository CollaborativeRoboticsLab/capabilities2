#pragma once
#include <rclcpp/rclcpp.hpp>
#include <capabilities2_msgs/msg/status.hpp>

class StatusClient
{
public:
    using Status = capabilities2_msgs::msg::Status;

    /**
     * @brief Construct a new Status Client object
     * 
     * @param node Pointer to the node
     * @param node_name node name to be used for status message
     * @param topic_name topic name to publish the message
     */
    StatusClient(rclcpp::Node::SharedPtr node, const std::string& node_name,const std::string& topic_name)
    {
        node_ = node;
        node_name_ = node_name;
        status_publisher_ = node_->create_publisher<Status>(topic_name, 10);
    }

    /**
     * @brief publishes status information to the given topic and prints to the logging as info
     *
     * @param text Text to be published
     * @param newline whether to add new line to the ROS_INFO print
     */
    void info(const std::string &text, bool newline = false)
    {
        auto status_msg = Status();
        status_msg.header.stamp = node_->now();
        status_msg.origin_node = node_name_;
        status_msg.text = text;
        status_msg.failure = false;

        if (newline)
            RCLCPP_INFO(node_->get_logger(), "");

        RCLCPP_INFO(node_->get_logger(), status_msg.text.c_str());

        status_publisher_->publish(status_msg);
    }

    /**
     * @brief publishes status information to the given topic and prints to the logging as error
     * 
     * @param text Text to be published
     * @param newline whether to add new line to the ROS_INFO print
     */
    void error(const std::string &text, bool newline = false)
    {
        auto status_msg = Status();
        status_msg.header.stamp = node_->now();
        status_msg.origin_node = node_name_;
        status_msg.text = text;
        status_msg.failure = true;
        status_msg.is_failed_element = false;

        if (newline)
            RCLCPP_ERROR(node_->get_logger(), "");

        RCLCPP_ERROR(node_->get_logger(), status_msg.text.c_str());

        status_publisher_->publish(status_msg);
    }

    /**
     * @brief publishes status information to the given topic and prints to the logging as error
     * 
     * @param text Text to be published
     * @param newline whether to add new line to the ROS_INFO print
     */
    void error_element(const std::string &element)
    {
        auto status_msg = Status();
        status_msg.header.stamp = node_->now();
        status_msg.origin_node = node_name_;
        status_msg.text = "Failed element";
        status_msg.failure = true;
        status_msg.is_failed_element = true;
        status_msg.failed_element = element;

        std::string msg = status_msg.text + " : " + status_msg.failed_element;

        RCLCPP_ERROR(node_->get_logger(), msg.c_str());

        status_publisher_->publish(status_msg);
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
    rclcpp::Publisher<Status>::SharedPtr status_publisher_;

    /**
     * @brief Node name
     *
     */
    std::string node_name_;
};