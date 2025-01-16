#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <algorithm>
#include <tinyxml2.h>
#include <functional>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <capabilities2_fabric/capabilities_xml_parser.hpp>
#include <capabilities2_msgs/action/plan.hpp>

/**
 * @brief Capabilities Executor File Parser
 *
 * Capabilities Executor File Parser node that provides a ROS client for the capabilities executor.
 * Will read an XML file that implements a plan and send it to the server
 */

class CapabilitiesFileParser : public rclcpp::Node
{
public:
  CapabilitiesFileParser() : Node("Capabilities2_File_Parser")
  {
    declare_parameter("plan_file_path", "install/capabilities2_fabric/share/capabilities2_fabric/plans/default.xml");
    plan_file_path = get_parameter("plan_file_path").as_string();
  }

  /**
   * @brief Initializer function
   *
   */
  void initialize()
  {
    // Create the action client for capabilities_fabric after the node is fully constructed
    this->client_ptr_ = rclcpp_action::create_client<capabilities2_msgs::action::Plan>(shared_from_this(), "/capabilities_fabric");

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sucessfully connected to the capabilities_fabric action server");

    send_goal();
  }

  void send_goal()
  {
    // try to load the file
    tinyxml2::XMLError xml_status = document.LoadFile(plan_file_path.c_str());

    // check if the file loading failed
    if (xml_status != tinyxml2::XMLError::XML_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Error loading plan: %s, Error: %s", plan_file_path.c_str(), document.ErrorName());
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Plan loaded from : %s", plan_file_path.c_str());

    auto goal_msg = capabilities2_msgs::action::Plan::Goal();

    capabilities2_xml_parser::convert_to_string(document, goal_msg.plan);

    RCLCPP_INFO(this->get_logger(), "Following plan was loaded :\n\n%s ", goal_msg.plan.c_str());

    RCLCPP_INFO(this->get_logger(), "Sending goal to the capabilities_fabric action server");

    auto send_goal_options = rclcpp_action::Client<capabilities2_msgs::action::Plan>::SendGoalOptions();

    // send goal options
    // goal response callback
    send_goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<capabilities2_msgs::action::Plan>::SharedPtr& goal_handle) {
          if (!goal_handle)
          {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
          }
        };

    // result callback
    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<capabilities2_msgs::action::Plan>::WrappedResult& result) {
      switch (result.code)
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }

      if (result.result->success)
      {
        RCLCPP_INFO(this->get_logger(), "Plan executed successfully");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Plan failed to complete");

        if (result.result->failed_elements.size() > 0)
        {
          RCLCPP_ERROR(this->get_logger(), "Plan failed due to incompatible XMLElements in the plan");

          for (const auto& failed_element : result.result->failed_elements)
            RCLCPP_ERROR(this->get_logger(), "Failed Elements : %s", failed_element.c_str());
        }
      }
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  /** File Path link */
  std::string plan_file_path;

  /** XML Document */
  tinyxml2::XMLDocument document;

  /** action client */
  rclcpp_action::Client<capabilities2_msgs::action::Plan>::SharedPtr client_ptr_;
};