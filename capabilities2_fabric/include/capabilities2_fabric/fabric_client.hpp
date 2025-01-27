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

#include <capabilities2_utils/xml_parser.hpp>
#include <capabilities2_utils/status_client.hpp>

#include <capabilities2_msgs/action/plan.hpp>

#include <capabilities2_msgs/srv/set_fabric_plan.hpp>
#include <capabilities2_msgs/srv/cancel_fabric_plan.hpp>
#include <capabilities2_msgs/srv/get_fabric_status.hpp>

/**
 * @brief Capabilities Executor File Parser
 *
 * Capabilities Executor File Parser node that provides a ROS client for the capabilities executor.
 * Will read an XML file that implements a plan and send it to the server
 */

class CapabilitiesFabricClient : public rclcpp::Node
{
public:
  enum fabric_status
  {
    IDLE,
    RUNNING,
    CANCELED,
    ABORTED,
    FAILED,
    SUCCEEDED
  };

  using Plan = capabilities2_msgs::action::Plan;
  using GoalHandlePlan = rclcpp_action::ClientGoalHandle<Plan>;

  using GetFabricStatus = capabilities2_msgs::srv::GetFabricStatus;
  using SetFabricPlan = capabilities2_msgs::srv::SetFabricPlan;
  using CancelFabricPlan = capabilities2_msgs::srv::CancelFabricPlan;

  CapabilitiesFabricClient() : Node("Capabilities2_Fabric_Client")
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
    fabric_state = fabric_status::IDLE;

    status_ = std::make_shared<StatusClient>(shared_from_this(), "capabilities_fabric_client", "/status/capabilities_fabric_client");

    status_server_ =
        this->create_service<GetFabricStatus>("/capabilities_fabric/get_status", std::bind(&CapabilitiesFabricClient::getStatusCallback, this,
                                                                                           std::placeholders::_1, std::placeholders::_2));

    plan_server_ = this->create_service<SetFabricPlan>(
        "/capabilities_fabric/set_plan", std::bind(&CapabilitiesFabricClient::setPlanCallback, this, std::placeholders::_1, std::placeholders::_2));

    cancel_server_ =
        this->create_service<CancelFabricPlan>("/capabilities_fabric/cancel_plan", std::bind(&CapabilitiesFabricClient::cancelPlanCallback, this,
                                                                                             std::placeholders::_1, std::placeholders::_2));

    // Create the action client for capabilities_fabric after the node is fully constructed
    this->planner_client_ = rclcpp_action::create_client<Plan>(shared_from_this(), "/capabilities_fabric");

    if (!this->planner_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      status_->error("Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    status_->info("Sucessfully connected to the capabilities_fabric action server");

    // try to load the file
    tinyxml2::XMLError xml_status = document.LoadFile(plan_file_path.c_str());

    // check if the file loading failed
    if (xml_status != tinyxml2::XMLError::XML_SUCCESS)
    {
      status = "Error loading plan: " + plan_file_path + ", Error: " + document.ErrorName();
      status_->error(status);
      rclcpp::shutdown();
    }

    status = "Plan loaded from : " + plan_file_path;
    status_->info(status);

    send_goal(document);
  }

private:
  void send_goal(tinyxml2::XMLDocument& document_xml)
  {
    auto goal_msg = Plan::Goal();

    xml_parser::convert_to_string(document_xml, goal_msg.plan);

    status = "Following plan was loaded :\n\n " + goal_msg.plan;
    status_->info(status);

    status_->info("Sending goal to the capabilities_fabric action server");

    auto send_goal_options = rclcpp_action::Client<Plan>::SendGoalOptions();

    // send goal options
    // goal response callback
    send_goal_options.goal_response_callback = [this](const GoalHandlePlan::SharedPtr& goal_handle) {
      if (!goal_handle)
      {
        status_->error("Goal was rejected by server");
        fabric_state = fabric_status::FAILED;
      }
      else
      {
        status_->info("Goal accepted by server, waiting for result");
        goal_handle_ = goal_handle;
        fabric_state = fabric_status::RUNNING;
      }
    };

    // result callback
    send_goal_options.result_callback = [this](const GoalHandlePlan::WrappedResult& result) {
      switch (result.code)
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          fabric_state = fabric_status::SUCCEEDED;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          status_->error("Goal was aborted");
          fabric_state = fabric_status::ABORTED;
          break;
        case rclcpp_action::ResultCode::CANCELED:
          status_->error("Goal was canceled");
          fabric_state = fabric_status::CANCELED;
          break;
        default:
          status_->error("Unknown result code");
          fabric_state = fabric_status::FAILED;
          break;
      }

      if (result.result->success)
      {
        status_->info("Plan executed successfully");
      }
      else
      {
        status_->error("Plan failed to complete");

        if (result.result->failed_elements.size() > 0)
        {
          status_->error("Plan failed due to incompatible XMLElements in the plan");

          for (const auto& failed_element : result.result->failed_elements)
            status_->error_element(failed_element);
        }
      }
    };

    this->planner_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void getStatusCallback(const std::shared_ptr<GetFabricStatus::Request> request, std::shared_ptr<GetFabricStatus::Response> response)
  {
    if (fabric_state == fabric_status::IDLE)
    {
      response->status = GetFabricStatus::Response::FABRIC_IDLE;
    }
    else if (fabric_state == fabric_status::RUNNING)
    {
      response->status = GetFabricStatus::Response::FABRIC_RUNNING;
    }
    else if (fabric_state == fabric_status::CANCELED)
    {
      response->status = GetFabricStatus::Response::FABRIC_CANCELED;
    }
    else if (fabric_state == fabric_status::ABORTED)
    {
      response->status = GetFabricStatus::Response::FABRIC_ABORTED;
    }
    else if (fabric_state == fabric_status::FAILED)
    {
      response->status = GetFabricStatus::Response::FABRIC_FAILED;
    }
    else if (fabric_state == fabric_status::SUCCEEDED)
    {
      response->status = GetFabricStatus::Response::FABRIC_SUCCEEDED;
    }
    else
    {
      response->status = GetFabricStatus::Response::UNKNOWN;
    }
  }

  void setPlanCallback(const std::shared_ptr<SetFabricPlan::Request> request, std::shared_ptr<SetFabricPlan::Response> response)
  {
    status_->info("Received the request with a plan");

    // try to parse the std::string plan from capabilities_msgs/Plan to the to a XMLDocument file
    tinyxml2::XMLError xml_status = document.Parse(request->plan.c_str());

    // check if the file parsing failed
    if (xml_status != tinyxml2::XMLError::XML_SUCCESS)
    {
      status_->info("Parsing the plan from service request message failed");
      response->success = false;
    }

    status_->info("Plan parsed and accepted");

    response->success = true;

    send_goal(document);
  }

  void cancelPlanCallback(const std::shared_ptr<CancelFabricPlan::Request> request, std::shared_ptr<CancelFabricPlan::Response> response)
  {
    if (fabric_state == fabric_status::RUNNING)
    {
      this->planner_client_->async_cancel_goal(goal_handle_);
    }

    response->success = true;
  }

private:
  /** File Path link */
  std::string plan_file_path;

  /** Status message */
  std::string status;

  /** XML Document */
  tinyxml2::XMLDocument document;

  /** action client */
  rclcpp_action::Client<Plan>::SharedPtr planner_client_;

  /** Handles status message sending and printing to logging */
  std::shared_ptr<StatusClient> status_;

  /** Goal handle for action client control */
  GoalHandlePlan::SharedPtr goal_handle_;

  /** server to get the status of the capabilities2 fabric */
  rclcpp::Service<GetFabricStatus>::SharedPtr status_server_;

  /** server to set a new plan to the capabilities2 fabric */
  rclcpp::Service<SetFabricPlan>::SharedPtr plan_server_;

  /** server to cancel the current plan in the capabilities2 fabric */
  rclcpp::Service<CancelFabricPlan>::SharedPtr cancel_server_;

  /** Status of the fabric */
  fabric_status fabric_state;
};