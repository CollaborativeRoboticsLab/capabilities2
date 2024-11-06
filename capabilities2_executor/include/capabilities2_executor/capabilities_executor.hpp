#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <algorithm>
#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <capabilities2_executor/capabilities_xml_parser.hpp>

#include <capabilities2_msgs/msg/capability_connection.hpp>

#include <capabilities2_msgs/action/plan.hpp>
#include <capabilities2_msgs/action/connections.hpp>

#include <capabilities2_msgs/srv/establish_bond.hpp>
#include <capabilities2_msgs/srv/get_interfaces.hpp>
#include <capabilities2_msgs/srv/get_semantic_interfaces.hpp>
#include <capabilities2_msgs/srv/get_providers.hpp>

/**
 * @brief Capabilities Executor
 *
 * Capabilities executor node that provides a ROS client for the capabilities server.
 * Able to receive a XML file that implements a plan via action server that it exposes
 * or via a file read.
 *
 */

class CapabilitiesExecutor : public rclcpp::Node
{
public:
  CapabilitiesExecutor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("Capabilities2_Executor", options)
  {
    control_tag_list = capabilities2_xml_parser::get_control_list();

    this->planner_server_ = rclcpp_action::create_server<capabilities2_msgs::action::Plan>(
        this, "~/capabilities", std::bind(&CapabilitiesExecutor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CapabilitiesExecutor::handle_cancel, this, std::placeholders::_1),
        std::bind(&CapabilitiesExecutor::handle_accepted, this, std::placeholders::_1));

    this->client_capabilities_ = rclcpp_action::create_client<capabilities2_msgs::action::Connections>(this, "~/capabilities_fabric");

    get_interfaces_client_ = this->create_client<capabilities2_msgs::srv::GetInterfaces>("~/get_interfaces");
    get_sem_interf_client_ = this->create_client<capabilities2_msgs::srv::GetSemanticInterfaces>("~/get_semantic_interfaces");

    establish_bond_client_ = this->create_client<capabilities2_msgs::srv::EstablishBond>("~/establish_bond");
  }

private:
  /**
   * @brief Handle the goal request that comes in from client. returns whether goal is accepted or rejected
   *
   *
   * @param uuid uuid of the goal
   * @param goal pointer to the action goal message
   * @return rclcpp_action::GoalResponse
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const capabilities2_msgs::action::Plan::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received the goal request with the plan");
    (void)uuid;

    // try to parse the std::string plan from capabilities_msgs/Plan to the to a XMLDocument file
    tinyxml2::XMLError xml_status = document.Parse(goal->plan.c_str());

    // check if the file parsing failed
    if (xml_status != tinyxml2::XMLError::XML_SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Parsing the plan from goal message failed");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief Handle the goal cancel request that comes in from client.
   *
   * @param goal_handle pointer to the action goal handle
   * @return rclcpp_action::GoalResponse
   */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<capabilities2_msgs::action::Plan>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received the request to cancel the plan");
    (void)goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief Handle the goal accept event originating from handle_goal.
   *
   * @param goal_handle pointer to the action goal handle
   */
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<capabilities2_msgs::action::Plan>> goal_handle)
  {
    execution_thread = std::make_unique<std::thread>(std::bind(&CapabilitiesExecutor::execute_plan, this, std::placeholders::_1), goal_handle);
  }

  /**
   * @brief request the interfaces, semantic_interfaces and providers from the capabilities2 server
   *
   * @return `true` if interface retreival is successful,`false` otherwise
   */
  bool request_information()
  {
    // create request messages
    auto request_interface = std::make_shared<capabilities2_msgs::srv::GetInterfaces::Request>();
    auto request_sematic = std::make_shared<capabilities2_msgs::srv::GetSemanticInterfaces::Request>();
    auto request_providers = std::make_shared<capabilities2_msgs::srv::GetProviders::Request>();

    // request data from the server
    auto result_future = get_interfaces_client_->async_send_request(request_interface);

    RCLCPP_INFO(this->get_logger(), "Requesting Interface information from Capabilities2 Server");

    // wait until data is received
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Failed to receive Interface information from Capabilities2 Server");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Received Interface information from Capabilities2 Server");

    // request semantic interfaces available for each and every interface got from the server
    for (const auto& interface : result_future.get()->interfaces)
    {
      request_sematic->interface = interface;

      // request semantic interface from the server
      auto result_semantic_future = get_sem_interf_client_->async_send_request(request_sematic);

      RCLCPP_INFO(this->get_logger(), "Requesting Semantic Interface information from Capabilities2 Server for %s", interface.c_str());

      // wait until data is received
      if (rclcpp::spin_until_future_complete(shared_from_this(), result_semantic_future) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Failed to receive Semantic Interface information from Capabilities2 Server for %s", interface.c_str());
        return false;
      }

      RCLCPP_INFO(this->get_logger(), "Received Semantic Interface information from Capabilities2 Server for %s", interface.c_str());

      // add sematic interfaces to the list if available
      if (result_semantic_future.get()->semantic_interfaces.size() > 0)
      {
        for (const auto& semantic_interface : result_semantic_future.get()->semantic_interfaces)
        {
          interface_list.push_back(semantic_interface);

          // request providers of the semantic interface
          request_providers->interface = semantic_interface;
          request_providers->include_semantic = true;

          auto result_providers_future = get_providers_client_->async_send_request(request_providers);

          RCLCPP_INFO(this->get_logger(), "Requesting Providers information from Capabilities2 Server for semantic interface %s",
                      semantic_interface.c_str());

          // wait until data is received
          if (rclcpp::spin_until_future_complete(shared_from_this(), result_providers_future) != rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_INFO(this->get_logger(), "Failed to receive Providers information from Capabilities2 Server for %s", semantic_interface.c_str());
            return false;
          }

          RCLCPP_INFO(this->get_logger(), "Received Providers information from Capabilities2 Server for %s", semantic_interface.c_str());

          // add defualt provider to the list
          providers_list.push_back(result_providers_future.get()->default_provider);

          // add additional providers to the list if available
          if (result_providers_future.get()->providers.size() > 0)
          {
            for (const auto& provider : result_providers_future.get()->providers)
            {
              providers_list.push_back(provider);
            }
          }
        }
      }
      // if no semantic interfaces are availble for a given interface, add the interface instead
      else
      {
        interface_list.push_back(interface);

        // request providers of the semantic interface
        request_providers->interface = interface;
        request_providers->include_semantic = false;

        auto result_providers_future = get_providers_client_->async_send_request(request_providers);

        RCLCPP_INFO(this->get_logger(), "Requesting Providers information from Capabilities2 Server for semantic interface %s", interface.c_str());

        // wait until data is received
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_providers_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(this->get_logger(), "Failed to receive Providers information from Capabilities2 Server for %s", interface.c_str());
          return false;
        }

        RCLCPP_INFO(this->get_logger(), "Received Providers information from Capabilities2 Server for %s", interface.c_str());

        providers_list.push_back(result_providers_future.get()->default_provider);

        // add sematic interfaces to the list if available
        if (result_providers_future.get()->providers.size() > 0)
        {
          for (const auto& provider : result_providers_future.get()->providers)
          {
            providers_list.push_back(provider);
          }
        }
      }
    }

    return true;
  }

  /**
   * @brief verify the plan using received interfaces
   *
   * @return `true` if interface retreival is successful,`false` otherwise
   */
  bool verify_plan()
  {
    // intialize a vector to accomodate elements from both
    std::vector<std::string> tag_list(interface_list.size() + control_tag_list.size());
    std::merge(interface_list.begin(), interface_list.end(), control_tag_list.begin(), control_tag_list.end(), tag_list.begin());

    // verify whether document got 'plan' tags
    if (!capabilities2_xml_parser::check_plan_tag(document))
    {
      RCLCPP_INFO(this->get_logger(), "Execution plan is not compatible. Please recheck and update");
      return false;
    }

    // extract the components within the 'plan' tags
    tinyxml2::XMLElement* plan = capabilities2_xml_parser::get_plan(document);

    // verify whether the plan is valid
    if (!capabilities2_xml_parser::check_tags(plan, interface_list, providers_list, control_tag_list, rejected_list))
    {
      RCLCPP_INFO(this->get_logger(), "Execution plan is faulty. Please recheck and update");
      return false;
    }

    return true;
  }

  /**
   * @brief establish the bond with capabilities2 server
   *
   * @return `true` if bond establishing is successful,`false` otherwise
   */
  bool establish_bond()
  {
    // create bond establishing server request
    auto request_bond = std::make_shared<capabilities2_msgs::srv::EstablishBond::Request>();

    // send the request
    auto result_future = establish_bond_client_->async_send_request(request_bond);

    RCLCPP_INFO(this->get_logger(), "Requesting to establish bond with Capabilities2 Server");

    // wait for the result
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Failed to establish bond with Capabilities2 Server");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Established bond with Capabilities2 Server");

    bond_id = result_future.get()->bond_id;

    return true;
  }

  /**
   * @brief execute the plan
   *
   * @param server_goal_handle goal handle of the server
   */
  void execute_plan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<capabilities2_msgs::action::Plan>> server_goal_handle)
  {
    auto result = std::make_shared<capabilities2_msgs::action::Plan::Result>();

    // verify the plan
    if (!request_information())
    {
      RCLCPP_INFO(this->get_logger(), "Interface retreival failed");

      // TODO: improve with error codes
      result->success = false;
      server_goal_handle->canceled(result);

      RCLCPP_INFO(this->get_logger(), "Server Execution Cancelled");
    }

    // verify the plan
    if (!verify_plan())
    {
      RCLCPP_INFO(this->get_logger(), "Plan verification failed");

      if (rejected_list.size() > 0)
      {
        // TODO: improve with error codes
        result->success = false;

        for (const auto& rejected_element : rejected_list)
        {
          RCLCPP_ERROR(this->get_logger(), "Failed Events : %s", rejected_element.c_str());
          result->failed_elements.push_back(rejected_element);
        }

        server_goal_handle->canceled(result);
        RCLCPP_ERROR(this->get_logger(), "Server Execution Cancelled");
      }
      else
      {
        // TODO: improve with error codes
        result->success = false;
        server_goal_handle->canceled(result);
      }

      RCLCPP_INFO(this->get_logger(), "Server Execution Cancelled");
    }

    // extract the plan from the XMLDocument
    tinyxml2::XMLElement* plan = capabilities2_xml_parser::get_plan(document);

    // Extract the connections from the plan
    capabilities2_xml_parser::extract_connections(plan, connection_map);

    // estasblish the bond with the server
    if (!establish_bond())
    {
      RCLCPP_INFO(this->get_logger(), "Establishing bond failed");

      // TODO: improve with error codes
      result->success = false;
      server_goal_handle->canceled(result);

      RCLCPP_INFO(this->get_logger(), "Server Execution Cancelled");
    }

    auto connection_goal_msg = capabilities2_msgs::action::Connections::Goal();
    connection_goal_msg.bond_id = bond_id;
    connection_goal_msg.header.stamp = this->get_clock()->now();

    capabilities2_msgs::msg::CapabilityConnection connection_msg;

    for (const auto& [key, value] : connection_map)
    {
      RCLCPP_INFO(this->get_logger(), "Node : %i", key);

      if (capabilities2_xml_parser::convert_to_string(value.source.parameters, connection_msg.source.parameters))
      {
        connection_msg.source.capability = value.source.runner;
        connection_msg.source.provider = value.source.provider;

        RCLCPP_INFO(this->get_logger(), "Source Capability : %s", connection_msg.source.capability.c_str());
        RCLCPP_INFO(this->get_logger(), "Source Provider   : %s", connection_msg.source.provider.c_str());
        RCLCPP_INFO(this->get_logger(), "Source Parameters : %s", connection_msg.source.parameters.c_str());
      }
      else
      {
        connection_msg.source.capability = "";
        connection_msg.source.provider = "";
      }

      if (capabilities2_xml_parser::convert_to_string(value.target_on_start.parameters, connection_msg.target_on_start.parameters))
      {
        connection_msg.target_on_start.capability = value.target_on_start.runner;
        connection_msg.target_on_start.provider = value.target_on_start.provider;

        RCLCPP_INFO(this->get_logger(), "Triggered on start Capability : %s", connection_msg.target_on_start.capability.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on start Provider   : %s", connection_msg.target_on_start.provider.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on start Parameters : %s", connection_msg.target_on_start.parameters.c_str());
      }
      else
      {
        connection_msg.target_on_start.capability = "";
        connection_msg.target_on_start.provider = "";
      }

      if (capabilities2_xml_parser::convert_to_string(value.target_on_stop.parameters, connection_msg.target_on_stop.parameters))
      {
        connection_msg.target_on_stop.capability = value.target_on_stop.runner;
        connection_msg.target_on_stop.provider = value.target_on_stop.provider;

        RCLCPP_INFO(this->get_logger(), "Triggered on stop Capability : %s", connection_msg.target_on_stop.capability.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on stop Provider   : %s", connection_msg.target_on_stop.provider.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on stop Parameters : %s", connection_msg.target_on_stop.parameters.c_str());
      }
      else
      {
        connection_msg.target_on_stop.capability = "";
        connection_msg.target_on_stop.provider = "";
      }

      if (capabilities2_xml_parser::convert_to_string(value.target_on_success.parameters, connection_msg.target_on_success.parameters))
      {
        connection_msg.target_on_success.capability = value.target_on_success.runner;
        connection_msg.target_on_success.provider = value.target_on_success.provider;

        RCLCPP_INFO(this->get_logger(), "Triggered on success Capability : %s", connection_msg.target_on_success.capability.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on success Provider   : %s", connection_msg.target_on_success.provider.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on success Parameters : %s", connection_msg.target_on_success.parameters.c_str());
      }
      else
      {
        connection_msg.target_on_success.capability = "";
        connection_msg.target_on_success.provider = "";
      }

      if (capabilities2_xml_parser::convert_to_string(value.target_on_failure.parameters, connection_msg.target_on_failure.parameters))
      {
        connection_msg.target_on_failure.capability = value.target_on_failure.runner;
        connection_msg.target_on_failure.provider = value.target_on_failure.provider;

        RCLCPP_INFO(this->get_logger(), "Triggered on failure Capability : %s", connection_msg.target_on_failure.capability.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on failure Provider   : %s", connection_msg.target_on_failure.provider.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on failure Parameters : %s", connection_msg.target_on_failure.parameters.c_str());
      }
      else
      {
        connection_msg.target_on_failure.capability = "";
        connection_msg.target_on_failure.provider = "";
      }

      connection_goal_msg.connections.push_back(connection_msg);
    }

    auto send_goal_options = rclcpp_action::Client<capabilities2_msgs::action::Connections>::SendGoalOptions();

    // send goal options
    // goal response callback
    send_goal_options.goal_response_callback =
        [this, server_goal_handle](const rclcpp_action::ClientGoalHandle<capabilities2_msgs::action::Connections>::SharedPtr& goal_handle) {
          if (!goal_handle)
          {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");

            auto result = std::make_shared<capabilities2_msgs::action::Plan::Result>();

            // TODO: improve with error codes
            result->success = false;
            server_goal_handle->canceled(result);

            RCLCPP_INFO(this->get_logger(), "Server Execution Cancelled");
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
          }
        };

    // result callback
    send_goal_options.result_callback =
        [this, server_goal_handle](const rclcpp_action::ClientGoalHandle<capabilities2_msgs::action::Connections>::WrappedResult& result) {
          auto result_out = std::make_shared<capabilities2_msgs::action::Plan::Result>();

          switch (result.code)
          {
            case rclcpp_action::ResultCode::SUCCEEDED:
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(this->get_logger(), "Goal was aborted");

              // TODO: improve with error codes
              result_out->success = false;
              server_goal_handle->canceled(result_out);

              RCLCPP_INFO(this->get_logger(), "Server Execution Cancelled");

              return;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_ERROR(this->get_logger(), "Goal was canceled");

              // TODO: improve with error codes
              result_out->success = false;
              server_goal_handle->canceled(result_out);

              RCLCPP_INFO(this->get_logger(), "Server Execution Cancelled");

              return;
            default:
              RCLCPP_ERROR(this->get_logger(), "Unknown result code");

              // TODO: improve with error codes
              result_out->success = false;
              server_goal_handle->canceled(result_out);

              RCLCPP_INFO(this->get_logger(), "Server Execution Cancelled");
              return;
          }

          if (result.result->failed_connections.size() == 0)
          {
            // TODO: improve with error codes
            result_out->success = true;
            server_goal_handle->succeed(result_out);

            RCLCPP_INFO(this->get_logger(), "Server Execution Succeeded");
          }

          rclcpp::shutdown();
        };

    this->client_capabilities_->async_send_goal(connection_goal_msg, send_goal_options);
  }

private:
  /** File Path link */
  std::string plan_file_path;

  /** flag to select loading from file or accepting via action server */
  bool read_file;

  /** Bond ID between capabilities server and this client */
  std::string bond_id;

  /** XML Document */
  tinyxml2::XMLDocument document;

  /** vector of connections */
  std::map<int, capabilities2_executor::node_t> connection_map;

  /** Execution Thread */
  std::shared_ptr<std::thread> execution_thread;

  /** Interface List */
  std::vector<std::string> interface_list;

  /** Providers List */
  std::vector<std::string> providers_list;

  /** Control flow List */
  std::vector<std::string> control_tag_list;

  /** Invalid events list */
  std::vector<std::string> rejected_list;

  /** action client for connecting with capabilities server*/
  rclcpp_action::Client<capabilities2_msgs::action::Connections>::SharedPtr client_capabilities_;

  /** action server that exposes executor*/
  std::shared_ptr<rclcpp_action::Server<capabilities2_msgs::action::Plan>> planner_server_;

  /** action server goal handle*/
  std::shared_ptr<rclcpp_action::ServerGoalHandle<capabilities2_msgs::action::Plan>> goal_handle;

  /** Get interfaces from capabilities server */
  rclcpp::Client<capabilities2_msgs::srv::GetInterfaces>::SharedPtr get_interfaces_client_;

  /** Get semantic interfaces from capabilities server */
  rclcpp::Client<capabilities2_msgs::srv::GetSemanticInterfaces>::SharedPtr get_sem_interf_client_;

  /** Get providers from capabilities server */
  rclcpp::Client<capabilities2_msgs::srv::GetProviders>::SharedPtr get_providers_client_;

  /** establish bond */
  rclcpp::Client<capabilities2_msgs::srv::EstablishBond>::SharedPtr establish_bond_client_;
};