#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <algorithm>
#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <capabilities2_executor/capabilities_xml_parser.hpp>

#include <capabilities2_msgs/action/plan.hpp>

#include <capabilities2_msgs/srv/establish_bond.hpp>
#include <capabilities2_msgs/srv/get_interfaces.hpp>
#include <capabilities2_msgs/srv/get_semantic_interfaces.hpp>
#include <capabilities2_msgs/srv/get_providers.hpp>
#include <capabilities2_msgs/srv/use_capability.hpp>
#include <capabilities2_msgs/srv/free_capability.hpp>
#include <capabilities2_msgs/srv/configure_capability.hpp>
#include <capabilities2_msgs/srv/trigger_capability.hpp>

/**
 * @brief Capabilities Executor
 *
 * Capabilities executor node that provides a ROS client for the capabilities server.
 * Able to receive a XML file that implements a plan via action server that it exposes
 * or via a file read.
 *
 */

class CapabilitiesFabric : public rclcpp::Node
{
public:
  CapabilitiesFabric() : Node("Capabilities2_Executor")
  {
    control_tag_list = capabilities2_xml_parser::get_control_list();

    this->planner_server_ = rclcpp_action::create_server<capabilities2_msgs::action::Plan>(
        this, "~/capabilities_fabric", std::bind(&CapabilitiesFabric::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CapabilitiesFabric::handle_cancel, this, std::placeholders::_1),
        std::bind(&CapabilitiesFabric::handle_accepted, this, std::placeholders::_1));

    get_interfaces_client_ = this->create_client<capabilities2_msgs::srv::GetInterfaces>("~/get_interfaces");
    get_sem_interf_client_ = this->create_client<capabilities2_msgs::srv::GetSemanticInterfaces>("~/get_semantic_interfaces");
    get_providers_client_ = this->create_client<capabilities2_msgs::srv::GetProviders>("~/get_providers");
    establish_bond_client_ = this->create_client<capabilities2_msgs::srv::EstablishBond>("~/establish_bond");
    use_capability_client_ = this->create_client<capabilities2_msgs::srv::UseCapability>("~/use_capability");
    free_capability_client_ = this->create_client<capabilities2_msgs::srv::FreeCapability>("~/free_capability");
    trigger_capability_client_ = this->create_client<capabilities2_msgs::srv::TriggerCapability>("~/trigger_capability");
    configure_capability_client_ = this->create_client<capabilities2_msgs::srv::ConfigureCapability>("~/configure_capability");
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
    fabric_thread = std::make_unique<std::thread>(std::bind(&CapabilitiesFabric::execute_plan, this, std::placeholders::_1), goal_handle);
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
   * @brief Request use of capability from capabilities2 server
   *
   * @param capability capability name to be started
   * @param provider provider of the capability
   * @param bond_id bond_id for the capability
   *
   * @return `true` if use of capability is successful,`false` otherwise
   */
  bool use_capability(const std::string& capability, const std::string& provider, const std::string& bond_id)
  {
    auto request_use = std::make_shared<capabilities2_msgs::srv::UseCapability::Request>();

    request_use->capability = capability;
    request_use->preferred_provider = provider;
    request_use->bond_id = bond_id;

    // send the request
    auto result_future = use_capability_client_->async_send_request(request_use);

    RCLCPP_INFO(this->get_logger(), "Requesting to use %s capability from Capabilities2 Server", capability.c_str());

    // wait for the result
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Failed to use requested capability from Capabilities2 Server");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully used requested capability from Capabilities2 Server");

    return true;
  }

  /**
   * @brief Request use of capability from capabilities2 server
   *
   * @param capability capability name to be started
   * @param bond_id bond_id for the capability
   *
   * @return `true` if use of capability is successful,`false` otherwise
   */
  bool free_capability(const std::string& capability, const std::string& bond_id)
  {
    auto request_free = std::make_shared<capabilities2_msgs::srv::FreeCapability::Request>();

    request_free->capability = capability;
    request_free->bond_id = bond_id;

    // send the request
    auto result_future = free_capability_client_->async_send_request(request_free);

    RCLCPP_INFO(this->get_logger(), "Requesting to free %s capability from Capabilities2 Server", capability.c_str());

    // wait for the result
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Failed to free requested capability from Capabilities2 Server");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully freed requested capability from Capabilities2 Server");

    return true;
  }

  /**
   * @brief Request use of capability from capabilities2 server
   *
   * @return `true` if configuration of capability is successful,`false` otherwise
   */
  bool configure_capabilities()
  {
    for (const auto& [key, value] : connection_map)
    {
      auto request_configure = std::make_shared<capabilities2_msgs::srv::ConfigureCapability::Request>();

      RCLCPP_INFO(this->get_logger(), "Configuring Node : %i", key);

      if (capabilities2_xml_parser::convert_to_string(value.source.parameters, request_configure->source.parameters))
      {
        request_configure->source.capability = value.source.runner;
        request_configure->source.provider = value.source.provider;

        RCLCPP_INFO(this->get_logger(), "Source Capability : %s", request_configure->source.capability.c_str());
        RCLCPP_INFO(this->get_logger(), "Source Provider   : %s", request_configure->source.provider.c_str());
        RCLCPP_INFO(this->get_logger(), "Source Parameters : %s", request_configure->source.parameters.c_str());
      }
      else
      {
        request_configure->source.capability = "";
        request_configure->source.provider = "";
      }

      if (capabilities2_xml_parser::convert_to_string(value.target_on_start.parameters, request_configure->target_on_start.parameters))
      {
        request_configure->target_on_start.capability = value.target_on_start.runner;
        request_configure->target_on_start.provider = value.target_on_start.provider;

        RCLCPP_INFO(this->get_logger(), "Triggered on start Capability : %s", request_configure->target_on_start.capability.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on start Provider   : %s", request_configure->target_on_start.provider.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on start Parameters : %s", request_configure->target_on_start.parameters.c_str());
      }
      else
      {
        request_configure->target_on_start.capability = "";
        request_configure->target_on_start.provider = "";
      }

      if (capabilities2_xml_parser::convert_to_string(value.target_on_stop.parameters, request_configure->target_on_stop.parameters))
      {
        request_configure->target_on_stop.capability = value.target_on_stop.runner;
        request_configure->target_on_stop.provider = value.target_on_stop.provider;

        RCLCPP_INFO(this->get_logger(), "Triggered on stop Capability : %s", request_configure->target_on_stop.capability.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on stop Provider   : %s", request_configure->target_on_stop.provider.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on stop Parameters : %s", request_configure->target_on_stop.parameters.c_str());
      }
      else
      {
        request_configure->target_on_stop.capability = "";
        request_configure->target_on_stop.provider = "";
      }

      if (capabilities2_xml_parser::convert_to_string(value.target_on_success.parameters, request_configure->target_on_success.parameters))
      {
        request_configure->target_on_success.capability = value.target_on_success.runner;
        request_configure->target_on_success.provider = value.target_on_success.provider;

        RCLCPP_INFO(this->get_logger(), "Triggered on success Capability : %s", request_configure->target_on_success.capability.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on success Provider   : %s", request_configure->target_on_success.provider.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on success Parameters : %s", request_configure->target_on_success.parameters.c_str());
      }
      else
      {
        request_configure->target_on_success.capability = "";
        request_configure->target_on_success.provider = "";
      }

      if (capabilities2_xml_parser::convert_to_string(value.target_on_failure.parameters, request_configure->target_on_failure.parameters))
      {
        request_configure->target_on_failure.capability = value.target_on_failure.runner;
        request_configure->target_on_failure.provider = value.target_on_failure.provider;

        RCLCPP_INFO(this->get_logger(), "Triggered on failure Capability : %s", request_configure->target_on_failure.capability.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on failure Provider   : %s", request_configure->target_on_failure.provider.c_str());
        RCLCPP_INFO(this->get_logger(), "Triggered on failure Parameters : %s", request_configure->target_on_failure.parameters.c_str());
      }
      else
      {
        request_configure->target_on_failure.capability = "";
        request_configure->target_on_failure.provider = "";
      }

      // send the request
      auto result_future = configure_capability_client_->async_send_request(request_configure);

      RCLCPP_INFO(this->get_logger(), "Configuring %s capability from Capabilities2 Server", request_configure->source.capability.c_str());

      // wait for the result
      if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Failed to configure requested capability from Capabilities2 Server");
        return false;
      }

      RCLCPP_INFO(this->get_logger(), "Successfully configured requested capability from Capabilities2 Server");
    }

    return true;
  }

  /**
   * @brief Trigger the first node
   *
   * @return `true` if triggering is successful,`false` otherwise
   */
  bool trigger_first_node()
  {
    auto request_trigger = std::make_shared<capabilities2_msgs::srv::TriggerCapability::Request>();

    std::string parameter_string;
    capabilities2_xml_parser::convert_to_string(connection_map[0].source.parameters, parameter_string);

    request_trigger->capability = connection_map[0].source.runner;
    request_trigger->parameters = parameter_string;

    // send the request
    auto result_future = trigger_capability_client_->async_send_request(request_trigger);

    RCLCPP_INFO(this->get_logger(), "Requesting to trigger %s capability from Capabilities2 Server", connection_map[0].source.runner.c_str());

    // wait for the result
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Failed to trigger requested capability from Capabilities2 Server");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully triggered requested capability from Capabilities2 Server");

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

    RCLCPP_INFO(this->get_logger(), "Execution started");

    // verify the plan
    if (!request_information())
    {
      RCLCPP_INFO(this->get_logger(), "Interface retreival failed");

      // TODO: improve with error codes
      result->success = false;
      server_goal_handle->canceled(result);

      RCLCPP_INFO(this->get_logger(), "Server Execution Cancelled");
    }

    RCLCPP_INFO(this->get_logger(), "Interface retreival successful");

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

    RCLCPP_INFO(this->get_logger(), "Plan verification successful");

    // extract the plan from the XMLDocument
    tinyxml2::XMLElement* plan = capabilities2_xml_parser::get_plan(document);

    RCLCPP_INFO(this->get_logger(), "Plan conversion successful");

    // Extract the connections from the plan
    capabilities2_xml_parser::extract_connections(plan, connection_map);

    RCLCPP_INFO(this->get_logger(), "Connection extraction successful");

    // estasblish the bond with the server
    if (!establish_bond())
    {
      RCLCPP_INFO(this->get_logger(), "Establishing bond failed");

      // TODO: improve with error codes
      result->success = false;
      server_goal_handle->canceled(result);

      RCLCPP_INFO(this->get_logger(), "Server Execution Cancelled");
    }

    RCLCPP_INFO(this->get_logger(), "Bond establishment successful");

    // start all runners and interfaces that the connections depend on
    for (const auto& [key, value] : connection_map)
    {
      RCLCPP_INFO(this->get_logger(), "Starting capability of Node : %i", key);

      // start the capability with bond id.
      if (use_capability(value.source.runner, value.source.provider, bond_id))
      {
        // capability started succeesfully
        RCLCPP_INFO(get_logger(), "Capability started: %s", value.source.runner.c_str());
      }
      else
      {
        std::string parameter_string;
        capabilities2_xml_parser::convert_to_string(value.source.parameters, parameter_string);

        // capability failed. so add it to failed connections to be sent to the action client
        result->failed_elements.push_back(parameter_string);
        RCLCPP_ERROR(get_logger(), "Capability failed: %s", parameter_string.c_str());
      }
    }

    RCLCPP_INFO(this->get_logger(), "All capability starting successful");

    // check if there are any failed elements of the plan
    if (result->failed_elements.size() > 0)
    {
      // there are failed connections. so cancel the action server process. and let the action
      // client know that about the failed connection
      server_goal_handle->canceled(result);

      // free the capabilites that were started since action execution failed
      for (const auto& [key, value] : connection_map)
      {
        free_capability(value.source.runner, bond_id);
        RCLCPP_ERROR(get_logger(), "Capability freed due to failure: %s", value.source.runner.c_str());
      }
    }

    // configuring the capabilities
    if (!configure_capabilities())
    {
      RCLCPP_INFO(this->get_logger(), "Configuring capabilities failed");

      // TODO: improve with error codes
      result->success = false;
      server_goal_handle->canceled(result);

      RCLCPP_INFO(this->get_logger(), "Server Execution Cancelled");
    }

    RCLCPP_INFO(this->get_logger(), "Capability configuration successful");

    // triggering the first node to start the fabric
    if (!trigger_first_node())
    {
      RCLCPP_INFO(this->get_logger(), "Triggering first capability failed");

      // TODO: improve with error codes
      result->success = false;
      server_goal_handle->canceled(result);

      RCLCPP_INFO(this->get_logger(), "Server Execution Cancelled");
    }
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
  std::shared_ptr<std::thread> fabric_thread;

  /** Interface List */
  std::vector<std::string> interface_list;

  /** Providers List */
  std::vector<std::string> providers_list;

  /** Control flow List */
  std::vector<std::string> control_tag_list;

  /** Invalid events list */
  std::vector<std::string> rejected_list;

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

  /** use an selected capability */
  rclcpp::Client<capabilities2_msgs::srv::UseCapability>::SharedPtr use_capability_client_;

  /** free an selected capability */
  rclcpp::Client<capabilities2_msgs::srv::FreeCapability>::SharedPtr free_capability_client_;

  /** configure an selected capability */
  rclcpp::Client<capabilities2_msgs::srv::ConfigureCapability>::SharedPtr configure_capability_client_;

  /** trigger an selected capability */
  rclcpp::Client<capabilities2_msgs::srv::TriggerCapability>::SharedPtr trigger_capability_client_;
};