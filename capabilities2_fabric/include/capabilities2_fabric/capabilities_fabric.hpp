#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <algorithm>
#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <capabilities2_fabric/utils/xml_parser.hpp>
#include <capabilities2_fabric/utils/bond_client.hpp>

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
 * @brief Capabilities Fabric
 *
 * Capabilities fabric node that provides a ROS client for the capabilities server.
 * Able to receive a XML file that implements a plan via action server that it exposes
 *
 */

class CapabilitiesFabric : public rclcpp::Node
{
public:
  using Plan = capabilities2_msgs::action::Plan;
  using GoalHandlePlan = rclcpp_action::ServerGoalHandle<Plan>;

  using GetInterfaces = capabilities2_msgs::srv::GetInterfaces;
  using GetSemanticInterfaces = capabilities2_msgs::srv::GetSemanticInterfaces;
  using GetProviders = capabilities2_msgs::srv::GetProviders;
  using EstablishBond = capabilities2_msgs::srv::EstablishBond;
  using UseCapability = capabilities2_msgs::srv::UseCapability;
  using FreeCapability = capabilities2_msgs::srv::FreeCapability;
  using ConfigureCapability = capabilities2_msgs::srv::ConfigureCapability;
  using TriggerCapability = capabilities2_msgs::srv::TriggerCapability;

  using GetInterfacesClient = rclcpp::Client<GetInterfaces>;
  using GetSemanticInterfacesClient = rclcpp::Client<GetSemanticInterfaces>;
  using GetProvidersClient = rclcpp::Client<GetProviders>;
  using EstablishBondClient = rclcpp::Client<EstablishBond>;
  using UseCapabilityClient = rclcpp::Client<UseCapability>;
  using FreeCapabilityClient = rclcpp::Client<FreeCapability>;
  using ConfigureCapabilityClient = rclcpp::Client<ConfigureCapability>;
  using TriggerCapabilityClient = rclcpp::Client<TriggerCapability>;

  CapabilitiesFabric() : Node("Capabilities2_Fabric")
  {
    control_tag_list = xml_parser::get_control_list();
  }

  /**
   * @brief Initializer function
   *
   */
  void initialize()
  {
    this->planner_server_ = rclcpp_action::create_server<Plan>(
        this, "/capabilities_fabric", std::bind(&CapabilitiesFabric::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CapabilitiesFabric::handle_cancel, this, std::placeholders::_1),
        std::bind(&CapabilitiesFabric::handle_accepted, this, std::placeholders::_1));

    get_interfaces_client_ = this->create_client<GetInterfaces>("/capabilities/get_interfaces");
    get_sem_interf_client_ = this->create_client<GetSemanticInterfaces>("/capabilities/get_semantic_interfaces");
    get_providers_client_ = this->create_client<GetProviders>("/capabilities/get_providers");
    establish_bond_client_ = this->create_client<EstablishBond>("/capabilities/establish_bond");
    use_capability_client_ = this->create_client<UseCapability>("/capabilities/use_capability");
    free_capability_client_ = this->create_client<FreeCapability>("/capabilities/free_capability");
    trig_capability_client_ = this->create_client<TriggerCapability>("/capabilities/trigger_capability");
    conf_capability_client_ = this->create_client<ConfigureCapability>("/capabilities/configure_capability");

    // Wait for services to become available
    check_service(!get_interfaces_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/get_interfaces");
    check_service(!get_sem_interf_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/get_semantic_interfaces");
    check_service(!get_providers_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/get_providers");
    check_service(!establish_bond_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/establish_bond");
    check_service(!use_capability_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/use_capability");
    check_service(!free_capability_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/free_capability");
    check_service(!trig_capability_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/trigger_capability");
    check_service(!conf_capability_client_->wait_for_service(std::chrono::seconds(1)), "/capabilities/configure_capability");

    feedback_msg = std::make_shared<Plan::Feedback>();
    result_msg = std::make_shared<Plan::Result>();
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
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Plan::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received the goal request with the plan");
    (void)uuid;

    // try to parse the std::string plan from capabilities_msgs/Plan to the to a XMLDocument file
    tinyxml2::XMLError xml_status = document.Parse(goal->plan.c_str());

    // check if the file parsing failed
    if (xml_status != tinyxml2::XMLError::XML_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Parsing the plan from goal message failed");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Plan parsed and accepted");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief Handle the goal cancel request that comes in from client.
   *
   * @param goal_handle pointer to the action goal handle
   * @return rclcpp_action::GoalResponse
   */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Received the request to cancel the plan");
    (void)goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief Handle the goal accept event originating from handle_goal.
   *
   * @param goal_handle pointer to the action goal handle
   */
  void handle_accepted(const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    execution(goal_handle);
  }

  /**
   * @brief Trigger execution
   */
  void execution(const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    print_and_feedback(goal_handle, "Execution started", false);

    expected_providers_ = 0;
    completed_providers_ = 0;

    expected_interfaces_ = 0;
    completed_interfaces_ = 0;

    expected_capabilities_ = 0;
    completed_capabilities_ = 0;

    expected_configurations_ = 0;
    completed_configurations_ = 0;

    getInterfaces(goal_handle);
  }

  /**
   * @brief Get Interfaces
   */
  void getInterfaces(const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    print_and_feedback(goal_handle, "Requesting Interface information", false);

    auto request_interface = std::make_shared<GetInterfaces::Request>();

    // request data from the server
    auto result_future = get_interfaces_client_->async_send_request(request_interface, [this, goal_handle](GetInterfacesClient::SharedFuture future) {
      auto result = std::make_shared<Plan::Result>();

      if (!future.valid())
      {
        print_and_result(goal_handle, "Failed to get Interface information. Server execution cancelled", false);
        return;
      }

      auto response = future.get();
      expected_interfaces_ = response->interfaces.size();

      status = "Received Interfaces. Requsting " + std::to_string(expected_interfaces_) + " semantic interface information";
      print_and_feedback(goal_handle, status, false);

      // Request each interface recursively for Semantic interfaces
      getSemanticInterfaces(response->interfaces, goal_handle);
    });
  }

  /**
   * @brief Get Semantic Interfaces
   */
  void getSemanticInterfaces(const std::vector<std::string>& interfaces, const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    std::string requested_interface = interfaces[completed_interfaces_];

    status = "Requesting semantic interfaces for " + requested_interface;
    print_and_feedback(goal_handle, status, true);

    auto request_semantic = std::make_shared<GetSemanticInterfaces::Request>();
    request_semantic->interface = requested_interface;

    // request semantic interface from the server
    auto result_semantic_future = get_sem_interf_client_->async_send_request(
        request_semantic, [this, goal_handle, interfaces, requested_interface](GetSemanticInterfacesClient::SharedFuture future) {
          if (!future.valid())
          {
            print_and_result(goal_handle, "Failed to get Semantic Interface information. Server execution cancelled", false);
            return;
          }

          completed_interfaces_++;
          auto response = future.get();

          // if semantic interfaces are availble for a given interface, add the semantic interface
          if (response->semantic_interfaces.size() > 0)
          {
            for (const auto& semantic_interface : response->semantic_interfaces)
            {
              interface_list.push_back(semantic_interface);
              is_semantic_list.push_back(true);

              status = (completed_interfaces_) + "/" + std::to_string(expected_interfaces_) + " : Received " + semantic_interface + " for " +
                       requested_interface + ". So added " + semantic_interface;
              print_and_feedback(goal_handle, status, false);
            }
          }
          // if no semantic interfaces are availble for a given interface, add the interface instead
          else
          {
            interface_list.push_back(requested_interface);
            is_semantic_list.push_back(false);

            status = std::to_string(completed_interfaces_) + "/" + std::to_string(expected_interfaces_) + " : Received none for " +
                     requested_interface + ". So added " + requested_interface;
            print_and_feedback(goal_handle, status, false);
          }

          if (completed_interfaces_ != expected_interfaces_)
          {
            // Request next interface recursively for Semantic interfaces
            getSemanticInterfaces(interfaces, goal_handle);
          }
          else
          {
            print_and_feedback(goal_handle, "Received all requested Interface information", true);

            expected_providers_ = interface_list.size();

            status = "Requsting Provider information for " + std::to_string(expected_providers_) + " providers";
            print_and_feedback(goal_handle, status, false);

            // request providers from the interfaces in the interfaces_list
            getProvider(interface_list, is_semantic_list, goal_handle);
          }
        });
  }

  /**
   * @brief Get Providers
   *
   */
  void getProvider(const std::vector<std::string>& interfaces, const std::vector<bool>& is_semantic,
                   const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    std::string requested_interface = interfaces[completed_providers_];
    bool semantic_flag = is_semantic[completed_providers_];

    status = "Requesting provider for " + requested_interface;
    print_and_feedback(goal_handle, status, true);

    auto request_providers = std::make_shared<GetProviders::Request>();

    // request providers of the semantic interface
    request_providers->interface = requested_interface;
    request_providers->include_semantic = semantic_flag;

    auto result_providers_future = get_providers_client_->async_send_request(
        request_providers, [this, is_semantic, requested_interface, interfaces, goal_handle](GetProvidersClient::SharedFuture future) {
          if (!future.valid())
          {
            status = "Did not retrieve providers for interface: " + requested_interface;
            print_and_result(goal_handle, status, false);
            return;
          }

          completed_providers_++;
          auto response = future.get();

          if (response->default_provider != "")
          {
            // add defualt provider to the list
            providers_list.push_back(response->default_provider);

            status = std::to_string(completed_providers_) + "/" + std::to_string(expected_providers_) + " : Received " + response->default_provider +
                     " for " + requested_interface + ". So added " + response->default_provider;
            print_and_feedback(goal_handle, status, false);
          }

          // add additional providers to the list if available
          if (response->providers.size() > 0)
          {
            for (const auto& provider : response->providers)
            {
              providers_list.push_back(provider);

              status = std::to_string(completed_providers_) + "/" + std::to_string(expected_providers_) + " : Received and added " + provider +
                       " for " + requested_interface;
              print_and_feedback(goal_handle, status, false);
            }
          }
          else
          {
            status = std::to_string(completed_providers_) + "/" + std::to_string(expected_providers_) + " : No providers for " + requested_interface;
            print_and_feedback(goal_handle, status, false);
          }

          // Check if all expected calls are completed before calling verify_plan
          if (completed_providers_ != expected_providers_)
          {
            // request providers for the next interface in the interfaces_list
            getProvider(interfaces, is_semantic, goal_handle);
          }
          else
          {
            print_and_feedback(goal_handle, "All requested interface, semantic interface and provider data recieved", true);

            verify_and_continue(goal_handle);
          }
        });
  }

  /**
   * @brief Verify the plan and continue the execution
   *
   */
  void verify_and_continue(const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    print_and_feedback(goal_handle, "Verifying the plan", false);

    // verify the plan
    if (!verify_plan(goal_handle))
    {
      print_and_feedback(goal_handle, "Plan verification failed", false);

      if (rejected_list.size() > 0)
      {
        // TODO: improve with error codes
        auto result = std::make_shared<Plan::Result>();
        result->success = false;
        result->message = "Plan verification failed. There are mismatched events";

        for (const auto& rejected_element : rejected_list)
        {
          RCLCPP_ERROR(this->get_logger(), "Failed Events : %s", rejected_element.c_str());
          result->failed_elements.push_back(rejected_element);
        }

        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Server Execution Cancelled");
      }
      else
      {
        // TODO: improve with error codes
        print_and_result(goal_handle, "Plan verification failed. Server Execution Cancelled.", false);
      }

      RCLCPP_ERROR(this->get_logger(), "Server Execution Cancelled");
    }

    print_and_feedback(goal_handle, "Plan verification successful", false);

    // extract the plan from the XMLDocument
    tinyxml2::XMLElement* plan = xml_parser::get_plan(document);

    print_and_feedback(goal_handle, "Plan conversion successful", false);

    // Extract the connections from the plan
    xml_parser::extract_connections(plan, connection_map);

    print_and_feedback(goal_handle, "Connection extraction successful", false);

    // estasblish the bond with the server
    establish_bond(goal_handle);
  }

  /**
   * @brief verify the plan using received interfaces
   *
   * @return `true` if interface retreival is successful,`false` otherwise
   */
  bool verify_plan(const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    auto feedback = std::make_shared<Plan::Feedback>();
    auto result = std::make_shared<Plan::Result>();

    // intialize a vector to accomodate elements from both
    std::vector<std::string> tag_list(interface_list.size() + control_tag_list.size());
    std::merge(interface_list.begin(), interface_list.end(), control_tag_list.begin(), control_tag_list.end(), tag_list.begin());

    // verify whether document got 'plan' tags
    if (!xml_parser::check_plan_tag(document))
    {
      print_and_result(goal_handle, "Execution plan is not compatible. Please recheck and update", false);
      return false;
    }

    print_and_feedback(goal_handle, "'Plan' tag checking successful", false);

    // extract the components within the 'plan' tags
    tinyxml2::XMLElement* plan = xml_parser::get_plan(document);

    print_and_feedback(goal_handle, "Plan extraction complete", false);

    // verify whether the plan is valid
    if (!xml_parser::check_tags(this->get_logger(), plan, interface_list, providers_list, control_tag_list, rejected_list))
    {
      print_and_result(goal_handle, "Execution plan is faulty. Please recheck and update", false);
      return false;
    }

    print_and_feedback(goal_handle, "Checking tags successful", false);
    return true;
  }

  /**
   * @brief establish the bond with capabilities2 server
   *
   */
  void establish_bond(const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    print_and_feedback(goal_handle, "Requesting bond id", false);

    // create bond establishing server request
    auto request_bond = std::make_shared<EstablishBond::Request>();

    // send the request
    auto result_future = establish_bond_client_->async_send_request(request_bond, [this, goal_handle](EstablishBondClient::SharedFuture future) {
      if (!future.valid())
      {
        print_and_result(goal_handle, "Failed to retrieve the bond id. Server execution cancelled", false);
        return;
      }

      auto response = future.get();
      bond_id_ = response->bond_id;

      status = "Received the bond id : " + bond_id_;
      print_and_feedback(goal_handle, status, false);

      bond_client_ = std::make_unique<BondClient>(shared_from_this(), bond_id_);
      bond_client_->start();

      expected_capabilities_ = connection_map.size();

      status = "Requsting start of " + std::to_string(expected_capabilities_) + "capabilities";
      print_and_feedback(goal_handle, status, false);

      use_capability(connection_map, goal_handle);
    });
  }

  /**
   * @brief Request use of capability from capabilities2 server
   *
   * @param capabilities capability list to be started
   * @param provider provider of the capability
   */
  void use_capability(std::map<int, capabilities2::node_t>& capabilities, const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    std::string capability = capabilities[completed_capabilities_].source.runner;
    std::string provider = capabilities[completed_capabilities_].source.provider;

    auto request_use = std::make_shared<UseCapability::Request>();
    request_use->capability = capability;
    request_use->preferred_provider = provider;
    request_use->bond_id = bond_id_;

    status = "Starting capability of Node " + std::to_string(completed_capabilities_) + " : " + capabilities[completed_capabilities_].source.runner;
    print_and_feedback(goal_handle, status, true);

    // send the request
    auto result_future =
        use_capability_client_->async_send_request(request_use, [this, goal_handle, capability, provider](UseCapabilityClient::SharedFuture future) {
          if (!future.valid())
          {
            status = "Failed to Use capability " + capability + " from " + provider + ". Server Execution Cancelled";
            print_and_result(goal_handle, status, false);

            // release all capabilities that were used since not all started successfully
            for (const auto& [key, value] : connection_map)
            {
              status = "Freeing capability of Node " + std::to_string(key) + " named " + value.source.runner;
              print_and_feedback(goal_handle, status, false);

              free_capability(value.source.runner, goal_handle);
            }

            return;
          }

          completed_capabilities_++;

          auto response = future.get();

          status = std::to_string(completed_capabilities_) + "/" + std::to_string(expected_capabilities_) + " : start succeessful";
          print_and_feedback(goal_handle, status, true);

          // Check if all expected calls are completed before calling verify_plan
          if (completed_capabilities_ == expected_capabilities_)
          {
            print_and_feedback(goal_handle, "All requested capabilities have been started. Configuring the capabilities with events", true);

            expected_configurations_ = connection_map.size();

            status = "Requsting capability configuration for " + std::to_string(expected_configurations_) + "capabilities";
            print_and_feedback(goal_handle, status, true);

            configure_capabilities(connection_map, goal_handle);
          }
          else
          {
            use_capability(connection_map, goal_handle);
          }
        });
  }

  /**
   * @brief Request use of capability from capabilities2 server
   *
   * @param capability capability name to be started
   */
  void free_capability(const std::string& capability, const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    auto request_free = std::make_shared<FreeCapability::Request>();
    request_free->capability = capability;
    request_free->bond_id = bond_id_;

    // send the request
    auto result_future =
        free_capability_client_->async_send_request(request_free, [this, goal_handle, capability](FreeCapabilityClient::SharedFuture future) {
          if (!future.valid())
          {
            status = "Failed to free capability " + capability;
            print_and_result(goal_handle, status, false);
            return;
          }

          auto response = future.get();

          status = "Successfully freed capability " + capability;
          print_and_feedback(goal_handle, status, true);

          bond_client_->stop();
        });
  }

  /**
   * @brief Request use of capability from capabilities2 server
   */
  void configure_capabilities(std::map<int, capabilities2::node_t>& capabilities, const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    auto request_configure = std::make_shared<ConfigureCapability::Request>();

    status = "Configuring capability of Node " + std::to_string(completed_configurations_) + " named " +
             capabilities[completed_configurations_].source.runner;
    print_and_feedback(goal_handle, status, true);

    if (xml_parser::convert_to_string(capabilities[completed_configurations_].source.parameters, request_configure->source.parameters))
    {
      request_configure->source.capability = capabilities[completed_configurations_].source.runner;
      request_configure->source.provider = capabilities[completed_configurations_].source.provider;

      RCLCPP_INFO(this->get_logger(), "Source capability : %s from provider : %s", request_configure->source.capability.c_str(),
                  request_configure->source.provider.c_str());
    }
    else
    {
      request_configure->source.capability = "";
      request_configure->source.provider = "";
    }

    if (xml_parser::convert_to_string(capabilities[completed_configurations_].target_on_start.parameters,
                                                    request_configure->target_on_start.parameters))
    {
      request_configure->target_on_start.capability = capabilities[completed_configurations_].target_on_start.runner;
      request_configure->target_on_start.provider = capabilities[completed_configurations_].target_on_start.provider;

      RCLCPP_INFO(this->get_logger(), "--> on_start capability : %s from provider : %s", request_configure->target_on_start.capability.c_str(),
                  request_configure->target_on_start.provider.c_str());
    }
    else
    {
      request_configure->target_on_start.capability = "";
      request_configure->target_on_start.provider = "";
    }

    if (xml_parser::convert_to_string(capabilities[completed_configurations_].target_on_stop.parameters,
                                                    request_configure->target_on_stop.parameters))
    {
      request_configure->target_on_stop.capability = capabilities[completed_configurations_].target_on_stop.runner;
      request_configure->target_on_stop.provider = capabilities[completed_configurations_].target_on_stop.provider;

      RCLCPP_INFO(this->get_logger(), "--> on stop capability : %s from provider : %s", request_configure->target_on_stop.capability.c_str(),
                  request_configure->target_on_stop.provider.c_str());
    }
    else
    {
      request_configure->target_on_stop.capability = "";
      request_configure->target_on_stop.provider = "";
    }

    if (xml_parser::convert_to_string(capabilities[completed_configurations_].target_on_success.parameters,
                                                    request_configure->target_on_success.parameters))
    {
      request_configure->target_on_success.capability = capabilities[completed_configurations_].target_on_success.runner;
      request_configure->target_on_success.provider = capabilities[completed_configurations_].target_on_success.provider;

      RCLCPP_INFO(this->get_logger(), "--> on success capability : %s from provider : %s", request_configure->target_on_success.capability.c_str(),
                  request_configure->target_on_success.provider.c_str());
    }
    else
    {
      request_configure->target_on_success.capability = "";
      request_configure->target_on_success.provider = "";
    }

    if (xml_parser::convert_to_string(capabilities[completed_configurations_].target_on_failure.parameters,
                                                    request_configure->target_on_failure.parameters))
    {
      request_configure->target_on_failure.capability = capabilities[completed_configurations_].target_on_failure.runner;
      request_configure->target_on_failure.provider = capabilities[completed_configurations_].target_on_failure.provider;

      RCLCPP_INFO(this->get_logger(), "--> on failure capability : %s from provider : %s", request_configure->target_on_failure.capability.c_str(),
                  request_configure->target_on_failure.provider.c_str());
    }
    else
    {
      request_configure->target_on_failure.capability = "";
      request_configure->target_on_failure.provider = "";
    }

    std::string source_capability = capabilities[completed_configurations_].source.runner;

    // send the request
    auto result_future = conf_capability_client_->async_send_request(
        request_configure, [this, goal_handle, source_capability](ConfigureCapabilityClient::SharedFuture future) {
          if (!future.valid())
          {
            status = "Failed to configure capability :" + source_capability + ". Server execution cancelled";
            print_and_result(goal_handle, status, false);
            return;
          }

          completed_configurations_++;

          auto response = future.get();

          status = std::to_string(completed_configurations_) + "/" + std::to_string(expected_configurations_) +
                   " : Successfully configured capability : " + source_capability;
          print_and_feedback(goal_handle, status, true);

          // Check if all expected calls are completed before calling verify_plan
          if (completed_configurations_ == expected_configurations_)
          {
            print_and_feedback(goal_handle, "All requested capabilities have been configured. Triggering the first capability", true);

            trigger_first_node(goal_handle);
          }
          else
          {
            configure_capabilities(connection_map, goal_handle);
          }
        });
  }

  /**
   * @brief Trigger the first node
   */
  void trigger_first_node(const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    auto request_trigger = std::make_shared<TriggerCapability::Request>();

    std::string parameter_string;
    xml_parser::convert_to_string(connection_map[0].source.parameters, parameter_string);
    request_trigger->capability = connection_map[0].source.runner;
    request_trigger->parameters = parameter_string;

    // send the request
    auto result_future =
        trig_capability_client_->async_send_request(request_trigger, [this, goal_handle](TriggerCapabilityClient::SharedFuture future) {
          if (!future.valid())
          {
            status = "Failed to trigger capability " + connection_map[0].source.runner;
            print_and_result(goal_handle, status, false);
            return;
          }

          auto response = future.get();

          status = "Successfully triggered capability " + connection_map[0].source.runner;
          print_and_feedback(goal_handle, status, false);

          print_and_result(goal_handle, "Successfully launched capabilities2 fabric", true);
        });
  }

  void check_service(bool wait_for_logic, const std::string& service_name)
  {
    while (wait_for_logic)
    {
      std::string failed = service_name + " not available";
      RCLCPP_ERROR(this->get_logger(), failed.c_str());
      rclcpp::shutdown();
      return;
    }

    std::string success = service_name + " connected";
    RCLCPP_INFO(this->get_logger(), success.c_str());
  }

  void print_and_feedback(const std::shared_ptr<GoalHandlePlan> goal_handle, const std::string& text, bool newline)
  {
    feedback_msg->progress = text;
    goal_handle->publish_feedback(feedback_msg);

    if (newline)
      RCLCPP_INFO(this->get_logger(), "");

    RCLCPP_INFO(this->get_logger(), feedback_msg->progress.c_str());
  }

  void print_and_result(const std::shared_ptr<GoalHandlePlan> goal_handle, const std::string& text, bool success)
  {
    result_msg->success = success;
    result_msg->message = text;

    if (success)
    {
      goal_handle->succeed(result_msg);
      RCLCPP_INFO(this->get_logger(), result_msg->message.c_str());
    }
    else
    {
      goal_handle->abort(result_msg);
      RCLCPP_ERROR(this->get_logger(), result_msg->message.c_str());
    }
  }

private:
  /** File Path link */
  std::string plan_file_path;

  /** flag to select loading from file or accepting via action server */
  bool read_file;

  int expected_interfaces_;
  int completed_interfaces_;

  int expected_providers_;
  int completed_providers_;

  int expected_capabilities_;
  int completed_capabilities_;

  int expected_configurations_;
  int completed_configurations_;

  /** status message string */
  std::string status;

  /** Bond id */
  std::string bond_id_;

  /** Bond between capabilities server and this client */
  std::shared_ptr<BondClient> bond_client_;

  /** XML Document */
  tinyxml2::XMLDocument document;

  /** vector of connections */
  std::map<int, capabilities2::node_t> connection_map;

  /** Interface List */
  std::vector<bool> is_semantic_list;

  /** Interface List */
  std::vector<std::string> interface_list;

  /** Providers List */
  std::vector<std::string> providers_list;

  /** Control flow List */
  std::vector<std::string> control_tag_list;

  /** Invalid events list */
  std::vector<std::string> rejected_list;

  /** Feedback message for plan action server*/
  std::shared_ptr<Plan::Feedback> feedback_msg;

  /** Result message for plan action server*/
  std::shared_ptr<Plan::Result> result_msg;

  /** action server that exposes executor*/
  std::shared_ptr<rclcpp_action::Server<Plan>> planner_server_;

  /** action server goal handle*/
  std::shared_ptr<GoalHandlePlan> goal_handle;

  /** Get interfaces from capabilities server */
  GetInterfacesClient::SharedPtr get_interfaces_client_;

  /** Get semantic interfaces from capabilities server */
  GetSemanticInterfacesClient::SharedPtr get_sem_interf_client_;

  /** Get providers from capabilities server */
  GetProvidersClient::SharedPtr get_providers_client_;

  /** establish bond */
  EstablishBondClient::SharedPtr establish_bond_client_;

  /** use an selected capability */
  UseCapabilityClient::SharedPtr use_capability_client_;

  /** free an selected capability */
  FreeCapabilityClient::SharedPtr free_capability_client_;

  /** configure an selected capability */
  ConfigureCapabilityClient::SharedPtr conf_capability_client_;

  /** trigger an selected capability */
  TriggerCapabilityClient::SharedPtr trig_capability_client_;
};