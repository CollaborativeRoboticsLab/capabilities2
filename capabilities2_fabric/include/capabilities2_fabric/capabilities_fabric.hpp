#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <algorithm>
#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <capabilities2_fabric/capabilities_xml_parser.hpp>
#include <capabilities2_fabric/structs/bond_client.hpp>

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
    control_tag_list = capabilities2_xml_parser::get_control_list();
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
    trigger_capability_client_ = this->create_client<TriggerCapability>("/capabilities/trigger_capability");
    configure_capability_client_ = this->create_client<ConfigureCapability>("/capabilities/configure_capability");

    // Wait for services to become available
    while (!get_interfaces_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "/capabilities/get_interfaces not available");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "/capabilities/get_interfaces connected");

    while (!get_sem_interf_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "/capabilities/get_semantic_interfaces not available");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "/capabilities/get_semantic_interfaces connected");

    while (!get_providers_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "/capabilities/get_providers not available");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "/capabilities/get_providers connected");

    while (!establish_bond_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "/capabilities/establish_bond not available");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "/capabilities/establish_bond connected");

    while (!use_capability_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "/capabilities/use_capability not available");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "/capabilities/use_capability connected");

    while (!free_capability_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "/capabilities/free_capability not available");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "/capabilities/free_capability connected");

    while (!trigger_capability_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "/capabilities/trigger_capability not available");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "/capabilities/trigger_capability connected");

    while (!configure_capability_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "/capabilities/configure_capability not available");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "/capabilities/configure_capability connected");
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
    auto feedback = std::make_shared<Plan::Feedback>();

    feedback->progress = "Execution started";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

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
    auto feedback = std::make_shared<Plan::Feedback>();

    feedback->progress = "Requesting Interface information";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

    auto request_interface = std::make_shared<GetInterfaces::Request>();

    // request data from the server
    auto result_future =
        get_interfaces_client_->async_send_request(request_interface, [this, goal_handle, feedback](GetInterfacesClient::SharedFuture future) {
          auto result = std::make_shared<Plan::Result>();

          if (!future.valid())
          {
            result->success = false;
            result->message = "Failed to get Interface information";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), result->message.c_str());

            RCLCPP_ERROR(this->get_logger(), "Server Execution Cancelled");
            return;
          }

          auto response = future.get();

          feedback->progress = "Received Interface information";
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

          expected_interfaces_ = response->interfaces.size();

          RCLCPP_INFO(this->get_logger(), "Requsting Semantic Interface information for %d interfaces", expected_interfaces_);

          // Request each interface recursively for Semantic interfaces
          getSemanticInterfaces(response->interfaces, goal_handle);
        });
  }

  /**
   * @brief Get Semantic Interfaces
   */
  void getSemanticInterfaces(const std::vector<std::string>& interfaces, const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    auto feedback = std::make_shared<Plan::Feedback>();

    std::string requested_interface = interfaces[completed_interfaces_];

    RCLCPP_INFO(this->get_logger(), "");
    feedback->progress = "Requesting semantic interfaces for " + requested_interface;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

    auto request_semantic = std::make_shared<GetSemanticInterfaces::Request>();
    request_semantic->interface = requested_interface;

    // request semantic interface from the server
    auto result_semantic_future = get_sem_interf_client_->async_send_request(
        request_semantic, [this, goal_handle, feedback, interfaces, requested_interface](GetSemanticInterfacesClient::SharedFuture future) {
          if (!future.valid())
          {
            auto result = std::make_shared<Plan::Result>();
            result->success = false;
            result->message = "Failed to get Semantic Interface information";
            goal_handle->abort(result);

            RCLCPP_ERROR(this->get_logger(), "Failed to get Semantic Interface information. Server Execution Cancelled");
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

              feedback->progress = std::to_string(completed_interfaces_) + "/" + std::to_string(expected_interfaces_) + " : Received " +
                                   semantic_interface + " for " + requested_interface + ". So added " + semantic_interface;
              goal_handle->publish_feedback(feedback);
              RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());
            }
          }
          // if no semantic interfaces are availble for a given interface, add the interface instead
          else
          {
            interface_list.push_back(requested_interface);
            is_semantic_list.push_back(false);

            feedback->progress = std::to_string(completed_interfaces_) + "/" + std::to_string(expected_interfaces_) + " : Received none for " +
                                 requested_interface + ". So added " + requested_interface;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());
          }

          if (completed_interfaces_ != expected_interfaces_)
          {
            // Request next interface recursively for Semantic interfaces
            getSemanticInterfaces(interfaces, goal_handle);
          }
          else
          {
            feedback->progress = "Received all requested Interface information";
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "");
            RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

            expected_providers_ = interface_list.size();

            RCLCPP_INFO(this->get_logger(), "Requsting Provider information for %d providers", expected_providers_);

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

    auto feedback = std::make_shared<Plan::Feedback>();

    RCLCPP_INFO(this->get_logger(), "");
    feedback->progress = "Requesting provider for " + requested_interface;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

    auto request_providers = std::make_shared<GetProviders::Request>();

    // request providers of the semantic interface
    request_providers->interface = requested_interface;
    request_providers->include_semantic = semantic_flag;

    auto result_providers_future = get_providers_client_->async_send_request(
        request_providers, [this, is_semantic, requested_interface, interfaces, goal_handle, feedback](GetProvidersClient::SharedFuture future) {
          if (!future.valid())
          {
            auto result = std::make_shared<Plan::Result>();
            result->success = false;
            result->message = "Did not retrieve providers for interface: " + requested_interface;
            goal_handle->abort(result);

            RCLCPP_ERROR(this->get_logger(), result->message.c_str());
            return;
          }

          completed_providers_++;
          auto response = future.get();

          if (response->default_provider != "")
          {
            // add defualt provider to the list
            providers_list.push_back(response->default_provider);

            feedback->progress = std::to_string(completed_providers_) + "/" + std::to_string(expected_providers_) + " : Received " +
                                 response->default_provider + " for " + requested_interface + ". So added " + response->default_provider;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());
          }

          // add additional providers to the list if available
          if (response->providers.size() > 0)
          {
            for (const auto& provider : response->providers)
            {
              providers_list.push_back(provider);
              feedback->progress = std::to_string(completed_providers_) + "/" + std::to_string(expected_providers_) + " : Received and added " +
                                   provider + " for " + requested_interface;
              goal_handle->publish_feedback(feedback);
              RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());
            }
          }
          else
          {
            feedback->progress =
                std::to_string(completed_providers_) + "/" + std::to_string(expected_providers_) + " : No providers for " + requested_interface;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());
          }

          // Check if all expected calls are completed before calling verify_plan
          if (completed_providers_ != expected_providers_)
          {
            // request providers for the next interface in the interfaces_list
            getProvider(interfaces, is_semantic, goal_handle);
          }
          else
          {
            auto feedback = std::make_shared<Plan::Feedback>();

            RCLCPP_INFO(this->get_logger(), "");
            feedback->progress = "All requested interface, semantic interface and provider data recieved";
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

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
    auto feedback = std::make_shared<Plan::Feedback>();
    feedback->progress = "Verifying the plan";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

    // verify the plan
    if (!verify_plan(goal_handle))
    {
      feedback->progress = "Plan verification failed";
      goal_handle->publish_feedback(feedback);
      RCLCPP_ERROR(this->get_logger(), feedback->progress.c_str());

      auto result = std::make_shared<Plan::Result>();

      if (rejected_list.size() > 0)
      {
        // TODO: improve with error codes
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
        result->success = false;
        result->message = "Plan verification failed.";
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Server Execution Cancelled");
      }

      RCLCPP_ERROR(this->get_logger(), "Server Execution Cancelled");
    }

    feedback->progress = "Plan verification successful";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

    // extract the plan from the XMLDocument
    tinyxml2::XMLElement* plan = capabilities2_xml_parser::get_plan(document);

    feedback->progress = "Plan conversion successful";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

    // Extract the connections from the plan
    capabilities2_xml_parser::extract_connections(plan, connection_map);

    feedback->progress = "Connection extraction successful";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

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
    if (!capabilities2_xml_parser::check_plan_tag(document))
    {
      result->success = false;
      result->message = "Execution plan is not compatible. Please recheck and update";
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), result->message.c_str());
      return false;
    }

    feedback->progress = "'Plan' tag checking successful";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

    // extract the components within the 'plan' tags
    tinyxml2::XMLElement* plan = capabilities2_xml_parser::get_plan(document);

    feedback->progress = "Plan extraction complete";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

    // verify whether the plan is valid
    if (!capabilities2_xml_parser::check_tags(this->get_logger(), plan, interface_list, providers_list, control_tag_list, rejected_list))
    {
      result->success = false;
      result->message = "Execution plan is faulty. Please recheck and update";
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), result->message.c_str());
      return false;
    }

    feedback->progress = "Checking tags successful";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());
    return true;
  }

  /**
   * @brief establish the bond with capabilities2 server
   *
   */
  void establish_bond(const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    auto feedback = std::make_shared<Plan::Feedback>();
    feedback->progress = "Requesting bond id";
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

    // create bond establishing server request
    auto request_bond = std::make_shared<EstablishBond::Request>();

    // send the request
    auto result_future =
        establish_bond_client_->async_send_request(request_bond, [this, goal_handle, feedback](EstablishBondClient::SharedFuture future) {
          if (!future.valid())
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive the bond id");

            auto result = std::make_shared<Plan::Result>();
            result->success = false;
            result->message = "Failed to retrieve the bond id";
            RCLCPP_ERROR(this->get_logger(), "Server Execution Cancelled");

            goal_handle->abort(result);
            return;
          }

          auto response = future.get();
          bond_id_ = response->bond_id;

          feedback->progress = "Received the bond id : " + bond_id_;
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

          bond_client_ = std::make_unique<BondClient>(shared_from_this(), bond_id_);
          bond_client_->start();

          expected_capabilities_ = connection_map.size();

          RCLCPP_INFO(this->get_logger(), "Requsting start of %d capabilities", expected_capabilities_);

          use_capability(connection_map, goal_handle);
        });
  }

  /**
   * @brief Request use of capability from capabilities2 server
   *
   * @param capabilities capability list to be started
   * @param provider provider of the capability
   */
  void use_capability(std::map<int, capabilities2_executor::node_t>& capabilities, const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    auto feedback = std::make_shared<Plan::Feedback>();

    std::string capability = capabilities[completed_capabilities_].source.runner;
    std::string provider = capabilities[completed_capabilities_].source.provider;

    auto request_use = std::make_shared<UseCapability::Request>();
    request_use->capability = capability;
    request_use->preferred_provider = provider;
    request_use->bond_id = bond_id_;

    feedback->progress =
        "Starting capability of Node " + std::to_string(completed_capabilities_) + " named " + capabilities[completed_capabilities_].source.runner;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

    // send the request
    auto result_future = use_capability_client_->async_send_request(request_use, [this, goal_handle, feedback, capability,
                                                                                  provider](UseCapabilityClient::SharedFuture future) {
      if (!future.valid())
      {
        auto result = std::make_shared<Plan::Result>();
        result->success = false;
        result->message = "Failed to Use capability " + capability + " from " + provider;
        goal_handle->abort(result);

        RCLCPP_ERROR(this->get_logger(), result->message.c_str());
        RCLCPP_ERROR(this->get_logger(), "Server Execution Cancelled");

        // release all capabilities that were used since not all started successfully
        for (const auto& [key, value] : connection_map)
        {
          feedback->progress = "Freeing capability of Node " + std::to_string(key) + " named " + value.source.runner;
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

          free_capability(value.source.runner, goal_handle);
        }

        return;
      }

      completed_capabilities_++;

      auto response = future.get();

      feedback->progress = std::to_string(completed_capabilities_) + "/" + std::to_string(expected_capabilities_) + " : Capability start succeessful";
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

      // Check if all expected calls are completed before calling verify_plan
      if (completed_capabilities_ == expected_capabilities_)
      {
        RCLCPP_INFO(this->get_logger(), "");
        feedback->progress = "All requested capabilities have been used";
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

        feedback->progress = "Configuring the capabilities with events";
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

        expected_configurations_ = connection_map.size();

        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Requsting capability configuration for %d capabilities", expected_configurations_);

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
    auto feedback = std::make_shared<Plan::Feedback>();

    auto request_free = std::make_shared<FreeCapability::Request>();
    request_free->capability = capability;
    request_free->bond_id = bond_id_;

    // send the request
    auto result_future = free_capability_client_->async_send_request(
        request_free, [this, goal_handle, capability, feedback](FreeCapabilityClient::SharedFuture future) {
          if (!future.valid())
          {
            auto result = std::make_shared<Plan::Result>();
            result->success = false;
            result->message = "Failed to free capability " + capability;
            goal_handle->abort(result);

            RCLCPP_ERROR(this->get_logger(), result->message.c_str());
            return;
          }

          auto response = future.get();
          feedback->progress = "Successfully freed capability " + capability;
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

          bond_client_->stop();
        });
  }

  /**
   * @brief Request use of capability from capabilities2 server
   */
  void configure_capabilities(std::map<int, capabilities2_executor::node_t>& capabilities, const std::shared_ptr<GoalHandlePlan> goal_handle)
  {
    auto feedback = std::make_shared<Plan::Feedback>();
    auto request_configure = std::make_shared<ConfigureCapability::Request>();

    RCLCPP_INFO(this->get_logger(), "");
    feedback->progress = "Configuring capability of Node " + std::to_string(completed_configurations_) + " named " +
                         capabilities[completed_configurations_].source.runner;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

    if (capabilities2_xml_parser::convert_to_string(capabilities[completed_configurations_].source.parameters, request_configure->source.parameters))
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

    if (capabilities2_xml_parser::convert_to_string(capabilities[completed_configurations_].target_on_start.parameters,
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

    if (capabilities2_xml_parser::convert_to_string(capabilities[completed_configurations_].target_on_stop.parameters,
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

    if (capabilities2_xml_parser::convert_to_string(capabilities[completed_configurations_].target_on_success.parameters,
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

    if (capabilities2_xml_parser::convert_to_string(capabilities[completed_configurations_].target_on_failure.parameters,
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
    auto result_future = configure_capability_client_->async_send_request(
        request_configure, [this, goal_handle, source_capability, feedback](ConfigureCapabilityClient::SharedFuture future) {
          if (!future.valid())
          {
            auto result = std::make_shared<Plan::Result>();
            result->success = false;
            result->message = "Failed to configure capability :" + source_capability;
            goal_handle->abort(result);

            RCLCPP_ERROR(this->get_logger(), result->message.c_str());
            RCLCPP_ERROR(this->get_logger(), "Server Execution Cancelled");
            return;
          }

          completed_configurations_++;

          auto response = future.get();

          feedback->progress = std::to_string(completed_configurations_) + "/" + std::to_string(expected_configurations_) +
                               " : Successfully configured capability : " + source_capability;
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), "");
          RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

          // Check if all expected calls are completed before calling verify_plan
          if (completed_configurations_ == expected_configurations_)
          {
            RCLCPP_INFO(this->get_logger(), "");
            feedback->progress = "All requested capabilities have been configured";
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

            RCLCPP_INFO(this->get_logger(), "");
            feedback->progress = "Triggering the first capability";
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

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
    auto feedback = std::make_shared<Plan::Feedback>();

    auto request_trigger = std::make_shared<TriggerCapability::Request>();

    std::string parameter_string;
    capabilities2_xml_parser::convert_to_string(connection_map[0].source.parameters, parameter_string);
    request_trigger->capability = connection_map[0].source.runner;
    request_trigger->parameters = parameter_string;

    // send the request
    auto result_future =
        trigger_capability_client_->async_send_request(request_trigger, [this, goal_handle, feedback](TriggerCapabilityClient::SharedFuture future) {
          if (!future.valid())
          {
            auto result = std::make_shared<Plan::Result>();
            result->success = false;
            result->message = "Failed to trigger capability " + connection_map[0].source.runner;
            goal_handle->abort(result);

            RCLCPP_ERROR(this->get_logger(), result->message.c_str());
            return;
          }

          auto response = future.get();
          feedback->progress = "Successfully triggered capability " + connection_map[0].source.runner;
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(this->get_logger(), feedback->progress.c_str());

          auto result = std::make_shared<Plan::Result>();
          result->success = true;
          result->message = "Successfully launched capabilities2 fabric";
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), result->message.c_str());
        });
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

  /** Bond id */
  std::string bond_id_;

  /** Bond between capabilities server and this client */
  std::shared_ptr<BondClient> bond_client_;

  /** XML Document */
  tinyxml2::XMLDocument document;

  /** vector of connections */
  std::map<int, capabilities2_executor::node_t> connection_map;

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
  ConfigureCapabilityClient::SharedPtr configure_capability_client_;

  /** trigger an selected capability */
  TriggerCapabilityClient::SharedPtr trigger_capability_client_;
};