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
	CapabilitiesExecutor(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
		: Node("Capabilities2_Executor", options)
	{
		control_tag_list.push_back("sequential");
		control_tag_list.push_back("parallel");
		control_tag_list.push_back("recovery");
		control_tag_list.push_back("roundrobin");
		// control_tag_list.push_back("behaviourtree");

		declare_parameter("read_file", false);
		read_file = get_parameter("read_file").as_bool();

		declare_parameter("plan_file_path", "plan.xml");
		plan_file_path = get_parameter("plan_file_path").as_string();

		using namespace std::placeholders;

		this->planner_server_ = rclcpp_action::create_server<capabilities2_msgs::action::Plan>(
			this,
			"~/capabilities",
			std::bind(&CapabilitiesExecutor::handle_goal, this, _1, _2),
			std::bind(&CapabilitiesExecutor::handle_cancel, this, _1),
			std::bind(&CapabilitiesExecutor::handle_accepted, this, _1));

		get_interfaces_client_ = this->create_client<capabilities2_msgs::srv::GetInterfaces>("~/get_interfaces");
		get_sem_interf_client_ = this->create_client<capabilities2_msgs::srv::GetSemanticInterfaces>("~/get_semantic_interfaces");

		establish_bond_client_ = this->create_client<capabilities2_msgs::srv::EstablishBond>("~/establish_bond");

		if (read_file)
		{
			// try to load the file
			tinyxml2::XMLError xml_status = document.LoadFile(plan_file_path.c_str());

			// check if the file loading failed
			if (xml_status != tinyxml2::XMLError::XML_SUCCESS)
			{
				RCLCPP_INFO(this->get_logger(), "Loading the file from path : %s failed", plan_file_path.c_str());

				rclcpp::shutdown();
			}

			execution_thread = std::make_unique<std::thread>(std::bind(&CapabilitiesExecutor::execute_plan, this));
		}
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
	rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const capabilities2_msgs::action::Plan::Goal> goal)
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

	rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<capabilities2_msgs::action::Plan>> goal_handle)
	{
		RCLCPP_INFO(this->get_logger(), "Received the request to cancel the plan");
		(void)goal_handle;

		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<capabilities2_msgs::action::Plan>> goal_handle)
	{
		execution_thread = std::make_unique<std::thread>(std::bind(&CapabilitiesExecutor::execute_plan, this));
	}

	bool verify_plan()
	{
		auto request_interface = std::make_shared<capabilities2_msgs::srv::GetInterfaces::Request>();
		auto request_sematic = std::make_shared<capabilities2_msgs::srv::GetSemanticInterfaces::Request>();

		auto result_future = get_interfaces_client_->async_send_request(request_interface);

		RCLCPP_INFO(this->get_logger(), "Requesting Interface information from Capabilities2 Server");

		if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
		{
			RCLCPP_INFO(this->get_logger(), "Failed to receive Interface information from Capabilities2 Server");
			return false;
		}

		RCLCPP_INFO(this->get_logger(), "Received Interface information from Capabilities2 Server");

		for (const auto &interface : result_future.get()->interfaces)
		{
			request_sematic->interface = interface;

			auto result_remantic_future = get_sem_interf_client_->async_send_request(request_sematic);

			RCLCPP_INFO(this->get_logger(), "Requesting Semantic Interface information from Capabilities2 Server for %s", interface.c_str());

			if (rclcpp::spin_until_future_complete(shared_from_this(), result_remantic_future) != rclcpp::FutureReturnCode::SUCCESS)
			{
				RCLCPP_INFO(this->get_logger(), "Failed to receive Semantic Interface information from Capabilities2 Server for %s", interface.c_str());
				return false;
			}

			RCLCPP_INFO(this->get_logger(), "Received Semantic Interface information from Capabilities2 Server for %s", interface.c_str());

			if (result_remantic_future.get()->semantic_interfaces.size() > 0)
			{
				for (const auto &semantic_interface : result_remantic_future.get()->semantic_interfaces)
				{
					interface_list.push_back(semantic_interface);
				}
			}
			else
			{
				interface_list.push_back(interface);
			}
		}

		// intialize a vector to accomodate elememnts from both
		std::vector<std::string> tag_list(interface_list.size() + control_tag_list.size());
		std::merge(interface_list.begin(), interface_list.end(), control_tag_list.begin(), control_tag_list.end(), tag_list.begin());

		if (!capabilities2_xml_parser::check_plan_tag(document))
			return false;

		tinyxml2::XMLElement* plan = capabilities2_xml_parser::get_plan(document);

		if (!capabilities2_xml_parser::check_tags(plan, interface_list, control_tag_list))
		{

		}
	}

	void execute_plan()
	{
		if (!verify_plan())
		{
			RCLCPP_INFO(this->get_logger(), "Plan verification failed");
		}

		// if (verify_plan())
		// {
		// 	establish_bonds();

		// 	use_capability();

		// 	trigger_capability();
		// }
		// else
		// {
		// 	request_updated_plan();
		// }

		// RCLCPP_INFO(this->get_logger(), "Executing goal");
		// rclcpp::Rate loop_rate(1);
		// const auto goal = goal_handle->get_goal();
		// auto feedback = std::make_shared<Fibonacci::Feedback>();
		// auto &sequence = feedback->partial_sequence;
		// sequence.push_back(0);
		// sequence.push_back(1);
		// auto result = std::make_shared<Fibonacci::Result>();

		// for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
		// {
		// 	// Check if there is a cancel request
		// 	if (goal_handle->is_canceling())
		// 	{
		// 		result->sequence = sequence;
		// 		goal_handle->canceled(result);
		// 		RCLCPP_INFO(this->get_logger(), "Goal canceled");
		// 		return;
		// 	}
		// 	// Update sequence
		// 	sequence.push_back(sequence[i] + sequence[i - 1]);
		// 	// Publish feedback
		// 	goal_handle->publish_feedback(feedback);
		// 	RCLCPP_INFO(this->get_logger(), "Publish feedback");

		// 	loop_rate.sleep();
		// }

		// // Check if goal is done
		// if (rclcpp::ok())
		// {
		// 	result->sequence = sequence;
		// 	goal_handle->succeed(result);
		// 	RCLCPP_INFO(this->get_logger(), "Goal succeeded");
		// }
	}

	/** File Path link */
	std::string plan_file_path;

	/** flag to select loading from file or accepting via action server */
	bool read_file;

	/** XML Document */
	tinyxml2::XMLDocument document;

	/** Execution Thread */
	std::shared_ptr<std::thread> execution_thread;

	/** Interface List */
	std::vector<std::string> interface_list;

	/** Control flow List */
	std::vector<std::string> control_tag_list;

	/** action server */
	std::shared_ptr<rclcpp_action::Server<capabilities2_msgs::action::Plan>> planner_server_;

	/** action server goal handle*/
	std::shared_ptr<rclcpp_action::ServerGoalHandle<capabilities2_msgs::action::Plan>> goal_handle;

	/** Get interfaces from capabilities server */
	rclcpp::Client<capabilities2_msgs::srv::GetInterfaces>::SharedPtr get_interfaces_client_;

	/** Get semantic interfaces from capabilities server */
	rclcpp::Client<capabilities2_msgs::srv::GetSemanticInterfaces>::SharedPtr get_sem_interf_client_;

	/** establish bond */
	rclcpp::Client<capabilities2_msgs::srv::EstablishBond>::SharedPtr establish_bond_client_;
};