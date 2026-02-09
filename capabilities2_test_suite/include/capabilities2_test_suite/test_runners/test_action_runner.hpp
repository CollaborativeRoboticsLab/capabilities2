#pragma once

#include <thread>
#include <chrono>
#include <string>
#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <capabilities2_runner/action_runner.hpp>
#include <capabilities2_test_suite/action/fibonacci.hpp>

using namespace std::chrono_literals;

namespace capabilities2_runner
{

class TestActionRunner : public ActionRunner<capabilities2_test_suite::action::Fibonacci>
{
public:
  TestActionRunner() : ActionRunner()
  {
  }

  /**
   * @brief Starter function for starting the action runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param bond_id bond id to be used for emitting events
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& bond_id) override
  {
    init_action(node, run_config, "test/action_server");

    // emit started event with empty bond_id and parameters to indicate runner is ready
    emit_started(bond_id); 
  }

protected:
  /**
   * @brief This generate goal function overrides the generate_goal() function from ActionRunner()
   * @param parameters XMLElement that contains parameters in the format
   '<Event name=follow_waypoints provider=WaypointRunner x='$value' y='$value' />'
   * @return ActionT::Goal the generated goal
   */
  virtual capabilities2_test_suite::action::Fibonacci::Goal generate_goal(capabilities2::CapabilityOptions options, int id) override
  {
    capabilities2_test_suite::action::Fibonacci::Goal goal_msg;

    // extract order parameter from options

    if (options.has_value("order"))
    {
      goal_msg.order = std::any_cast<int>(options.get_value("order"));
			RCLCPP_INFO(node_->get_logger(), "generated goal with order: %d for event %d", goal_msg.order, id);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "missing parameter: order");
      throw runner_exception("missing parameter: order");
		}

		return goal_msg;
  }

  /**
   * @brief This generate feedback function overrides the generate_feedback() function from ActionRunner()
   *
   * @param msg feedback message from the action server
   * @return std::string of feedback information
   */
  virtual std::string
  generate_feedback(const typename capabilities2_test_suite::action::Fibonacci::Feedback::ConstSharedPtr msg) override
  {
    std::string feedback = "Current sequence: [";
		for (auto number : msg->sequence)
		{
			feedback += std::to_string(number) + " ";
		}
		feedback += "]";
		return feedback;
	}
    return "";
  }
};

}  // namespace capabilities2_runner