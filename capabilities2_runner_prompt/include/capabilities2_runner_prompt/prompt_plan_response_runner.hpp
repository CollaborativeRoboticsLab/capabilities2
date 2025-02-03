#pragma once

#include <thread>

#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <capabilities2_runner/action_runner.hpp>
#include <capabilities2_msgs/action/plan.hpp>

namespace capabilities2_runner
{

/**
 * @brief Executor runner class
 *
 * Class to run capabilities2 executor action based capability
 *
 */
class PromptPlanResponseRunner : public ActionRunner<capabilities2_msgs::action::Plan>
{
public:
  PromptPlanResponseRunner() : ActionRunner()
  {
  }

  /**
   * @brief Starter function for starting the action runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(Event&)> print) override
  {
    init_action(node, run_config, "capabilities_fabric", print);
  }

protected:
  /**
   * @brief This generate goal function overrides the generate_goal() function from ActionRunner()
   * @param parameters XMLElement that contains parameters in the format
   '<Event name=follow_waypoints provider=WaypointRunner x='$value' y='$value' />'
   * @return ActionT::Goal the generated goal
   */
  virtual capabilities2_msgs::action::Plan::Goal generate_goal(tinyxml2::XMLElement* parameters, int id) override
  {
    tinyxml2::XMLElement* planElement = parameters->FirstChildElement("ReceievdPlan");

    auto goal_msg = capabilities2_msgs::action::Plan::Goal();

    // Check if the element was found and has text content
    if (planElement && planElement->GetText())
    {
      goal_msg.plan = std::string(planElement->GetText());
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
  generate_feedback(const typename capabilities2_msgs::action::Plan::Feedback::ConstSharedPtr msg) override
  {
    std::string feedback = msg->progress;
    return feedback;
  }

  /**
   * @brief Update on_failure event parameters with new data if avaible.
   *
   * This function is used to inject new data into the XMLElement containing
   * parameters related to the on_failure trigger event
   *
   * A pattern needs to be implemented in the derived class
   *
   * @param parameters pointer to the XMLElement containing parameters
   * @return pointer to the XMLElement containing updated parameters
   */
  virtual std::string update_on_failure(std::string& parameters)
  {
    tinyxml2::XMLElement* element = convert_to_xml(parameters);

    element->SetAttribute("replan", true);

    // Create the failed elements element as a child of the existing parameters element
    tinyxml2::XMLElement* failedElements = element->GetDocument()->NewElement("FailedElements");
    element->InsertEndChild(failedElements);

    std::string failedElementsString = "";

    for (const auto& element : result_->failed_elements)
      failedElementsString += element;

    failedElements->SetText(failedElementsString.c_str());

    std::string result = convert_to_string(element);

    return result;
  };
};

}  // namespace capabilities2_runner
