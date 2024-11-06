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
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config) override
  {
    init_action(node, run_config, "capabilities");
  }

protected:
  /**
   * @brief This generate goal function overrides the generate_goal() function from ActionRunner()
   * @param parameters XMLElement that contains parameters in the format
   '<Event name=follow_waypoints provider=WaypointRunner x='$value' y='$value' />'
   * @return ActionT::Goal the generated goal
   */
  virtual capabilities2_msgs::action::Plan::Goal generate_goal(tinyxml2::XMLElement* parameters) override
  {
    parameters_ = parameters;

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
   * @brief This generate result function overrides the generate_result() function from ActionRunner(). Since
   * this is not used in this context, this returns nullptr
   * @param result message from FollowWaypoints action
   * @return nullptr
   */
  virtual tinyxml2::XMLElement*
  generate_result(const capabilities2_msgs::action::Plan::Result::SharedPtr& result) override
  {
    return nullptr;
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
  virtual tinyxml2::XMLElement* update_on_failure(tinyxml2::XMLElement* parameters)
  {
    parameters->SetAttribute("replan", true);

    // Create the failed elements element as a child of the existing parameters element
    tinyxml2::XMLElement* failedElements = parameters->GetDocument()->NewElement("FailedElements");
    parameters->InsertEndChild(failedElements);

    std::string failedElementsString = "";

    for (const auto& element : result_->failed_elements)
      failedElementsString += element;

    failedElements->SetText(failedElementsString.c_str());

    return parameters;
  };
};

}  // namespace capabilities2_runner
