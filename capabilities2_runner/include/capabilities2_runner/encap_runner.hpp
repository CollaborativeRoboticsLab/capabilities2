#pragma once

#include <capabilities2_runner/notrigger_action_runner.hpp>
#include <capabilities2_msgs/action/capability.hpp>

namespace capabilities2_runner
{

/**
 * @brief encapsulated capability runner
 *
 * Create an action client to run an action based capability
 * using an encapsulated capability action
 * this allows a system to run an action that is not a managed action
 *
 * Use the generate action, and generate result functions from the action runner
 * to implement an encapsulation strategy for child classes
 *
 */
template <typename ActionT>
class EnCapRunner : public NoTriggerActionRunner<ActionT>
{
public:
  EnCapRunner() : NoTriggerActionRunner()
  {
  }

  /**
   * @brief init encapsulated action
   *
   * call in start when inheriting from EncapRunner
   * to initialize the encapsulated action server and the base action runner
   * this is deferred since the action client topic name is not known at this level
   * of abstraction
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param action_name action name used in the yaml file, used to load specific configuration from the run_config
   */
  virtual void init_encapsulated_action(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                                        const std::string& action_name)
  {
    // init the base action runner
    init_action(node, run_config, action_name);

    // create an encapsulating action server
    encap_action_ = rclcpp_action::create_server<capabilities2_msgs::action::Capability>(
        node_, "~/encap/" + get_provider(),
        std::bind(&EnCapRunner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&EnCapRunner::handle_cancel, this, std::placeholders::_1),
        std::bind(&EnCapRunner::handle_accepted, this, std::placeholders::_1));
  }

  /**
   * @brief stop the encapsulated action server
   *
   * call the parent stop and stop the encapsulated action
   *
   */
  virtual void stop() override
  {
    // stop the encapsulating action server
    encap_action_->cancel_all_goals();
    encap_action_.reset();

    // stop the base class
    ActionRunner::stop();
  }

  // encapsulated action server related functions
  /**
   * @brief handle encap action goal
   *
   * @param uuid
   * @param goal
   */
  virtual void handle_goal(const rclcpp_action::GoalUUID& uuid,
                           std::shared_ptr<const capabilities2_msgs::action::Capability::Goal> goal);

  /**
   * @brief handle encap action cancel goal
   *
   * @param goal_handle
   */
  virtual void handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<capabilities2_msgs::action::Capability>> goal_handle);

  /**
   * @brief handle encap action accepted
   *
   * @param goal_handle
   */
  virtual void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<capabilities2_msgs::action::Capability>> goal_handle);

  /**
   * @brief execute the encapsulated action request
   *
   */
  virtual void execute();

private:
  /** encap action server */
  std::shared_ptr<rclcpp_action::Server<capabilities2_msgs::action::Capability>> encap_action_;
};

}  // namespace capabilities2_runner
