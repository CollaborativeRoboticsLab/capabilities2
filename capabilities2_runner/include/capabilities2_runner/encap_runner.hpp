#pragma once

#include <capabilities2_runner/action_runner.hpp>
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
class EnCapRunner : public ActionRunner<ActionT>
{
public:
  EnCapRunner() : ActionRunner()
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
   * @param node
   * @param run_config
   * @param action_name
   * @param on_started
   * @param on_terminated
   * @param on_stopped
   */
  virtual void init_encapsulated_action(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                                        const std::string& action_name,
                                        std::function<void(const std::string&)> on_started = nullptr,
                                        std::function<void(const std::string&)> on_terminated = nullptr,
                                        std::function<void(const std::string&)> on_stopped = nullptr)
  {
    // init the base action runner
    init_action(node, run_config, action_name, on_started, on_terminated, on_stopped);

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

  /**
   * @brief the trigger method is deleted
   * this is because there is an action server available
   * to run the encapsulated action
   *
   * @param parameters
   * @return std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>>
   */
  virtual std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>>
  trigger(std::shared_ptr<tinyxml2::XMLElement> parameters) override = delete;

  // encapsulated action server related functions
  /** */
  void handle_goal(const rclcpp_action::GoalUUID& uuid,
                   std::shared_ptr<const capabilities2_msgs::action::Capability::Goal> goal);
  /** */
  void handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<capabilities2_msgs::action::Capability>> goal_handle);
  /** */
  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<capabilities2_msgs::action::Capability>> goal_handle);
  /** */
  virtual void execute();

private:
  /**< encap action server */
  std::shared_ptr<rclcpp_action::Server<capabilities2_msgs::action::Capability>> encap_action_;
};

}  // namespace capabilities2_runner
