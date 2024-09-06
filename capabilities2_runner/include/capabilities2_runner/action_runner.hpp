#pragma once

#include <capabilities2_runner/runner_base.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace capabilities2_runner
{

/**
 * @brief action runner base class
 *
 * Create an action client to run an action based capability
 *
 */
template <typename ActionT>
class ActionRunner : public RunnerBase
{
public:
  ActionRunner() : RunnerBase()
  {
  }

  // helpers
  // init action base members
  void init_action(rclcpp::Node::SharedPtr node, const runner_opts& opts, const std::string& action_type)
  {
    // store node pointer and opts
    init_base(node, opts);

    // create an action client
    action_client_ = rclcpp_action::create_client<ActionT>(node_, get_action_name_by_type(action_type));

    // wait for action server
    RCLCPP_INFO(node_->get_logger(), "%s waiting for action: %s", run_config_.interface.c_str(),
                get_action_name_by_type(action_type).c_str());

    if (!action_client_->wait_for_action_server())
    {
      RCLCPP_ERROR(node_->get_logger(), "%s failed to connect to action server", run_config_.interface.c_str());
      throw runner_exception("failed to connect to action server");
    }
  }

  // find resource name by action type
  std::string get_action_name_by_type(const std::string& action_type)
  {
    for (const auto& resource : run_config_.resources)
    {
      if (resource.resource_type == "action")
      {
        if (resource.msg_type == action_type)
        {
          return resource.name;
        }
      }
    }

    throw runner_exception("no action resource found: " + action_type);
  }

  // get first action resource
  std::string get_first_action_name()
  {
    for (const auto& resource : run_config_.resources)
    {
      if (resource.resource_type == "action")
      {
        return resource.name;
      }
    }

    throw runner_exception("no action resources found for interface: " + run_config_.interface);
  }

protected:
  // action client
  std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;
};

}  // namespace capabilities2_runner
