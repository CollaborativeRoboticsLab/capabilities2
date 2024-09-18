#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <map>
#include <any>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/srv/cancel_goal.hpp>

namespace capabilities2_runner
{

/**
 * @brief templated struct to handle Action clients and their respective goal_handles
 */
template <typename ActionT>
struct ActionClientBundle
{
  std::shared_ptr<rclcpp_action::Client<ActionT>> action_client;
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle;
  typename rclcpp_action::Client<ActionT>::SendGoalOptions send_goal_options;
};

}  // namespace capabilities2_runner