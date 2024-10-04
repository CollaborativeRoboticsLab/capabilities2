#pragma once

#include <thread>

#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>

#include <capabilities2_runner/action_runner.hpp>

namespace capabilities2_runner
{

/**
 * @brief action runner base class
 *
 * Class to run waypointfollower action based capability
 *
 */
class WayPointRunner : public ActionRunner<nav2_msgs::action::FollowWaypoints>
{
public:
  WayPointRunner() : ActionRunner()
  {
  }

  /**
   * @brief Starter function for starting the action runner
   *
   * @param node shared pointer to the capabilities node. Allows to use ros node related functionalities
   * @param run_config runner configuration loaded from the yaml file
   * @param on_started pointer to function to execute on starting the runner
   * @param on_failure pointer to function to execute on failure of the runner
   * @param on_success pointer to function to execute on success of the runner
   * @param on_stopped pointer to function to execute on stopping the runner
   */
  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_failure = nullptr,
                     std::function<void(const std::string&)> on_success = nullptr,
                     std::function<void(const std::string&)> on_stopped = nullptr) override
  {
    init_action(node, run_config, "follow_waypoints", on_started, on_failure, on_success, on_stopped);
  }

protected:
  /**
   * @brief This generate goal function overrides the generate_goal() function from ActionRunner()
   * @param parameters XMLElement that contains parameters in the format
   '<Event name=follow_waypoints provider=WaypointRunner x='$value' y='$value' />'
   * @return ActionT::Goal the generated goal
   */
  virtual nav2_msgs::action::FollowWaypoints::Goal generate_goal(tinyxml2::XMLElement* parameters) override
  {
    parameters_ = parameters;

    parameters_->QueryDoubleAttribute("x", &x);
    parameters_->QueryDoubleAttribute("y", &y);

    nav2_msgs::action::FollowWaypoints::Goal goal_msg;
    geometry_msgs::msg::PoseStamped pose_msg;

    global_frame_ = "map";
    robot_base_frame_ = "base_link";

    pose_msg.header.stamp = node_->get_clock()->now();
    pose_msg.header.frame_id = global_frame_;
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = 0.0;

    goal_msg.poses.push_back(pose_msg);

    return goal_msg;
  }

  /**
   * @brief This generate result function overrides the generate_result() function from ActionRunner(). Since
   * this is not used in this context, this returns nullptr
   * @param result message from FollowWaypoints action
   * @return nullptr
   */
  virtual tinyxml2::XMLElement*
  generate_result(const nav2_msgs::action::FollowWaypoints::Result::SharedPtr& result) override
  {
    return nullptr;
  }

protected:
  std::string global_frame_;     /**The global frame of the robot*/
  std::string robot_base_frame_; /**The frame of the robot base*/

  double x, y; /**Coordinate frame parameters*/
};

}  // namespace capabilities2_runner
