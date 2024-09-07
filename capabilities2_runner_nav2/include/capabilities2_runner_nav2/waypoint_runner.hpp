#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
	 * @param parameters string that contains parameters in the format '0.00,0.00'
	 * @param on_started function pointer to trigger at the start of the action client in the runner
	 * @param on_terminated function pointer to trigger at the termination of the action client in the runner
	 */
	virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config, std::string& parameters,
						std::function<void(const std::string&)> on_started = nullptr,
						std::function<void(const std::string&)> on_terminated = nullptr) override
	{
		// intialize actionRunner
		init_action(node, run_config, "WaypointFollower", on_started, on_terminated);

		std::vector<double> values = split_parameters(parameters, ",");

		nav2_msgs::action::FollowWaypoints::Goal goal_msg;
		geometry_msgs::msg::PoseStamped pose_msg;

		global_frame_ = "map";
		robot_base_frame_ = "base_link";

		pose_msg.header.stamp 		= clock_->now();
		pose_msg.header.frame_id 	= global_frame_;
		pose_msg.pose.position.x	= values[0];
		pose_msg.pose.position.y	= values[1];
		pose_msg.pose.position.z	= 0.0;

		goal_msg.poses.push_back(pose_msg);

		// launch runner using action client
    	action_client_->async_send_goal(goal_msg, send_goal_options_);
	}

protected:
		std::string global_frame_;     		/**The global frame of the robot*/
		std::string robot_base_frame_;  	/**The frame of the robot base*/
};

}  // namespace capabilities2_runner
