#pragma once

#include <thread>

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

	virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
						std::function<void(const std::string&)> on_started = nullptr,
						std::function<void(const std::string&)> on_terminated = nullptr) override
	{
		// intialize actionRunner
		init_action(node, run_config, "WaypointFollower", on_started, on_terminated);

		nav2_msgs::action::FollowWaypoints::Goal goal_msg;
		geometry_msgs::msg::PoseStamped pose_msg;

		global_frame_ = "map";
		robot_base_frame_ = "base_link";

		pose_msg.header.stamp 		= clock_->now();
		pose_msg.header.frame_id 	= global_frame_;
		pose_msg.pose.position.x	= 1.0;
		pose_msg.pose.position.y	= 0.0;
		pose_msg.pose.position.z	= 0.0;

		goal_msg.poses.push_back(pose_msg);

		pose_msg.header.stamp 		= clock_->now();
		pose_msg.header.frame_id 	= global_frame_;
		pose_msg.pose.position.x	= 6.0;
		pose_msg.pose.position.y	= 2.0;
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
