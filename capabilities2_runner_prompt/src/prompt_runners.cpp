#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner/runner_base.hpp>
#include <capabilities2_runner_prompt/prompt_text_runner.hpp>
#include <capabilities2_runner_prompt/prompt_pose_runner.hpp>
#include <capabilities2_runner_prompt/prompt_occupancy_runner.hpp>
#include <capabilities2_runner_prompt/prompt_plan_runner.hpp>
#include <capabilities2_runner_prompt/prompt_capability_runner.hpp>

// register runner plugins
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::PromptTextRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::PromptPoseRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::PromptOccupancyRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::PromptPlanRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::PromptCapabilityRunner, capabilities2_runner::RunnerBase)