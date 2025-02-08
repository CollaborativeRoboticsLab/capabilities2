#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner/runner_base.hpp>
#include <capabilities2_runner_capabilities/fabric_completion_runner.hpp>
#include <capabilities2_runner_capabilities/fabric_set_plan_runner.hpp>
#include <capabilities2_runner_capabilities/capability_get_runner.hpp>

// register runner plugins
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::FabricCompletionRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::FabricSetPlanRunner, capabilities2_runner::RunnerBase)
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::CapabilityGetRunner, capabilities2_runner::RunnerBase)
