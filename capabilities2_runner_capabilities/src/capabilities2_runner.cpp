#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner/runner_base.hpp>
#include <capabilities2_runner_capabilities/get_runner.hpp>
#include <capabilities2_runner_capabilities/input_multiplex_all_runner.hpp>
#include <capabilities2_runner_capabilities/input_multiplex_any_runner.hpp>

// register runner plugins
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::CapabilityGetRunner, capabilities2_runner::RunnerBase);
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::InputMultiplexAllRunner, capabilities2_runner::RunnerBase);
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::InputMultiplexAnyRunner, capabilities2_runner::RunnerBase);
