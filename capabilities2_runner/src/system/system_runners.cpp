#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner/runner_base.hpp>
#include <capabilities2_runner/system/get_capability_specs_runner.hpp>
#include <capabilities2_runner/system/input_multiplex_runner.hpp>

// register runner plugins
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::GetCapabilitySpecsRunner, capabilities2_runner::RunnerBase);
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::InputMultiplexRunner, capabilities2_runner::RunnerBase);