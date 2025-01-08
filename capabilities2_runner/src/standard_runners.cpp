#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner/runner_base.hpp>
#include <capabilities2_runner/launch_runner.hpp>

namespace capabilities2_runner
{
class DummyRunner : public RunnerBase
{
public:
  void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config) override
  {
    // init the base runner
    init_base(node, run_config);

    // do nothing
    RCLCPP_INFO(node_->get_logger(), "Dummy runner started");
  }

  void stop() override
  {
    // guard node
    if (!node_)
      throw runner_exception("node not initialized");

    // stop the runner
    RCLCPP_INFO(node_->get_logger(), "Dummy runner stopped");
  }

  void trigger(const std::string& parameters) override
  {
    RCLCPP_INFO(node_->get_logger(), "Dummy runner cannot trigger");
  }

protected:
  // throw on triggerExecution function
  void execution() override
  {
    RCLCPP_INFO(node_->get_logger(), "Dummy runner does not have triggerExecution()");
  }
};

}  // namespace capabilities2_runner

// register runner plugins
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::LaunchRunner, capabilities2_runner::RunnerBase)

// dummy runner
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::DummyRunner, capabilities2_runner::RunnerBase)
