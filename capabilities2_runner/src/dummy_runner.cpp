#include <pluginlib/class_list_macros.hpp>
#include <capabilities2_runner/runner_base.hpp>

namespace capabilities2_runner
{
/**
 * @brief Dummy runner
 *
 * A sample runner that can be used to test the functionality of capabilities server.
 * It does not perform any real action but logs messages when started and stopped.
 *
 */
class DummyRunner : public RunnerBase
{
public:
  void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& bond_id) override
  {
    // init the base runner
    init_base(node, run_config);

    // do nothing
    RCLCPP_INFO(node_->get_logger(), "Dummy runner started");

    // emit started event
    emit_started(bond_id, "", param_on_started());
  }

  void stop(const std::string& bond_id, const std::string& instance_id = "") override
  {
    // guard node
    if (!node_)
      throw runner_exception("node not initialized");

    // stop the runner
    RCLCPP_INFO(node_->get_logger(), "Dummy runner stopped");

    // emit stopped event
    emit_stopped(bond_id, instance_id, param_on_stopped());
  }

  void trigger(capabilities2_events::EventParameters& parameters, const std::string& bond_id,
               const std::string& instance_id = "") override
  {
    RCLCPP_WARN(node_->get_logger(), "Dummy runner cannot trigger");

    // emit failed event
    emit_failed(bond_id, instance_id, param_on_failure());

    // throw an exception as this runner does not support trigger execution
    throw runner_exception("Dummy runner does not support trigger execution");
  }
};

}  // namespace capabilities2_runner

// dummy runner
PLUGINLIB_EXPORT_CLASS(capabilities2_runner::DummyRunner, capabilities2_runner::RunnerBase)
