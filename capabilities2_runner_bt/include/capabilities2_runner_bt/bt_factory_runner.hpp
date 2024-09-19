#pragma once

#include <capabilities2_runner/runner_base.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

namespace capabilities2_runner
{

/**
 * @brief base runner class for a behaviortree factory
 *
 * The runner implements a behaviour defined by a behaviourtree
 *
 */
class BTRunnerBase : public RunnerBase, public BT::BehaviorTreeFactory
{
public:
  BTRunnerBase() : RunnerBase(), BT::BehaviorTreeFactory()
  {
  }

  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr,
                     std::function<void(const std::string&)> on_stopped = nullptr)
  {
    // init the runner base
    init_base(node, run_config, on_started, on_terminated, on_stopped);

    // init the tree
    // tree_ = createTreeFromText();
  }

protected:
  // the tree
  BT::TreeNode::Ptr tree_;
};

}  // namespace capabilities2_runner
