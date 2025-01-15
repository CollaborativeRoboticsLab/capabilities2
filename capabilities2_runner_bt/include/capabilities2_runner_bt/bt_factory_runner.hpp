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
 * and executes it in a behaviortree
 *
 */
class BTRunnerBase : public RunnerBase, public BT::BehaviorTreeFactory
{
public:
  BTRunnerBase() : RunnerBase(), BT::BehaviorTreeFactory()
  {
  }

  virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config)
  {
    // init the runner base
    init_base(node, run_config);

    // register (bt)actions from ROS plugins
    try
    {
      this->registerFromROSPlugins();
    }
    catch (const std::exception& e)  // TODO: add more specific exception
    {
      throw runner_exception(e.what());
    }
  }

  virtual void stop()
  {
    // reset the tree (this destroys the nodes)
    tree_.reset();
  }

protected:
  /**
   * @brief trigger the behaviour factory with the input data
   *
   * @param parameters
   * @return std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>>
   */
  virtual void trigger(const std::string& parameters) override
  {
    tinyxml2::XMLElement* parameters_xml = convert_to_xml(parameters);

    // if parameters are not provided then cannot proceed
    if (!parameters_xml)
      throw runner_exception("cannot trigger action without parameters");

    // create the tree (ptr)
    tree_ = std::make_shared<BT::Tree>(this->createTreeFromText(parameters_xml->GetText()));

    // return the tick function
    // // the caller can call this function to tick the tree
    // std::function<void(std::shared_ptr<tinyxml2::XMLElement>)> result_callback =
    //     [this](std::shared_ptr<tinyxml2::XMLElement> result) {
    //       // tick the tree
    //       BT::NodeStatus status = tree_->tickWhileRunning();

    //       // fill the result
    //       if (status == BT::NodeStatus::SUCCESS)
    //         result->SetText("OK");
    //       else
    //         result->SetText("FAIL");

    //       return;
    //     };

    // return result_callback;
  }

  // the tree
  std::shared_ptr<BT::Tree> tree_;
};

}  // namespace capabilities2_runner
