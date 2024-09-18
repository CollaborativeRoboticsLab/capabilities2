#pragma once

/**
 * @file bt_runner_base.hpp
 * @brief This file is heavily inspired by the nav2_behavior_tree::BtActionNode implementation
 * as it performs almost the same functionality
 *
 * The nav2_behavior_tree::BtActionNode is part of navigation2
 * which is licensed under the Apache 2.0 license. see the following
 * license file for more details:
 *
 */

// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <capabilities2_runner/action_runner.hpp>
#include <behaviortree_cpp/action_node.h>

namespace capabilities2_runner
{

/**
 * @brief base class for a behaviortree action
 *
 * This class is heavily inspired by the nav2_behavior_tree::BtActionNode
 *
 * The action implements a action runner and an action node to combine the two
 * domain definitions of 'action'
 *
 * Through this class, an action can be run in a behaviortree which implements a ROS action
 *
 * @tparam ActionT
 */
template <class ActionT>
class BTRunnerBase : public ActionRunner<ActionT>, public BT::ActionNodeBase
{
public:
  BTRunnerBase() : ActionRunner<ActionT>(), BT::ActionNodeBase()
  {
  }

  // init pattern for bt and runner
  /**
   * @brief initializer function for initializing the action runner in place of constructor due to plugin semantics
   *
   * TODO: this is getting pretty messy
   *
   * @param node
   * @param run_config
   * @param action_name
   * @param on_started
   * @param on_terminated
   * @param on_stopped
   * @param bt_tag_name
   * @param conf
   */
  virtual void init_bt(rclcpp::Node::SharedPtr node, const runner_opts& run_config, const std::string& action_name,
                       const std::string& bt_tag_name, const BT::NodeConfiguration& conf,
                       std::function<void(const std::string&)> on_started = nullptr,
                       std::function<void(const std::string&)> on_terminated = nullptr,
                       std::function<void(const std::string&)> on_stopped = nullptr)
  {
    ActionRunner<ActionT>::init_action(node, run_config, action_name, on_started, on_terminated, on_stopped);

    // FIXME: no init in action base
    // BT::ActionNodeBase::init(bt_tag_name, conf);
  }

  // bt methods that need to be overridden
  BT::NodeStatus tick() override
  {
    // TODO: run the action client?
  }

  void halt() override
  {
    // cancel the action client if it is running
    // can be implemented with the stop virtual method
    // from action runner
    ActionRunner<ActionT>::stop();
  }

  // the action runner methods that need to be overridden
  // these can be overridden when this class is inherited and the action runner
  // action template is resolved
  //
  // see runner_base.hpp for more details
  //
  // virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
  //                    std::function<void(const std::string&)> on_started = nullptr,
  //                    std::function<void(const std::string&)> on_terminated = nullptr,
  //                    std::function<void(const std::string&)> on_stopped = nullptr) override
  //
  // virtual void trigger(std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr) override
};

}  // namespace capabilities2_runner
