// Copyright (c) 2024
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

#include <string>

#include "nav2_behavior_tree/plugins/decorator/keep_running_until_failure.hpp"

namespace nav2_behavior_tree
{

KeepRunningUntilFailure::KeepRunningUntilFailure(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
}

BT::NodeStatus KeepRunningUntilFailure::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  const BT::NodeStatus child_status = child_node_->executeTick();

  switch (child_status) {
    case BT::NodeStatus::FAILURE:
      // Child failed → condition is no longer true → we are done, return SUCCESS
      resetChild();
      return BT::NodeStatus::SUCCESS;

    case BT::NodeStatus::SUCCESS:
      // Child succeeded → condition still true → keep waiting
      resetChild();
      return BT::NodeStatus::RUNNING;

    case BT::NodeStatus::RUNNING:
      // Child is still evaluating
      return BT::NodeStatus::RUNNING;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  // BT.CPP 4.x (Jazzy) registers KeepRunningUntilFailure as a built-in at factory init.
  // Guard against double-registration to prevent the "ID already registered" exception.
  if (factory.manifests().find("KeepRunningUntilFailure") == factory.manifests().end()) {
    factory.registerNodeType<nav2_behavior_tree::KeepRunningUntilFailure>("KeepRunningUntilFailure");
  }
}
