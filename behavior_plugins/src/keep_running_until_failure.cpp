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

#include "behavior_plugins/keep_running_until_failure.hpp"

namespace behavior_plugins
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

}  // namespace behavior_plugins
