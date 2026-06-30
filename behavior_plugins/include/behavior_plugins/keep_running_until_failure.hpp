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

#ifndef BEHAVIOR_PLUGINS__KEEP_RUNNING_UNTIL_FAILURE_HPP_
#define BEHAVIOR_PLUGINS__KEEP_RUNNING_UNTIL_FAILURE_HPP_

#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace behavior_plugins
{

/**
 * @brief A BT::DecoratorNode that keeps returning RUNNING while its child
 * returns SUCCESS, and returns SUCCESS as soon as the child returns FAILURE.
 *
 * Typical use case: wait until a condition becomes false.
 *   <KeepRunningUntilFailure>
 *     <HumanInNarrowPath/>
 *   </KeepRunningUntilFailure>
 * → RUNNING while human is detected (child SUCCESS)
 * → SUCCESS  once human is gone    (child FAILURE)
 */
class KeepRunningUntilFailure : public BT::DecoratorNode
{
public:
  KeepRunningUntilFailure(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  BT::NodeStatus tick() override;
};

}  // namespace behavior_plugins

#endif  // BEHAVIOR_PLUGINS__KEEP_RUNNING_UNTIL_FAILURE_HPP_
