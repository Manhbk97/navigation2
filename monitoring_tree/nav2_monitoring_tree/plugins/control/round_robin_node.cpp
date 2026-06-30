// BTcpp v3 RoundRobin control node: cycles through children one at a time,
// returning SUCCESS on the first child that succeeds.  No SKIPPED status (v3 API).
#include <string>
#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_monitoring_tree
{

class RoundRobinNode : public BT::ControlNode
{
public:
  RoundRobinNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config),
    current_child_idx_(0),
    num_failed_children_(0)
  {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    const auto num_children = children_nodes_.size();
    setStatus(BT::NodeStatus::RUNNING);

    while (num_failed_children_ < num_children) {
      TreeNode * child = children_nodes_[current_child_idx_];
      const BT::NodeStatus child_status = child->executeTick();

      if (child_status != BT::NodeStatus::RUNNING) {
        if (++current_child_idx_ >= num_children) {
          current_child_idx_ = 0;
        }
      }

      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          num_failed_children_ = 0;
          haltChildren();
          return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::FAILURE:
          num_failed_children_++;
          break;

        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;

        default:
          throw BT::LogicError("Child returned IDLE");
      }
    }

    halt();
    return BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
    ControlNode::halt();
    current_child_idx_ = 0;
    num_failed_children_ = 0;
  }

private:
  unsigned current_child_idx_;
  unsigned num_failed_children_;
};

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::RoundRobinNode>("RoundRobin");
}
