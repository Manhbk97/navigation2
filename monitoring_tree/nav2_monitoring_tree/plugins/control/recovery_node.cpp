// BTcpp v3 RecoveryNode: retries a main child up to number_of_retries times,
// running a recovery child between each attempt.  No SKIPPED status (v3 API).
#include <string>
#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_monitoring_tree
{

class RecoveryNode : public BT::ControlNode
{
public:
  RecoveryNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config),
    current_child_idx_(0),
    retry_count_(0),
    number_of_retries_(1)
  {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<unsigned int>("number_of_retries", 1u, "Max recovery retries")};
  }

  BT::NodeStatus tick() override
  {
    getInput("number_of_retries", number_of_retries_);
    const unsigned children_count = children_nodes_.size();

    if (children_count != 2) {
      throw BT::BehaviorTreeException(
        "RecoveryNode '" + name() + "' must have exactly 2 children.");
    }

    setStatus(BT::NodeStatus::RUNNING);

    while (current_child_idx_ < children_count && retry_count_ <= number_of_retries_) {
      TreeNode * child = children_nodes_[current_child_idx_];
      const BT::NodeStatus child_status = child->executeTick();

      if (current_child_idx_ == 0) {
        switch (child_status) {
          case BT::NodeStatus::SUCCESS:
            // Navigation succeeded — halt recovery child and reset
            children_nodes_[1]->halt();
            halt();
            return BT::NodeStatus::SUCCESS;

          case BT::NodeStatus::RUNNING:
            return BT::NodeStatus::RUNNING;

          case BT::NodeStatus::FAILURE:
            if (retry_count_ < number_of_retries_) {
              // Halt navigator, move to recovery child
              children_nodes_[0]->halt();
              current_child_idx_ = 1;
            } else {
              halt();
              return BT::NodeStatus::FAILURE;
            }
            break;

          default:
            throw BT::LogicError("Child returned IDLE");
        }

      } else {  // current_child_idx_ == 1 (recovery)
        switch (child_status) {
          case BT::NodeStatus::RUNNING:
            return BT::NodeStatus::RUNNING;

          case BT::NodeStatus::SUCCESS:
            // Recovery done — go back to navigator
            children_nodes_[1]->halt();
            retry_count_++;
            current_child_idx_ = 0;
            break;

          case BT::NodeStatus::FAILURE:
            halt();
            return BT::NodeStatus::FAILURE;

          default:
            throw BT::LogicError("Child returned IDLE");
        }
      }
    }

    halt();
    return BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
    ControlNode::halt();
    retry_count_ = 0;
    current_child_idx_ = 0;
  }

private:
  unsigned current_child_idx_;
  unsigned retry_count_;
  unsigned number_of_retries_;
};

}  // namespace nav2_monitoring_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_monitoring_tree::RecoveryNode>("RecoveryNode");
}
