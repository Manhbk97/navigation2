#ifndef BEHAVIOR_PLUGINS__NARROW_PATH_HPP_
#define BEHAVIOR_PLUGINS__NARROW_PATH_HPP_

#include  <atomic>
#include <memory>
#include <mutex>
#include <thread>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace behavior_plugins
{   
/**
 * @brief BT::ConditionNode that returns SUCCESS when both
 *        /narrow_path_detected is true.
 *
 * Used as the gate in NarrowPathHandler: the robot moves to a wait
 * location only when a human is detected
*/

class NarrowPath : public BT::ConditionNode
{
public:
  NarrowPath(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  NarrowPath() = delete;

  ~NarrowPath();

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {};
  } 
private: 

  void initialize();
  
  void narrowPathCallback(std_msgs::msg::Bool::SharedPtr msg);
  void publishStatus();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr narrow_path_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;

  std::atomic<bool> narrow_path_detected_{false};
  bool initialized_{false};
  std::thread executor_thread_;
    
};


} // namespace behavior_plugins

#endif // BEHAVIOR_PLUGINS__NARROW_PATH_HPP_