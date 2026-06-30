#include "behaviortree_cpp/bt_factory.h"
#include "behavior_plugins/heading_correction.hpp"
#include "behavior_plugins/narrow_path.hpp"
#include "behavior_plugins/human_in_narrow_path.hpp"
#include "behavior_plugins/nearest_free_point_on_path.hpp"
#include "behavior_plugins/recovery_once_per_location.hpp"
#include "behavior_plugins/reset_path.hpp"
#include "behavior_plugins/search_local_goal_aside.hpp"
#include "behavior_plugins/set_costmap_inflation_radius.hpp"
#include "behavior_plugins/set_goal_from_location.hpp"
#include "behavior_plugins/is_goal_nearby_condition.hpp"
#include "behavior_plugins/validate_path_action.hpp"
// #include "behavior_plugins/keep_running_until_failure.hpp"

// Register all behavior_plugins nodes in a single factory
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_plugins::NarrowPath>("NarrowPath");
  factory.registerNodeType<behavior_plugins::HumanInNarrowPath>("HumanInNarrowPath");
  factory.registerNodeType<behavior_plugins::RecoveryOncePerLocation>("RecoveryOncePerLocation");
  factory.registerNodeType<behavior_plugins::ResetPath>("ResetPath");
  factory.registerNodeType<behavior_plugins::SearchLocalGoalAside>("SearchLocalGoalAside");
  factory.registerNodeType<behavior_plugins::SetCostmapInflationRadius>("SetCostmapInflationRadius");
  factory.registerNodeType<behavior_plugins::SetGoalFromLocation>("SetGoalFromLocation");
  factory.registerNodeType<behavior_plugins::HeadingCorrection>("HeadingCorrection");
  factory.registerNodeType<behavior_plugins::NearestFreePointOnPath>("NearestFreePointOnPath");
  factory.registerNodeType<behavior_plugins::IsGoalNearbyCondition>("IsGoalNearby");
  factory.registerNodeType<behavior_plugins::ValidatePath>("ValidatePath");
  // factory.registerNodeType<behavior_plugins::KeepRunningUntilFailure>("KeepRunningUntilFailure");
}
