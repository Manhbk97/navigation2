// Include header files
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>


#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <planner_cspace_msgs/PlannerStatus.h>
#include <teb_local_planner/teb_local_planner_ros.h>



// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>

// timed-elastic-band related classes
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/visualization.h>

// message types
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <costmap_converter/ObstacleMsg.h>
// #include <obstacle.h>
// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_converter/costmap_converter_interface.h>
#include <costmap_2d/costmap_2d.h>

// dynamic reconfigure
#include <teb_local_planner/TebLocalPlannerReconfigureConfig.h>
#include <dynamic_reconfigure/server.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <mutex>
#include <std_msgs/Int16MultiArray.h>
#include <planner_cspace_msgs/PlannerStatus.h>
#include <queue> 
#include <costmap_2d/obstacle_layer.h>

// YAML parsing for footprint configuration
#include <yaml-cpp/yaml.h>
#include <fstream>

// recovery 
#include <move_base_msgs/RecoveryStatus.h>
#include <nav_core/recovery_behavior.h>
#include <trajectory_tracker_msgs/TrajectoryTrackerStatus.h>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>

using namespace teb_local_planner;

class TebOptimNode {

enum class NavInd {
  SYS_STATE,
  NAVI_STATE,
  TASK_TYPE,
  REQUEST_STATE,
  REPLAN_COUNT,
  GOAL_STATE,
  DIST_REMAIN,
  DIST_GLOBAL,
  DURATION,
  DOCK_STATE,
  SAFETY_ERROR,
  PLAN_ERROR,
  TRACK_ERROR,
  TASKS_PENDING
};

enum class NavState {
  IDLE,
  PAUSED,
  PLANNING,
  NAVIGATING,
  OBSTACLE_AVOIDING,
  GOAL_POINT,
  DOCKING,
  HALT
};

enum class GoalState {
  NO_GOAL,
  HAVE_GOAL,
  NEAR_GOAL,
  BULLSEYE_GOAL,
  TOLERANT_GOAL,
  NEAR_DOCK_GOAL,
  ALIGN_DOCK_GOAL,
  DOCK_GOAL,
  REACHED_GOAL
};

public:
    TebOptimNode(ros::NodeHandle& nh);

    ~TebOptimNode();

    /**
     * @brief Initialize the node
     */
    void initialize();
    void spin();

private:
    // ROS node handle
    ros::NodeHandle nh_;
    std::string robot_namespace_; //!< Current robot's namespace, e.g. "/sirbot2"
    // external objects (store weak pointers)
    costmap_2d::Costmap2DROS* costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
    costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
    // Planner and visualization
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    PlannerInterfacePtr planner_;
    TebVisualizationPtr visualization_;
    boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;



    pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_; //!< Load costmap converter plugins at runtime
    boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_; //!< Store the current costmap_converter  
    boost::mutex custom_obst_mutex_; //!< Mutex that locks the obstacle array (multi-threaded)
    // Configuration
    TebConfig cfg_;
    boost::shared_ptr<dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>> dynamic_recfg_;
    base_local_planner::OdometryHelperRos odom_helper_;
    FailureDetector failure_detector_;
  

    // Data containers
    ObstContainer obstacles_;
    ViaPointContainer via_points_;
    std::mutex global_plan_mutex_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<geometry_msgs::PoseStamped> working_plan_;  // For processing
    geometry_msgs::PoseStamped first_waypoint_;
    geometry_msgs::PoseStamped end_waypoint_; // enf_waypoint of the global path 
    geometry_msgs::PoseStamped start_; // Robot's starting pose

    PoseSE2 robot_pose_;
    PoseSE2 robot_goal_;
    geometry_msgs::Twist robot_vel_;
    std::vector<geometry_msgs::Point> footprint_spec_;
    double robot_inscribed_radius_;
    double robot_circumscribed_radius_;

    // Footprint configuration from YAML
    std::string footprint_yaml_path_;
    std::string selected_footprint_name_;
    std::map<std::string, std::vector<geometry_msgs::Point>> footprint_configs_;

    double controller_frequency_;
    // Status flags

    bool initialized_;
    bool goal_reached_;
    bool custom_via_points_active_;
    int no_infeasible_plans_;
  


    // Stop and Go configuration parameters
    double distance_end_waypoint_threshold_;
    double waypoint_removal_distance_;
    double teb_angle_threshold_;
    double stop_n_go_angle_threshold_;
    double heading_threshold_;
    int no_try_plans_;
    int counter_large_heading_diff_;
    int counter_large_heading_diff_threshold_;


    RotType last_preferred_rotdir_;
    RobotFootprintModelPtr robot_model;

    // Status
    std_msgs::Int16MultiArray navi_status_;
    NavState nav_state_;
    GoalState goal_state_;
    geometry_msgs::PoseStamped goal_;
    planner_cspace_msgs::PlannerStatus status_;
    ros::Time time_last_infeasible_plan_;
    ros::Time time_last_oscillation_;
    geometry_msgs::Twist last_cmd_;
    std::string global_frame_;
    std::string robot_base_frame_;
    // std::string name_;

    bool stop_and_go_active_;

    // Timer for REPLAN_COUNT delay mechanism
    ros::Time time_replan_count_zero_;    // Timestamp when REPLAN_COUNT became 0
    double stop_and_go_min_duration_;     // Minimum duration in seconds (2.0s)
    bool replan_count_zero_timer_active_; // Track if timer is active
    int prev_replan_count_;               // Track previous REPLAN_COUNT value to detect transitions

    // ROS subscribers and publishers
    ros::Subscriber global_path_sub_;
    ros::Subscriber custom_obst_sub_;
    ros::Subscriber navi_status_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub;
    ros::Publisher pub_status_;
    ros::Publisher received_path_pub_;
    ros::Publisher path_segment_pub_;

    // ros::Timer StopAndGoTimer;
    // Message storage
    nav_msgs::OccupancyGrid::ConstPtr costmap_msg_;
    geometry_msgs::PoseStamped::ConstPtr subgoal_;
    costmap_converter::ObstacleArrayMsg custom_obstacle_msg_; //!< Copy of the most recent obstacle message
    // nav_msgs::Odometry::ConstPtr odom_msg_;

    // Mutexes for thread-safe access
    std::mutex odom_mutex_;
    std::mutex costmap_mutex_;

    // Methods
 
    void validateFootprints(double inscribed_radius, double circumscribed_radius, double min_obstacle_dist);
    // double convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const;
    static RobotFootprintModelPtr getRobotFootprintFromParamServer(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, const TebConfig& config, int& no_try_plans);
    static Point2dContainer makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name);
    static double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name);
    double estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
        int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length=3) const;
    bool pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
        std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot=1);
    void updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation);
    bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
        const geometry_msgs::PoseStamped& global_pose,  const costmap_2d::Costmap2D& costmap,
        const std::string& global_frame, double max_plan_length, std::vector<geometry_msgs::PoseStamped>& transformed_plan,
        int* current_goal_idx = NULL, geometry_msgs::TransformStamped* tf_plan_to_global = NULL) const;

    void configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan, int& goal_idx);
    void updateObstacleContainerWithCostmap();
    void updateObstacleContainerWithCostmapConverter();

    void checkStopAndGoConditions();
    void publishEmptyPath();


    // Callback methods
    void mainCycleCallback();

    void reconfigureCallback(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
    // void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void pathSegmentCallback(const nav_msgs::Path::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void Callback_navistate(const std_msgs::Int16MultiArray::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    bool loadFootprintFromYAML(const std::string& yaml_path);
    
};

// Constructor implementation
TebOptimNode::TebOptimNode(ros::NodeHandle& nh) : 
    nh_(nh),
    costmap_ros_(nullptr),
    costmap_(nullptr),
    nav_state_(NavState::IDLE),
    goal_state_(GoalState::NO_GOAL),   
    initialized_(false),
    goal_reached_(false),
    no_infeasible_plans_(0),
    counter_large_heading_diff_(0),
    dynamic_recfg_(NULL),
    custom_via_points_active_(false), 
    last_preferred_rotdir_(RotType::none),
    stop_and_go_active_(false),
    replan_count_zero_timer_active_(false),
    prev_replan_count_(-1),

    costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons") {
        initialize();
}


// Destructor implementation
TebOptimNode::~TebOptimNode() {
       // Stop timers first to prevent callbacks during cleanup
    obstacles_.clear();
    global_plan_.clear();
    via_points_.clear();
    // Shut down subscribers, publishers, and timers
    if (costmap_ros_) {
        delete costmap_ros_;
        costmap_ros_ = nullptr;
    }
}
void TebOptimNode::initialize() {
    // Initialize TF listener
    tf_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
    ros::NodeHandle nh_private("~/teb_config");

    // cfg_.loadRosParamFromNodeHandle(nh_);
    obstacles_.reserve(500);


    ros::NodeHandle pnh;
    cfg_.loadRosParamFromNodeHandle(nh_);
    robot_namespace_ = cfg_.robot_name_space;  // e.g. "/sirbot2"
    // if (!robot_namespace_.empty() && robot_namespace_[0] == '/')
    //   robot_namespace_ = robot_namespace_.substr(1);  // Remove leading '/' to get namespace like "sirbot1"
    // ROS_INFO("TebOptimNode: robot_namespace_ = %s", robot_namespace_.c_str());
    nh_private.param("controller_frequency", controller_frequency_, 10.0);

    // Distance thresholds for stop and go activation
    nh_private.param("distance_end_waypoint_threshold", distance_end_waypoint_threshold_, 0.1);
    // nh_private.param("distance_first_waypoint_threshold", distance_first_waypoint_threshold_, 0.2);
    nh_private.param("waypoint_removal_distance", waypoint_removal_distance_, 0.1);
    nh_private.param("teb_angle_threshold", teb_angle_threshold_, 4*M_PI/5);
    nh_private.param("stop_n_go_angle_threshold", stop_n_go_angle_threshold_, M_PI/3);
    nh_private.param("heading_threshold", heading_threshold_, 90.0 * M_PI / 180.0);
    nh_private.param("no_try_plans", no_try_plans_, 8);
    nh_private.param("counter_large_heading_diff_threshold", counter_large_heading_diff_threshold_, 20);
    nh_private.param("stop_and_go_min_duration", stop_and_go_min_duration_, 2.0);


    // Print the loaded parameters for debugging
    
    // ROS_INFO_STREAM("distance_end_waypoint_threshold: " << distance_end_waypoint_threshold_);
    // ROS_INFO_STREAM("waypoint_removal_distance: " << waypoint_removal_distance_);
    // ROS_INFO_STREAM("teb_angle_threshold: " << teb_angle_threshold_);
    // ROS_INFO_STREAM("stop_n_go_angle_threshold: " << stop_n_go_angle_threshold_);
    
    // set up visualization 
    visualization_ = TebVisualizationPtr(new TebVisualization(pnh, cfg_));

    // create robot footprint/contour model for optimization
    robot_model = getRobotFootprintFromParamServer(pnh, nh_private, cfg_, no_try_plans_);

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", *tf_);
    // costmap_ros_->pause();  
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
    costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);

    global_frame_ = costmap_ros_->getGlobalFrameID();
    cfg_.map_frame = global_frame_; // TODO
    robot_base_frame_ = costmap_ros_->getBaseFrameID();
    // ROS_INFO_STREAM("TebOptimNode: global_frame: " << global_frame_ << ", robot_base_frame: " << robot_base_frame_);

    //Initialize a costmap to polygon converter
    if (!cfg_.obstacles.costmap_converter_plugin.empty())
    {
      try
      {
        costmap_converter_ = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
        std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin);
        // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
        boost::replace_all(converter_name, "::", "/");
        costmap_converter_->setOdomTopic(cfg_.odom_topic);
        costmap_converter_->initialize(ros::NodeHandle(nh_, "costmap_converter/" + converter_name));
        costmap_converter_->setCostmap2D(costmap_);
        
        costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_, cfg_.obstacles.costmap_converter_spin_thread);
        ROS_INFO_STREAM("Costmap conversion plugin " << cfg_.obstacles.costmap_converter_plugin << " loaded.");        
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
        costmap_converter_.reset();
      }
    }
    else 
      ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");

    footprint_spec_ = costmap_ros_->getRobotFootprint();
    //ROS_INFO("Debug: footprint_spec_ size = %zu", footprint_spec_.size());

    // Calculate inscribed and circumscribed radii from the loaded footprint
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);
    ROS_INFO("TebOptimNode: robot_inscribed_radius: %f, robot_circumscribed_radius: %f",
             robot_inscribed_radius_, robot_circumscribed_radius_);
    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(cfg_.odom_topic);

    // Setup planner (homotopy class planning or just the local teb planner)
    if (cfg_.hcp.enable_homotopy_class_planning) {
        // ROS_INFO("enable_homotopy_class_planning is set to true. Using HomotopyClassPlanner.");
        planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_));
    } else
        planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_));


    // Setup subscribers
    global_path_sub_ = pnh.subscribe("local_path", 1, &TebOptimNode::pathSegmentCallback, this);
    path_sub_ = pnh.subscribe("smooth_path", 1, &TebOptimNode::pathCallback, this);
    navi_status_sub_ = pnh.subscribe("navi_status", 1, &TebOptimNode::Callback_navistate, this);
    odom_sub = pnh.subscribe<nav_msgs::Odometry>("tf_odom", 1, &TebOptimNode::odomCallback, this);
    

    // Setup publishers
    received_path_pub_ = pnh.advertise<nav_msgs::Path>("track_path", 1);
    pub_status_ = pnh.advertise<planner_cspace_msgs::PlannerStatus>("planner_3d/status", 1);
    path_segment_pub_ = pnh.advertise<nav_msgs::Path>("path_segment", 1); 


    dynamic_recfg_ = boost::make_shared<dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>>(nh_private);

    dynamic_recfg_->setCallback(boost::bind(&TebOptimNode::reconfigureCallback, this, _1, _2));
    // validate optimization footprint and costmap footprint
    validateFootprints(robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);

    // costmap_ros_->start();

    failure_detector_.setBufferLength(std::round(cfg_.recovery.oscillation_filter_duration*controller_frequency_));    

    initialized_ = true;
    ROS_INFO("TebOptimNode initialized successfully");
   
}

void TebOptimNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Copy header and pose from odometry to start_
    start_.header = msg->header;
    start_.pose = msg->pose.pose;

    // ROS_DEBUG_STREAM("Received odometry update:");
    // ROS_INFO("Position: x=%f, y=%f, z=%f", 
    //          start_.pose.position.x, 
    //          start_.pose.position.y, 
    //          start_.pose.position.z);


}



// Load footprint configurations from YAML file
bool TebOptimNode::loadFootprintFromYAML(const std::string& yaml_path) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_path);

        if (!config["footprints"]) {
            ROS_ERROR("No 'footprints' section found in YAML file: %s", yaml_path.c_str());
            return false;
        }

        footprint_configs_.clear();

        // Iterate through all footprint configurations
        for (YAML::const_iterator it = config["footprints"].begin();
             it != config["footprints"].end(); ++it) {

            std::string footprint_name = it->first.as<std::string>();
            const YAML::Node& points = it->second;

            std::vector<geometry_msgs::Point> footprint;

            // Parse each point in the footprint
            for (size_t i = 0; i < points.size(); ++i) {
                if (points[i].size() != 2) {
                    ROS_WARN("Invalid point format in footprint '%s' at index %zu",
                             footprint_name.c_str(), i);
                    continue;
                }

                geometry_msgs::Point pt;
                pt.x = points[i][0].as<double>();
                pt.y = points[i][1].as<double>();
                pt.z = 0.0;
                footprint.push_back(pt);
            }

            if (!footprint.empty()) {
                footprint_configs_[footprint_name] = footprint;
                ROS_INFO("Loaded footprint '%s' with %zu points",
                         footprint_name.c_str(), footprint.size());
            }
        }

        ROS_INFO("Successfully loaded %zu footprint configurations from %s",
                 footprint_configs_.size(), yaml_path.c_str());
        return true;

    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parsing error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        ROS_ERROR("Error loading footprint from YAML: %s", e.what());
        return false;
    }
}

// Add this function to check stop and go conditions:
void TebOptimNode::checkStopAndGoConditions() {
  
  // Check if we should activate stop_and_go mode
  bool should_activate_stop_and_go = false;
  

  // Condition2: check distance from first waypoint 

  double distance_to_first_waypoint = std::hypot(start_.pose.position.x - first_waypoint_.pose.position.x,
                                                  start_.pose.position.y - first_waypoint_.pose.position.y);
  double distance_to_end_waypoint = std::hypot(start_.pose.position.x - end_waypoint_.pose.position.x,
                                                  start_.pose.position.y - end_waypoint_.pose.position.y);
  // ROS_INFO("Distance to first waypoint: %f", distance_to_first_waypoint);
  // ROS_DEBUG_THROTTLE(1.0, "Distance to first waypoint: %f", distance_to_first_waypoint);
  // ROS_DEBUG_STREAM("Distance to end waypoint: " << distance_to_end_waypoint);
  // Calculate heading difference (thread-safe), range [0, π]
  bool large_heading_diff = false;
  {
      std::lock_guard<std::mutex> lock(global_plan_mutex_);
      if (!global_plan_.empty()) {
          double waypoint_yaw ;
          double robot_yaw = tf2::getYaw(start_.pose.orientation);

          if (global_plan_.size() > 10){
            size_t mid_idx = global_plan_.size() / 5;
            waypoint_yaw = tf2::getYaw(global_plan_[mid_idx].pose.orientation);
          } else {
            waypoint_yaw = tf2::getYaw(global_plan_.front().pose.orientation);
          }
          // double waypoint_yaw = tf2::getYaw(global_plan_.front().pose.orientation);
          double heading_diff = std::abs(g2o::normalize_theta(waypoint_yaw - robot_yaw));
          large_heading_diff = (heading_diff > heading_threshold_);
          // ROS_INFO_THROTTLE(2.0, "Heading difference: %.3f radians", heading_diff);
      }
  }
  


  if (navi_status_.data.size() > static_cast<size_t>(NavInd::REPLAN_COUNT)) {
    int current_replan_count = static_cast<int>(navi_status_.data[static_cast<size_t>(NavInd::REPLAN_COUNT)]);

    if (current_replan_count == 0) {
        should_activate_stop_and_go = true;

        // Start timer ONLY when REPLAN_COUNT transitions from non-zero to 0
        if (prev_replan_count_ != 0 && !replan_count_zero_timer_active_) {
            time_replan_count_zero_ = ros::Time::now();
            replan_count_zero_timer_active_ = true;
            ROS_INFO("Starting 2s timer: REPLAN_COUNT transitioned %d -> 0", prev_replan_count_);
        }

        ROS_DEBUG_THROTTLE(1.0, "Activating stop_and_go mode due to REPLAN_COUNT: %d", current_replan_count);
    }

    // Update previous value for next cycle
    prev_replan_count_ = current_replan_count;

  } else {
    ROS_DEBUG_THROTTLE(5.0, "navi_status_ not yet received; skipping REPLAN_COUNT check");
  }

  // Condition 3: Distance to end waypoint < 0.5
  if (distance_to_end_waypoint < distance_end_waypoint_threshold_) {
      should_activate_stop_and_go = true;
      ROS_DEBUG_THROTTLE(2.0, "Activating stop_and_go mode due to distance_to_end_waypoint: %f", distance_to_end_waypoint);
  }
  // Condition 4: NEAR_GOAL or GOAL_POINT state
  // if (nav_state_ == NavState::IDLE) {
  //     should_activate_stop_and_go = true;
  //     ROS_WARN("Activating stop_and_go mode due to  GOAL_POINT state");
  // }
  
  // Activate stop_and_go if conditions are met and not already active
  if (should_activate_stop_and_go && !stop_and_go_active_) {

      stop_and_go_active_ = true;
  }
  
  // Check if we should deactivate stop_and_go
  if (stop_and_go_active_) {

      // Deactivate conditions:
      // 1. REPLAN_COUNT != 0 AND small heading diff
      // 2. Distance to first waypoint > threshold

      bool should_deactivate = false;
      
      if (navi_status_.data.size() > static_cast<size_t>(NavInd::REPLAN_COUNT)) {

        // Calculate time elapsed since REPLAN_COUNT became 0
        double time_since_replan_zero = (ros::Time::now() - time_replan_count_zero_).toSec();
        bool timer_expired = !replan_count_zero_timer_active_ ||
                             (time_since_replan_zero > stop_and_go_min_duration_);

        if (!large_heading_diff &&
          (static_cast<int>(navi_status_.data[static_cast<size_t>(NavInd::REPLAN_COUNT)]) ) !=0 &&
           distance_to_end_waypoint > distance_end_waypoint_threshold_ &&
           timer_expired) {  // Add timer check

            stop_and_go_active_ = false;
            replan_count_zero_timer_active_ = false;  // Reset timer

            ROS_INFO_THROTTLE(2.0,
              "Deactivating stop_and_go: REPLAN_COUNT=%d, distance_to_end_waypoint=%.3f, time_elapsed=%.2fs",
              static_cast<int>(navi_status_.data[static_cast<size_t>(NavInd::REPLAN_COUNT)]),
              distance_to_end_waypoint,
              time_since_replan_zero);
        }
     }   
  }
}

void TebOptimNode::publishEmptyPath() {
    // Publish an empty path to the received_path_pub_ topic
    nav_msgs::Path empty_path;
    empty_path.header.frame_id = global_frame_; // Use the appropriate frame ID
    empty_path.header.stamp = ros::Time::now();
    empty_path.poses.clear();
    received_path_pub_.publish(empty_path);
    //ROS_INFO("Published empty path");
}
     
bool TebOptimNode::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
            const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
            std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::TransformStamped* tf_plan_to_global) const {

    // this method is a slightly modified version of base_local_planner/goal_functions.h
    if (global_plan.empty()) {
        *current_goal_idx = 0;
        ROS_ERROR("Empty global plan");
        return false;
    }
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    transformed_plan.clear();

    try 
    {
      // if (global_plan.empty())
      // {
      //   ROS_ERROR("Received plan with zero length");
      //   *current_goal_idx = 0;
      //   return false;
      // }

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
                                                                        plan_pose.header.frame_id, ros::Duration(cfg_.robot.transform_tolerance));

    //let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                        costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    // dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
            // located on the border of the local costmap
    dist_threshold *= 1.0; 

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;

    //we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
        double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
        double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
        double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

        if (robot_reached && new_sq_dist > sq_dist)
        break;

        if (new_sq_dist < sq_dist) // find closest distance
        {
            sq_dist = new_sq_dist;
            i = j;
            if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
            robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
        }
    }

    geometry_msgs::PoseStamped newer_pose;

    double plan_length = 0; // check cumulative Euclidean distance along the plan

    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
        const geometry_msgs::PoseStamped& pose = global_plan[i];
        tf2::doTransform(pose, newer_pose, plan_to_global_transform);

        transformed_plan.push_back(newer_pose);

        double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
        double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;

        // caclulate distance to previous pose
        if (i>0 && max_plan_length>0)
            plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

        ++i;
    }

    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
        tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

        transformed_plan.push_back(newer_pose);

        // Return the index of the current goal point (inside the distance threshold)
        if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
        // Return the index of the current goal point (inside the distance threshold)
        if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }

        // Return the transformation from the global plan to the global planning frame if desired
        if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
    }
    catch(tf::LookupException& ex)
    {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ConnectivityException& ex) 
    {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ExtrapolationException& ex) 
    {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        if (global_plan.size() > 0)
            ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

        return false;
    }

    return true;
}

void TebOptimNode::configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan,  int& goal_idx)
{ 
    // ROS_INFO("back_up  mode");
    ros::Time current_time = ros::Time::now();
    if (cfg_.recovery.shrink_horizon_backup && 
        goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
       (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).toSec() < cfg_.recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
    {
        ROS_INFO_COND(no_infeasible_plans_==1, "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_.recovery.shrink_horizon_min_duration);
        // ROS_INFO("recude horizon_backup: goal_idx: %d, transformed_plan.size(): %d", goal_idx, (int)transformed_plan.size());


        // Shorten horizon if requested
        // reduce to 50 percent:
        int horizon_reduction = goal_idx/2;
        
        if (no_infeasible_plans_ > 6)
        {
            ROS_INFO_COND(no_infeasible_plans_==10, "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
            horizon_reduction /= 2;

        }
        
        // we have a small overhead here, since we already transformed 50% more of the trajectory.
        // But that's ok for now, since we do not need to make transformGlobalPlan more complex 
        // and a reduced horizon should occur just rarely.
        int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
        goal_idx -= horizon_reduction;
        if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
            transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
        else
            goal_idx += horizon_reduction; // this should not happen, but safety first ;-) 
    }
    
    
    // detect and resolve oscillations
    if (cfg_.recovery.oscillation_recovery)
    {
        double max_vel_theta;
        double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_.robot.max_vel_x : cfg_.robot.max_vel_x_backwards;
        if (cfg_.robot.min_turning_radius!=0 && max_vel_current>0)
            max_vel_theta = std::max( max_vel_current/std::abs(cfg_.robot.min_turning_radius),  cfg_.robot.max_vel_theta );
        else
            max_vel_theta = cfg_.robot.max_vel_theta;
        
        failure_detector_.update(last_cmd_, cfg_.robot.max_vel_x, cfg_.robot.max_vel_x_backwards, max_vel_theta,
                               cfg_.recovery.oscillation_v_eps, cfg_.recovery.oscillation_omega_eps);
        
        bool oscillating = failure_detector_.isOscillating();
        bool recently_oscillated = (ros::Time::now()-time_last_oscillation_).toSec() < cfg_.recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently
        // ROS_INFO("oscillating: %d", oscillating);        
          


        if (oscillating)
        {
            if (!recently_oscillated)
            {
                // save current turning direction
                if (robot_vel_.angular.z > 0)
                    last_preferred_rotdir_ = RotType::left;
                else
                    last_preferred_rotdir_ = RotType::right;
                ROS_WARN("TebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
            }
            time_last_oscillation_ = ros::Time::now();  
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
        }
        else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
        {
            last_preferred_rotdir_ = RotType::none;
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
            ROS_INFO("TebLocalPlannerROS: oscillation recovery disabled/expired.");
        }
    }

}


void TebOptimNode::updateObstacleContainerWithCostmap()
{  
  // Add costmap obstacles if desired
  if (cfg_.obstacles.include_costmap_obstacles)
  {
    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();
    
    for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
    {
      for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
      {
        if (costmap_->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE)
        {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
            
          // check if obstacle is interesting (e.g. not far behind the robot)
          Eigen::Vector2d obs_dir = obs-robot_pose_.position();
          if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist  )
            continue;
            
          obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
        }
      }
    }

  }
  
}

double TebOptimNode::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
              int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
  int n = (int)global_plan.size();
  
  // check if we are near the global goal already
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // we've exactly reached the goal
    {
      return tf2::getYaw(local_goal.pose.orientation);
    }
    else
    {
      tf2::Quaternion global_orientation;
      tf2::convert(global_plan.back().pose.orientation, global_orientation);
      tf2::Quaternion rotation;
      tf2::convert(tf_plan_to_global.transform.rotation, rotation);
      // TODO(roesmann): avoid conversion to tf2::Quaternion
      return tf2::getYaw(rotation *  global_orientation);
    }     
  }
  
  // reduce number of poses taken into account if the desired number of poses is not available
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
  
  std::vector<double> candidates;
  geometry_msgs::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::PoseStamped tf_pose_kp1;
  
  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // Transform pose of the global plan to the planning frame
    tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);

    // calculate yaw angle  
    candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
        tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );
    
    if (i<range_end-1) 
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);
}
      
bool TebOptimNode::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return true;
  
  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
    geometry_msgs::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
         erase_end = it;
         break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;
    
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}
    

void TebOptimNode::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation)
{
  via_points_.clear();
  
  if (min_separation<=0)
    return;
  
  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
      continue;
        
    // add via-point
    via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
    prev_idx = i;
  }
  
}
      
void TebOptimNode::mainCycleCallback() {

    // Get robot velocity
    geometry_msgs::PoseStamped robot_vel_tf;
      // Update robot pose and velocity
    {
        // std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_helper_.getRobotVel(robot_vel_tf);
    }
    robot_vel_.linear.x = robot_vel_tf.pose.position.x ;
    robot_vel_.linear.y = robot_vel_tf.pose.position.y ;
    robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);
    // ROS_INFO("robot_vel_x: %f, robot_vel_y: %f, robot_vel_theta: %f", robot_vel_.linear.x, robot_vel_.linear.y, robot_vel_.angular.z);
    checkStopAndGoConditions();
    // If stop_and_go is active, don't run TEB planner
    if (stop_and_go_active_) {

        nav_msgs::Path stop_n_go_path;
        stop_n_go_path.header.frame_id = global_frame_; // Use the appropriate frame ID
        stop_n_go_path.header.stamp = ros::Time::now(); // Update timestamp
        // stop_n_go_path.poses.clear();
        {
            std::lock_guard<std::mutex> lock(global_plan_mutex_);  //  Add lock!
            stop_n_go_path.poses = global_plan_;
        }
        received_path_pub_.publish(stop_n_go_path);
        ROS_INFO_THROTTLE(4.0, "publish global plan in stop and go mode, size: %zu", stop_n_go_path.poses.size());
        return;
    }

    // Create thread-safe working copy
    {
        std::lock_guard<std::mutex> lock(global_plan_mutex_);
        working_plan_ = global_plan_;  // Copy the current global plan
        //working_plan_.swap(global_plan_);  // O(1) swap instead of O(n) copy
    }
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = ros::Time::now();
    cmd_vel.header.frame_id = robot_base_frame_;
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
    // Check if we have valid input
    if (!initialized_ || working_plan_.empty()) {
        status_.error = planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND;
        pub_status_.publish(status_); 
        return;
    } 
    else if (nav_state_ == NavState::PAUSED || nav_state_ == NavState::IDLE || goal_state_ == GoalState::NO_GOAL) {
        // ROS_INFO("nav_state_: %d, goal_state_: %d", (int)nav_state_, (int)goal_state_);
        ROS_DEBUG_THROTTLE(1.0, "TebOptimNode: Navigation is paused. Not computing velocity commands.");

        return;
      // ROS_INFO("TebOptimNode: Navigation is active. Computing velocity commands.");
    } 
    else {
      // print global_size
    // Now work with working_plan_ safely (no race conditions)
    ROS_DEBUG("working_plan_.size(): %zu", working_plan_.size());
     
      ros::Time start = ros::Time::now();
      costmap_ros_->start();
      
      if (!costmap_ros_ || !costmap_) {
          ROS_ERROR("Costmap not available!");
          return;
      }

      boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ros_->getCostmap()->getMutex()));
      
      goal_reached_ = false;  
 
      // Get robot pose
      geometry_msgs::PoseStamped robot_pose;
      costmap_ros_->getRobotPose(robot_pose);
      robot_pose_ = PoseSE2(robot_pose.pose);
  
      // ROS_INFO("robot_pose_x: %f, robot_pose_y: %f, robot_pose_theta: %f", robot_pose_.x(), robot_pose_.y(), robot_pose_.theta());
 
  
  
      // Transform global plan to the frame of interest (w.r.t. the local costmap)
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      int goal_idx;
      geometry_msgs::TransformStamped tf_plan_to_global;
      if (!transformGlobalPlan(*tf_, working_plan_, robot_pose, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist, 
                              transformed_plan, &goal_idx, &tf_plan_to_global))
      {
          ROS_WARN("Could not transform the global plan to the frame of the controller");
          return ;
      }
      // ROS_INFO("goal index: %d", goal_idx);
      if (!custom_via_points_active_)
      updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);
  
      nav_msgs::Odometry base_odom;
      odom_helper_.getOdom(base_odom);
  
      // check if global goal is reached
      geometry_msgs::PoseStamped global_goal;
      tf2::doTransform(working_plan_.back(), global_goal, tf_plan_to_global);
      double dx = global_goal.pose.position.x - robot_pose_.x();
      double dy = global_goal.pose.position.y - robot_pose_.y();
      double delta_orient = g2o::normalize_theta( tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta() );

      if(fabs(std::sqrt(dx*dx+dy*dy)) < cfg_.goal_tolerance.xy_goal_tolerance
          && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance
          && (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0)
          && (base_local_planner::stopped(base_odom, cfg_.goal_tolerance.theta_stopped_vel, cfg_.goal_tolerance.trans_stopped_vel)
              || cfg_.goal_tolerance.free_goal_vel))
      {
          goal_reached_ = true;
          ROS_DEBUG("Goal reached!");
          cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
          last_cmd_ = cmd_vel.twist;
          return ;
      }
      // check if we should enter any backup mode and apply settings
      configureBackupModes(transformed_plan, goal_idx);
  
      // Return false if the transformed global plan is empty
      if (transformed_plan.empty())
      {
          ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
          return ;
      }
  
      // Get current goal point (last point of the transformed plan)
      robot_goal_.x() = transformed_plan.back().pose.position.x;
      robot_goal_.y() = transformed_plan.back().pose.position.y;
      // Overwrite goal orientation if needed
      if (cfg_.trajectory.global_plan_overwrite_orientation)
      {
          robot_goal_.theta() = estimateLocalGoalOrientation(working_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
          // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
          tf2::Quaternion q;
          q.setRPY(0, 0, robot_goal_.theta());
          tf2::convert(q, transformed_plan.back().pose.orientation);
      }  
      else
      {
          robot_goal_.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
      }
      // ROS_INFO("robot_goal_x: %f, robot_goal_y: %f, robot_goal_theta: %f", robot_goal_.x(), robot_goal_.y(), robot_goal_.theta());
  
      // Extract heading (yaw) from the transformed_plan's first pose
      double original_heading = tf2::getYaw(transformed_plan.front().pose.orientation);

      // Calculate angle difference between original waypoint heading and robot heading
      double angle_diff = std::abs(g2o::normalize_theta(original_heading - robot_pose_.theta()));
      //transformed_plan.front() = robot_pose;
      // Conditionally update the first waypoint
      
      if (angle_diff > teb_angle_threshold_) {  
        // Keep original waypoint position and orientation
        // global_plan_.front().pose
        counter_large_heading_diff_= counter_large_heading_diff_ + 1;
        // ROS_INFO("counter_large_heading_diff_: %d", counter_large_heading_diff_);
        if (counter_large_heading_diff_ >= counter_large_heading_diff_threshold_ ){
          stop_and_go_active_ = true;
          counter_large_heading_diff_ = 0;
          ROS_WARN_THROTTLE(1.0,"Activating stop_and_go mode due to large heading difference for %d cycles", counter_large_heading_diff_);
        }
        if (!transformed_plan.empty() && transformed_plan.size() >= 4) {
          // set the first waypoint to the pose in the middle of the transformed plan
          size_t mid_idx = transformed_plan.size() / 4;
          transformed_plan.front().pose.orientation = transformed_plan[mid_idx].pose.orientation;
        }

      } else {
          // Update with robot pose
          transformed_plan.front() = robot_pose;
          // ROS_DEBUG("Heading difference %.3f >= 4PI/5, updating first waypoint with robot pose", angle_diff);
      }


      obstacles_.clear();
      // update obstacles in costmap

      // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
      if (costmap_converter_)
        updateObstacleContainerWithCostmapConverter();
      else
        updateObstacleContainerWithCostmap();


      // Do not allow config changes during the following optimization step
      boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());
      ROS_DEBUG_THROTTLE(2.0, "number of feasible plans: %d",no_infeasible_plans_);

      if (no_infeasible_plans_ > no_try_plans_){
        status_.error = planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND;
        ROS_WARN_THROTTLE(2.0, "TebLocalPlannerROS: no feasible plan found for %d times in a row. Resetting smooth_path...", no_try_plans_);

        no_infeasible_plans_ = 0;
        publishEmptyPath();
        // Clear costmap around robot
        pub_status_.publish(status_);
        return;
      }  

      // std::string message;
      bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
      status_.error = planner_cspace_msgs::PlannerStatus::GOING_WELL;
      if (!success) {
          planner_->clearPlanner(); // force reinitialization for next time
          ROS_INFO_THROTTLE(2.0, "teb_local_planner was not able to obtain a local plan for the current setting.");
          ++no_infeasible_plans_; // increase number of infeasible solutions in a row
          time_last_infeasible_plan_ = ros::Time::now();
          last_cmd_ = cmd_vel.twist;
          status_.error = planner_cspace_msgs::PlannerStatus::IN_ROCK;
          // pub_status_.publish(status_);
          return;
      } 


      // Check for divergence
      if (planner_->hasDiverged())
      {
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

        planner_->clearPlanner();
        ROS_WARN_THROTTLE(2.0, "TebLocalPlannerROS: the trajectory has diverged. Resetting plannerasdasd...");
        // status_.error = planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND;
        // status_.status = planner_cspace_msgs::PlannerStatus::DOING;
        // pub_status_.publish(status_);
        // publishEmptyPath(); // publish empty path to the received_path_pub_ topic
        // stop_and_go_active_ = true; // activate stop and go mode
        ++no_infeasible_plans_; // increase number of infeasible solutions in a row
        time_last_infeasible_plan_ = ros::Time::now();
        last_cmd_ = cmd_vel.twist;
        return ; 
      }
      // Check feasibility (but within the first few states only)

      if(cfg_.robot.is_footprint_dynamic)
      {
        // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.

        // Check if we should reload from YAML based on /sirbot1/hw/model parameter
        std::string hw_model;
        // ROS_INFO("TebOptimNode: Detected robot namespace: '%s'", robot_namespace_.c_str()); ros::NodeHandle(robot_namespace_).getParam("hw/model", hw_model)
        if (ros::NodeHandle(robot_namespace_).getParam("hw/model", hw_model) && !footprint_configs_.empty()) {
        // if (nh_.getParam(robot_namespace_ + "/hw/model", hw_model) && !footprint_configs_.empty()) {
            auto it = footprint_configs_.find(hw_model);
            if (it != footprint_configs_.end()) {
                footprint_spec_ = it->second;
                selected_footprint_name_ = hw_model;
            } else {
                // Model not found in YAML, fall back to costmap
                footprint_spec_ = costmap_ros_->getRobotFootprint();
            }
        } else {
            // No parameter or configs not loaded, use costmap
            footprint_spec_ = costmap_ros_->getRobotFootprint();
        }

        costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);
      }
      // ROS_INFO("robot_inscribed_radius_: %f, robot_circumscribed_radius_: %f", robot_inscribed_radius_, robot_circumscribed_radius_);
      // ROS_INFO("costmap_model_.get(): %f", costmap_model_.get());
  
      bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_, cfg_.trajectory.feasibility_check_no_poses);
      if (!feasible) {
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
        planner_->clearPlanner();
        ROS_WARN_THROTTLE(1.0, "TebLocalPlannerROS: trajectory is not feasible. Resetting planner...");
        // initialized_=false;
        // obstacles_.clear();
        publishEmptyPath(); // publish empty path to the received_path_pub_ topic
        // status_.error = planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND;
        // pub_status_.publish(status_); 
        ++no_infeasible_plans_; // increase number of infeasible solutions in a row
        time_last_infeasible_plan_ = ros::Time::now();
        last_cmd_ = cmd_vel.twist;
        return;
  
      }
       else {
        status_.status = planner_cspace_msgs::PlannerStatus::DOING;
        // pub_status_.publish(status_);
      }
      no_infeasible_plans_ = 0; // reset number of infeasible solutions in a row

      pub_status_.publish(status_); 

      planner_->visualize();
      visualization_->publishObstacles(obstacles_);
      visualization_->publishViaPoints(via_points_);
    }

}

RobotFootprintModelPtr TebOptimNode::getRobotFootprintFromParamServer(const ros::NodeHandle& global_nh,const ros::NodeHandle& nh, const TebConfig& config, int& no_try_plans)
{
  // nh is private nh (e.g./sirbot2/teb_planner_node/teb_config/footprint_model/type) 
  std::string model_name;
  // Check dev_model first, then fall back to hw/model
  std::string model_name_setting;
  bool model_found = false;

  // First, check dev_model parameter
  if (global_nh.getParam("dev_model", model_name_setting))
  {
    if (!model_name_setting.empty() && model_name_setting != "none")
    {
      model_found = true;
      ROS_INFO("Using dev_model parameter: '%s'", model_name_setting.c_str());
    }
  }

  // If dev_model is not found or is "none", check hw/model
  if (!model_found)
  {
    if (global_nh.getParam("hw/model", model_name_setting))
    {
      model_found = true;
      ROS_INFO("Using hw/model parameter: '%s'", model_name_setting.c_str());
    }
  }

  // If neither parameter is found, use point model
  if (!model_found)
  {
    ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model===.");
    return boost::make_shared<PointRobotFootprint>();
  }
  if (model_name_setting.compare("square") == 0) 
  {
    model_name = "two_circles";
    ROS_INFO("Robot model is square, using polygon footprint model");
  } 
  else if (model_name_setting.compare("circle") == 0 || model_name_setting.compare("c_order") == 0){
    model_name = "circular";
    ROS_INFO("Robot model is circular, using polygon footprint model");
  } 
  else if (model_name_setting.compare("square2") == 0) 
  {
    model_name = "square2";
    ROS_INFO("Robot model is square2, using polygon footprint model");
  } 
  else if (model_name_setting.compare("circle2") == 0) 
  {
    model_name = "circle2";
    ROS_INFO("Robot model is circle2, using polygon footprint model");
  }    
  // // cart 
  // if (model_name_setting.compare("cart") == 0) 
  // {
  //   model_name = "cart";
  //   ROS_INFO("Robot model is cart, using polygon footprint model");
  // }   
  // if (model_name_setting.compare("s_tray") == 0) 
  // {
  //   model_name = "s_tray";
  //   ROS_INFO("Robot model is s_tray, using polygon footprint model");
  // }   


  // point  
  if (model_name.compare("point") == 0)
  {
    ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
    return boost::make_shared<PointRobotFootprint>(config.obstacles.min_obstacle_dist);
  }
  
  // circular
  if (model_name.compare("circular") == 0)
  {
    // get radius
    double radius;
    if (!nh.getParam("footprint_model/radius", radius))
    {
      ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/radius' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius <<"m) loaded for trajectory optimization.");
    return boost::make_shared<CircularRobotFootprint>(radius);
  }
  
  // circular2
  if (model_name.compare("circle2") == 0)
  {
    // get radius
    double radius;
    if (!nh.getParam("footprint_model/radius", radius))
    {
      ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/radius' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    radius = radius + 0.05; // adjust for circle2 shape
    ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius <<"m) loaded for trajectory optimization.");
    return boost::make_shared<CircularRobotFootprint>(radius);
  }
  // line
  if (model_name.compare("line") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end"))
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get line coordinates
    std::vector<double> line_start, line_end;
    nh.getParam("footprint_model/line_start", line_start);
    nh.getParam("footprint_model/line_end", line_end);
    if (line_start.size() != 2 || line_end.size() != 2)
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    
    ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] <<"]m, line_end: ["
                     << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
    return boost::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()), config.obstacles.min_obstacle_dist);
  }
  
  // two circles
  if (model_name.compare("two_circles") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") 
        || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
    {
      ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                       << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    nh.getParam("footprint_model/front_offset", front_offset);
    nh.getParam("footprint_model/front_radius", front_radius);
    nh.getParam("footprint_model/rear_offset", rear_offset);
    nh.getParam("footprint_model/rear_radius", rear_radius);
    ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius 
                    << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }


  // two circles
  if (model_name.compare("square2") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") 
        || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
    {
      ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                       << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    nh.getParam("footprint_model/front_offset", front_offset);
    nh.getParam("footprint_model/front_radius", front_radius);
    nh.getParam("footprint_model/rear_offset", rear_offset);
    nh.getParam("footprint_model/rear_radius", rear_radius);
    double adjustment = 0.05; // adjust for square2 shape
    front_offset = front_offset + 0.01;
    rear_offset = rear_offset + adjustment;
    front_radius = front_radius + adjustment;
    rear_radius = rear_radius + adjustment;
    no_try_plans = no_try_plans + 3; // square2 is more difficult to plan for
    ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius 
                    << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }
  // two circles
  if (model_name.compare("cart") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") 
        || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
    {
      ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                       << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    nh.getParam("footprint_model/front_offset", front_offset);
    nh.getParam("footprint_model/front_radius", front_radius);
    nh.getParam("footprint_model/rear_offset", rear_offset);
    nh.getParam("footprint_model/rear_radius", rear_radius);
    double adjustment = 0.05; // adjust for square2 shape
    front_offset = front_offset + adjustment;
    rear_offset = rear_offset + adjustment;
    front_radius = front_radius + adjustment;
    rear_radius = rear_radius + adjustment;
    no_try_plans = no_try_plans + 3; // cart is more difficult to plan for
    ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius 
                    << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }

  // two circles
  if (model_name.compare("s_tray") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") 
        || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
    {
      ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                       << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    nh.getParam("footprint_model/front_offset", front_offset);
    nh.getParam("footprint_model/front_radius", front_radius);
    nh.getParam("footprint_model/rear_offset", rear_offset);
    nh.getParam("footprint_model/rear_radius", rear_radius);
    double adjustment = 0.05; // adjust for square2 shape
    front_offset = front_offset + adjustment;
    rear_offset = rear_offset + adjustment;
    front_radius = front_radius + 0.02;
    rear_radius = rear_radius + 0.02;
    ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius 
                    << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }


  // polygon
  if (model_name.compare("polygon") == 0)
  {

    // check parameters
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc) )
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/vertices' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get vertices
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      try
      {
        Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
        ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
        return boost::make_shared<PolygonRobotFootprint>(polygon);
      } 
      catch(const std::exception& ex)
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
    }
    else
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    
  }
  
  // otherwise
  ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
  return boost::make_shared<PointRobotFootprint>();
}
         
Point2dContainer TebOptimNode::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name)
{
   // Make sure we have an array of at least 3 elements.
   if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       footprint_xmlrpc.size() < 3)
   {
     ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
     throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                              "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
   }
 
   Point2dContainer footprint;
   Eigen::Vector2d pt;
 
   for (int i = 0; i < footprint_xmlrpc.size(); ++i)
   {
     // Make sure each element of the list is an array of size 2. (x and y coordinates)
     XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
     if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
         point.size() != 2)
     {
       ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                 "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                  full_param_name.c_str());
       throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x() = getNumberFromXMLRPC(point[ 0 ], full_param_name);
    pt.y() = getNumberFromXMLRPC(point[ 1 ], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

double TebOptimNode::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str());
     throw std::runtime_error("Values in the footprint specification must be numbers");
   }
   return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}


void TebOptimNode::updateObstacleContainerWithCostmapConverter()
{
  if (!costmap_converter_)
    return;
    
  //Get obstacles from costmap converter
  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  if (!obstacles)
    return;

  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::Polygon* polygon = &obstacle->polygon;

    if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
    {
      obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
    }
    else if (polygon->points.size()==1) // Point
    {
      obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
    }
    else if (polygon->points.size()==2) // Line
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        polygon->points[1].x, polygon->points[1].y )));
    }
    else if (polygon->points.size()>2) // Real polygon
    {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (std::size_t j=0; j<polygon->points.size(); ++j)
        {
            polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
    }

    // Set velocity, if obstacle is moving
    if(!obstacles_.empty())
      obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
  }
}



void TebOptimNode::reconfigureCallback(TebLocalPlannerReconfigureConfig& config, uint32_t level) {
    cfg_.reconfigure(config);
    ros::NodeHandle nh_private("~");
    ROS_INFO("TEB parameters updated via dynamic reconfigure");
}


void TebOptimNode::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
  if (!msg->poses.empty()) { // Check if the path is not empty
          first_waypoint_ = msg->poses[0]; // Assign the first waypoint (PoseStamped)
          // ROS_INFO("First waypoint received: x=%f, y=%f, z=%f",
          //         first_waypoint_.pose.position.x,
          //         first_waypoint_.pose.position.y,
          //         first_waypoint_.pose.position.z);
          end_waypoint_ = msg->poses.back(); // Assign the last waypoint (PoseStamped) 
          // ROS_INFO("End waypoint received: x=%f, y=%f, z=%f",
          //         end_waypoint_.pose.position.x,
          //         end_waypoint_.pose.position.y,
          //         end_waypoint_.pose.position.z);       
          // has_first_waypoint_ = true; // Set the flag to true
      } else {
          ROS_WARN("Received empty path!");
          // has_first_waypoint_ = false; // Reset the flag if the path is empty
      }
}

void TebOptimNode::pathSegmentCallback(const nav_msgs::Path::ConstPtr& msg) {

// Desired frequency: 5 Hz (0.2 seconds per message)
  ros::Time current_time = ros::Time::now();


  // } 
  // **PUBLISH THE RECEIVED PATH**
  nav_msgs::Path received_path;
  received_path.header = msg->header;
  received_path.header.stamp = current_time; // Update timestamp

  double distance_to_end_waypoint = std::hypot(
      start_.pose.position.x - end_waypoint_.pose.position.x,
      start_.pose.position.y - end_waypoint_.pose.position.y
    );
  // ROS_INFO("distance_to_end_waypoint: %.2f", distance_to_end_waypoint);
  if (distance_to_end_waypoint < 3.0) {
    if (msg->poses.size() < 3) {
      // If less than 3 poses, no orientation filtering possible, just copy
      received_path.poses = msg->poses;
    } else {
      // For 3+ poses, apply orientation outlier filtering (no distance filter)
      const double ORIENTATION_OUTLIER_THRESHOLD = M_PI / 2.0;  // 90 degrees
      received_path.poses.reserve(msg->poses.size()); // avoid relloc many times by reserving space (optional optimization)
      // Always add the first pose
      received_path.poses.push_back(msg->poses[0]);
      // Process middle waypoints (check for orientation outliers)
      for (size_t i = 1; i < msg->poses.size() - 1; ++i) {
        const geometry_msgs::PoseStamped& prev_pose = msg->poses[i - 1];
        const geometry_msgs::PoseStamped& curr_pose = msg->poses[i];
        const geometry_msgs::PoseStamped& next_pose = msg->poses[i + 1];

        // Get yaw angles from quaternions
        double prev_yaw = tf2::getYaw(prev_pose.pose.orientation);
        double curr_yaw = tf2::getYaw(curr_pose.pose.orientation);
        double next_yaw = tf2::getYaw(next_pose.pose.orientation);

        // Calculate normalized angle differences
        double diff_with_prev = std::abs(g2o::normalize_theta(curr_yaw - prev_yaw));
        double diff_with_next = std::abs(g2o::normalize_theta(next_yaw - curr_yaw));

        // Skip if this waypoint is an orientation outlier (differs >90° from BOTH neighbors)
        if (diff_with_prev > ORIENTATION_OUTLIER_THRESHOLD && diff_with_next > ORIENTATION_OUTLIER_THRESHOLD) {
            ROS_WARN_THROTTLE(1.0, "Filtering out orientation outlier at index %zu (diff_prev: %.2f deg, diff_next: %.2f deg)",
                    i, diff_with_prev * 180.0 / M_PI, diff_with_next * 180.0 / M_PI);
          continue;  // Skip this outlier waypoint
        }

        // Keep this waypoint
        received_path.poses.push_back(curr_pose);
      }

      // Always add the last pose
      received_path.poses.push_back(msg->poses.back());
    }
  } else {
    // If stop_and_go_active_ is true or distance_to_end_waypoint >= 2.0, just copy the poses
    received_path.poses = msg->poses;
  }

  // Validate last waypoint
  const auto& last_pose = received_path.poses.back().pose.position;
  const double MAX_COORD_VALUE = 10000.0;  // Adjust as needed
  if (std::abs(last_pose.x) > MAX_COORD_VALUE || std::abs(last_pose.y) > MAX_COORD_VALUE) {
    ROS_ERROR("REJECTED: Last waypoint coordinates out of bounds: x=%f, y=%f (max=%f)",
              last_pose.x, last_pose.y, MAX_COORD_VALUE);
    // return;  // Reject this path
    received_path.poses.pop_back(); // Remove the last waypoint
    ROS_WARN("After removing last waypoint, new last waypoint x: %f, y: %f", received_path.poses.back().pose.position.x, received_path.poses.back().pose.position.y);
  }

  
  path_segment_pub_.publish(received_path); // Publish the modified path segment

  {
    std::lock_guard<std::mutex> lock(global_plan_mutex_);
    // global_plan_ = received_path.poses; // Update global_plan_ with modified poses
    global_plan_ = std::move(received_path.poses); // Move instead of copy
  }


}

void TebOptimNode::Callback_navistate(const std_msgs::Int16MultiArray::ConstPtr &msg) {
  if (msg->data.size() == 14) {
      navi_status_ = *msg;
      nav_state_ = static_cast<NavState>(navi_status_.data[static_cast<size_t>(NavInd::NAVI_STATE)]);
      goal_state_  = static_cast<GoalState>(navi_status_.data[static_cast<size_t>(NavInd::GOAL_STATE)]);
      // ROS_INFO("static_cast<int>(navi_status_.data[static_cast<REPLAN_COUNT>(NavInd::REPLAN_COUNT)])  %d", static_cast<int>(navi_status_.data[static_cast<size_t>(NavInd::REPLAN_COUNT)]));
      // ROS_DEBUG("navi State: %d", static_cast<int>(nav_state_));
      // ROS_INFO ("goal_state: %d", static_cast<int>(goal_state_));
  }

}


void TebOptimNode::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    ROS_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}

void TebOptimNode::spin(){
  ros::Rate rate(controller_frequency_); // Adjust frequency as needed
  while (ros::ok()) {
    //check that the observation buffers for the costmap are current, we don't want to drive blind
    if(!costmap_ros_->isCurrent()){
      ROS_INFO("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishEmptyPath(); 
      continue;
    }

      mainCycleCallback();

      ros::spinOnce();
      rate.sleep();

  }
}   

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "teb_planner");
    ros::NodeHandle nh("~");
    // ros::NodeHandle nh;

    TebOptimNode node(nh);
    node.spin();
    // ros::spin();
    return 0;
}