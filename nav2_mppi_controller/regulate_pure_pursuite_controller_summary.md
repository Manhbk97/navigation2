Regulated Pure Pursuit Controller - Algorithm Documentation
Overview
The Regulated Pure Pursuit Controller is a ROS2 Nav2 controller plugin that implements an enhanced version of the classic Pure Pursuit path tracking algorithm. It adds several "regulations" (constraints) to improve performance and safety.
Core Algorithm Flow

┌─────────────────────────────────────────────────────────────────┐
│                    computeVelocityCommands()                     │
│                      (Main Control Loop)                         │
└──────────────────────────┬──────────────────────────────────────┘
                           │
                           ▼
        ┌──────────────────────────────────────────┐
        │  1. Transform Global Path to Robot Frame │
        │     (path_handler_->transformGlobalPlan) │
        └──────────────┬───────────────────────────┘
                       │
                       ▼
        ┌──────────────────────────────────────────┐
        │  2. Calculate Lookahead Distance         │
        │     - Velocity-scaled or static          │
        │     - Check for cusps (velocity changes) │
        └──────────────┬───────────────────────────┘
                       │
                       ▼
        ┌──────────────────────────────────────────┐
        │  3. Find Lookahead Point (Carrot)        │
        │     - Circle-line intersection           │
        │     - Interpolation if needed            │
        └──────────────┬───────────────────────────┘
                       │
                       ▼
        ┌──────────────────────────────────────────┐
        │  4. Calculate Path Curvature             │
        │     k = 2y / (x² + y²)                   │
        └──────────────┬───────────────────────────┘
                       │
                       ▼
        ┌──────────────────────────────────────────┐
        │  5. Decision: Rotation Mode?             │
        └──────────────┬───────────────────────────┘
                       │
         ┌─────────────┴─────────────┐
         │                           │
         ▼                           ▼
  ┌─────────────┐            ┌─────────────────┐
  │ Rotate Mode │            │  Tracking Mode  │
  └──────┬──────┘            └────────┬────────┘
         │                            │
         ▼                            ▼
  ┌──────────────────┐    ┌─────────────────────────┐
  │ rotateToHeading()│    │   applyConstraints()    │
  │  - Linear = 0    │    │  - Curvature regulation │
  │  - Angular vel   │    │  - Cost regulation      │
  │  - Accel limits  │    │  - Approach velocity    │
  └──────┬───────────┘    └──────────┬──────────────┘
         │                           │
         │                           ▼
         │               ┌────────────────────────┐
         │               │ Angular = Linear * k   │
         │               └────────────┬───────────┘
         │                            │
         └────────────┬───────────────┘
                      │
                      ▼
        ┌──────────────────────────────────────────┐
        │  6. Collision Check                      │
        │     (collision_checker_)                 │
        └──────────────┬───────────────────────────┘
                       │
                       ▼
        ┌──────────────────────────────────────────┐
        │  7. Publish Command Velocity             │
        │     (TwistStamped)                       │
        └──────────────────────────────────────────┘
Key Functions
Lifecycle Management Functions
configure() - Initialize the controller
Sets up parameter handler, path handler, collision checker
Creates publishers for visualization
Gets control frequency
activate() / deactivate() - Manage lifecycle state
cleanup() - Resource cleanup
reset() - Reset controller state flags
Main Control Functions
computeVelocityCommands() - MAIN CONTROL LOOP
Transforms global path to robot frame
Computes lookahead point
Calculates velocities
Performs collision checking
Returns command velocities
Path Following Functions
getLookAheadDistance()
Computes lookahead distance based on velocity (adaptive) or static
Formula: lookahead = velocity * lookahead_time
Clamped between min and max values
getLookAheadPoint()
Finds the "carrot" point on path at lookahead distance
Uses circle-segment intersection algorithm
Can interpolate beyond goal if configured
circleSegmentIntersection()
Geometric calculation for finding intersection point
Based on circle-line intersection formula
Returns point on segment that intersects circle of radius r
calculateCurvature() (free function)
Computes path curvature: k = 2y / (x² + y²)
Used to calculate angular velocity: ω = v * k
Rotation Control Functions
shouldRotateToPath()
Checks if robot should rotate to align with path
Triggers when angle to path exceeds threshold
shouldRotateToGoalHeading()
Checks if robot should rotate to goal orientation
Triggers when within goal distance tolerance
rotateToHeading()
In-place rotation controller
Applies angular acceleration limits
Slows down to avoid overshooting
Velocity Regulation Functions
applyConstraints()
Curvature constraint: Slow down on sharp turns
Cost constraint: Slow down near obstacles
Approach constraint: Slow down near goal
Takes minimum of all constraints
Special Features
findVelocitySignChange()
Detects "cusps" (direction reversals) in path
Uses dot product of consecutive segments
Limits lookahead to cusp distance for reversing support
setPlan() - Update global path
setSpeedLimit() - Dynamic speed limiting
cancel() - Handle goal cancellation with deceleration
Helper Components
PathHandler
Transforms global path to robot frame
Handles coordinate transformations
CollisionChecker
Checks for imminent collisions
Evaluates costmap costs at poses
ParameterHandler
Manages dynamic reconfiguration
Stores all controller parameters
Control Algorithm Details
Pure Pursuit Geometry
The pure pursuit algorithm follows these steps:
Lookahead Point Selection: Find point on path at distance L from robot
Curvature Calculation: κ = 2y / (x² + y²) where (x,y) is carrot in robot frame
Angular Velocity: ω = v * κ (couples linear and angular velocity)
Regulations (Constraints)
The controller applies multiple velocity constraints:
Curvature Regulation (regulated_pure_pursuit_controller.cpp#L462-L465)
Reduces speed on sharp curves
Prevents excessive lateral acceleration
Cost Regulation (regulated_pure_pursuit_controller.cpp#L468-L470)
Slows down near obstacles (high costmap values)
Provides safety margin
Approach Velocity (regulated_pure_pursuit_controller.cpp#L477-L479)
Gradually reduces speed approaching goal
Ensures smooth arrival
Rotation to Heading (regulated_pure_pursuit_controller.cpp#L239-L246)
In-place rotation when path direction differs significantly
Rotation at goal for orientation alignment
State Machine

┌─────────────┐
│   IDLE      │
└──────┬──────┘
       │ setPlan()
       ▼
┌─────────────┐     shouldRotateToPath?
│  TRACKING   ├────────────────┐
└──────┬──────┘                │
       │                       ▼
       │              ┌─────────────────┐
       │              │ ROTATE_TO_PATH  │
       │              └────────┬────────┘
       │                       │
       │ Near goal?            │
       ▼                       │
┌─────────────────┐            │
│ ROTATE_TO_GOAL  │◄───────────┘
└────────┬────────┘
         │ Goal reached
         ▼
    ┌────────┐
    │  DONE  │
    └────────┘
Key Parameters
lookahead_dist / lookahead_time: Lookahead distance calculation
desired_linear_vel: Target forward speed
rotate_to_heading_angular_vel: Rotation speed
max_angular_accel: Angular acceleration limit
regulated_linear_scaling_min_radius: Curvature regulation threshold
use_velocity_scaled_lookahead_dist: Adaptive lookahead
allow_reversing: Enable backward motion
use_collision_detection: Enable safety checking
Publishers (Visualization/Debug)
/received_global_plan: Transformed path in robot frame
/lookahead_point: Current carrot point
/curvature_lookahead_point: Curvature calculation point
/is_rotating_to_heading: Rotation mode indicator
Summary
The Regulated Pure Pursuit Controller enhances the classic pure pursuit algorithm with: ✓ Adaptive lookahead based on velocity
✓ Multiple velocity constraints for safety and smoothness
✓ Rotation behaviors for better path tracking
✓ Collision avoidance integration
✓ Reverse driving support with cusp detection
✓ Smooth goal approach with deceleration This makes it suitable for dynamic environments and provides smoother, safer robot navigation compared to basic pure pursuit.



#### Carrot Pose vs Robot Pose
Carrot Pose (carrot_pose)
The carrot_pose is the lookahead point (target point) on the path that the robot is trying to reach. It's called a "carrot" metaphorically - like dangling a carrot in front of a donkey to guide it forward. Key characteristics:
Location: A point on the global path at a distance lookahead_dist ahead of the robot
Calculated at: regulated_pure_pursuit_controller.cpp:207

auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
Purpose: The robot steers toward this point to follow the path
Frame: Expressed in the robot's base frame (after path transformation)
Dynamic: Moves along the path as the robot moves
Distance from robot: Typically 0.5-2.0 meters ahead (velocity-dependent)
Robot Pose (pose)
The robot_pose (passed as pose parameter) is the current actual position and orientation of the robot in the world. Key characteristics:
Location: Where the robot currently is
Source: Passed into computeVelocityCommands() as a parameter
Frame: Usually in map/odom frame initially, then used as the origin for transformations
Static: Represents the current moment in time
Visual Diagram

        Robot Frame (origin at robot)
              
              Path
         ╱────────────────→ Goal
        ╱
       ╱
      ●────────● Carrot Pose (lookahead point)
      │        ↑
      │        │ 
      │        └─ lookahead_dist (e.g., 1.0m)
      │
      ▲ Robot Pose
     ╱│╲ (0, 0) in transformed frame
    ╱ │ ╲
   ╱  │  ╲
  🤖 Current robot position
In the Code
Where they're used:
Robot Pose - Line 165:

geometry_msgs::msg::TwistStamped computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,  // ← Robot's current pose
  ...
Path transformed to robot frame - Line 184-185:

auto transformed_plan = path_handler_->transformGlobalPlan(
  pose, ...);  // Uses robot pose as transform origin
Carrot Pose computed - Line 207:

auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
Curvature calculated from carrot - Line 213:

double lookahead_curvature = calculateCurvature(carrot_pose.pose.position);
// Uses carrot position in robot frame: (x, y)
The Pure Pursuit Formula
The pure pursuit algorithm uses the carrot pose to compute steering:

// Line 148-161: Curvature calculation
double calculateCurvature(geometry_msgs::msg::Point lookahead_point)
{
  const double carrot_dist2 = 
    (lookahead_point.x * lookahead_point.x) +  // Carrot X in robot frame
    (lookahead_point.y * lookahead_point.y);   // Carrot Y in robot frame
    
  return 2.0 * lookahead_point.y / carrot_dist2;  // k = 2y / (x² + y²)
}
Then angular velocity is: ω = v × k (Line 271)
Key Insight
Robot pose: "Where am I?"
Carrot pose: "Where should I aim for next?"
The controller computes velocities to move the robot toward the carrot, which continuously slides forward along the path as the robot moves. This creates smooth path following behavior. The carrot is published for visualization at Line 209:

carrot_pub_->publish(createCarrotMsg(carrot_pose));
You can visualize this in RViz on the /lookahead_point topic to see the carrot moving along the path!