# MPPI Visual Architecture & Flow Diagrams

## 1. Class Hierarchy and Relationships

### Overview Architecture
```
┌─────────────────────────────────────────────────────────────────────┐
│                    nav2_core::Controller                             │
│                      (ROS 2 Plugin Interface)                        │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ inherits
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                        MPPIController                                 │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │ Main Members:                                                   │  │
│  │  - optimizer_               : Optimizer                         │  │
│  │  - path_handler_            : PathHandler                       │  │
│  │  - trajectory_visualizer_   : TrajectoryVisualizer              │  │
│  │  - parameters_handler_      : ParametersHandler                 │  │
│  │  - costmap_ros_             : nav2_costmap_2d::Costmap2DROS*    │  │
│  │  - tf_buffer_               : tf2_ros::Buffer*                  │  │
│  │                                                                  │  │
│  │ Key Methods:                                                     │  │
│  │  + computeVelocityCommands() → TwistStamped                     │  │
│  │  + setPlan(path)             → Set global plan                  │  │
│  │  + configure(), activate(), deactivate(), cleanup()             │  │
│  └────────────────────────────────────────────────────────────────┘  │
└─────────┬──────────────┬───────────────┬──────────────┬─────────────┘
          │              │               │              │
          │ owns         │ owns          │ owns         │ owns
          ▼              ▼               ▼              ▼
    ┌──────────┐  ┌────────────┐  ┌──────────────┐  ┌──────────────┐
    │Optimizer │  │    Path    │  │  Trajectory  │  │  Parameters  │
    │          │  │   Handler  │  │  Visualizer  │  │   Handler    │
    └─────┬────┘  └────────────┘  └──────────────┘  └──────────────┘
          │
          │ owns (composition)
          │
          └─────────┬──────────────┬───────────────┬──────────────┐
                    │              │               │              │
                    ▼              ▼               ▼              ▼
              ┌──────────┐   ┌──────────┐   ┌───────────┐  ┌──────────┐
              │ Critic   │   │  Noise   │   │  Motion   │  │   Data   │
              │ Manager  │   │Generator │   │  Model*   │  │  Models  │
              └────┬─────┘   └──────────┘   └─────┬─────┘  └──────────┘
                   │                               │
                   │ manages (pluginlib)           │ points to one
                   │                               │
                   ▼                               ▼
              ┌──────────────────────────┐   ┌─────────────────────┐
              │  mppi::critics::         │   │  MotionModel        │
              │  CriticFunction          │   │  (abstract base)    │
              │  (abstract base)         │   └──────────┬──────────┘
              └──────────┬───────────────┘              │
                         │                              │ inherited by
                         │ inherited by                 │
                         │ (11 critic types)            │
                         │                              │
        ┌────────────────┼─────────────────┐            │
        │                │                 │            │
        ▼                ▼                 ▼            ▼
┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────────┐
│ Obstacles   │  │    Goal     │  │    Path     │  │  DiffDrive     │
│   Critic    │  │   Critic    │  │   Follow    │  │  MotionModel   │
└─────────────┘  └─────────────┘  │   Critic    │  └────────────────┘
                                  └─────────────┘           │
┌─────────────┐  ┌─────────────┐  ┌─────────────┐          │
│    Cost     │  │  GoalAngle  │  │  PathAlign  │          ▼
│   Critic    │  │   Critic    │  │   Critic    │  ┌────────────────┐
└─────────────┘  └─────────────┘  └─────────────┘  │   Ackermann    │
                                                    │  MotionModel   │
┌─────────────┐  ┌─────────────┐  ┌─────────────┐  └────────────────┘
│  PathAngle  │  │ Constraint  │  │   Prefer    │           │
│   Critic    │  │   Critic    │  │   Forward   │           ▼
└─────────────┘  └─────────────┘  │   Critic    │  ┌────────────────┐
                                  └─────────────┘  │      Omni      │
┌─────────────┐  ┌─────────────┐                   │  MotionModel   │
│  Twirling   │  │  Velocity   │                   └────────────────┘
│   Critic    │  │  Deadband   │
└─────────────┘  │   Critic    │
                 └─────────────┘
```

### Detailed Component Breakdown

#### 1. Optimizer Core Components
```
┌──────────────────────────────────────────────────────────────┐
│                        Optimizer                              │
├──────────────────────────────────────────────────────────────┤
│ Responsibilities:                                             │
│  • Generate noised trajectory samples                         │
│  • Score trajectories via critics                             │
│  • Compute weighted control update (MPPI algorithm)           │
│  • Integrate velocities to poses                              │
│                                                               │
│ Key Members:                                                  │
│  - critic_manager_         : CriticManager                    │
│  - noise_generator_        : NoiseGenerator                   │
│  - motion_model_           : MotionModel*                     │
│  - state_                  : models::State                    │
│  - generated_trajectories_ : models::Trajectories             │
│  - control_sequence_       : models::ControlSequence          │
│  - critics_data_           : CriticData                       │
│  - settings_               : models::OptimizerSettings        │
│                                                               │
│ Key Methods:                                                  │
│  + evalControl() → void                                       │
│  + optimize() → void                                          │
│  + generateNoisedTrajectories() → void                        │
│  + integrateStateVelocities() → void                          │
│  + updateControlSequence() → void                             │
│  + updateStateScores() → void                                 │
└──────────────────────────────────────────────────────────────┘
```

#### 2. Motion Model Hierarchy (Strategy Pattern)
```
┌──────────────────────────────────────────────────────────────┐
│              MotionModel (abstract base class)                │
├──────────────────────────────────────────────────────────────┤
│ Virtual Methods:                                              │
│  + predict(state, dt) → void                                  │
│  + isHolonomic() → bool                                       │
│  + applyConstraints(vx, vy, wz) → void                        │
└───────────────────────────┬──────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
        ▼                   ▼                   ▼
┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│  DiffDrive   │   │  Ackermann   │   │     Omni     │
│ MotionModel  │   │ MotionModel  │   │ MotionModel  │
├──────────────┤   ├──────────────┤   ├──────────────┤
│ Non-holonomic│   │ Non-holonomic│   │  Holonomic   │
│              │   │              │   │              │
│ isHolonomic()│   │ isHolonomic()│   │ isHolonomic()│
│  → false     │   │  → false     │   │  → true      │
│              │   │              │   │              │
│ Constraints: │   │ Constraints: │   │ Constraints: │
│  • vx limits │   │  • vx limits │   │  • vx limits │
│  • wz limits │   │  • wz limits │   │  • vy limits │
│  • vy = 0    │   │  • vy = 0    │   │  • wz limits │
│              │   │  • min turn  │   │              │
│              │   │    radius    │   │              │
└──────────────┘   └──────────────┘   └──────────────┘
```

#### 3. Critic System Architecture (Plugin-Based)
```
┌──────────────────────────────────────────────────────────────┐
│                      CriticManager                            │
├──────────────────────────────────────────────────────────────┤
│ Responsibilities:                                             │
│  • Load critic plugins dynamically (pluginlib)                │
│  • Coordinate evaluation across all critics                   │
│  • Manage critic lifecycle (on_configure, on_activate)        │
│                                                               │
│ Key Methods:                                                  │
│  + loadCritics(critics_list) → void                           │
│  + evalTrajectoriesScores(CriticData&) → void                │
│                                                               │
│ Data Flow:                                                    │
│  1. Receives CriticData reference                             │
│  2. Iterates through all loaded critics                       │
│  3. Each critic scores trajectories in-place                  │
│  4. Costs accumulate in CriticData.costs array                │
└───────────────────────────┬──────────────────────────────────┘
                            │
                            │ manages collection of
                            ▼
┌──────────────────────────────────────────────────────────────┐
│       mppi::critics::CriticFunction (abstract base)           │
├──────────────────────────────────────────────────────────────┤
│ Pure Virtual Methods:                                         │
│  + score(CriticData& data) → void                             │
│                                                               │
│ Virtual Methods:                                              │
│  + initialize() → void                                        │
│  + on_configure(parent, costmap_ros, name, tf, ...) → void   │
│                                                               │
│ Common Members (in derived classes):                          │
│  - enabled_           : bool                                  │
│  - weight_            : float                                 │
│  - power_             : int                                   │
│  - costmap_ros_       : Costmap2DROS*                         │
│  - parameters_handler_: ParametersHandler*                    │
└───────────────────────────┬──────────────────────────────────┘
                            │
                            │ 11 implementations
                            │
        ┌───────────────────┼───────────────────┬──────────────┐
        │                   │                   │              │
        ▼                   ▼                   ▼              ▼
┌──────────────┐   ┌──────────────┐   ┌──────────────┐  ┌─────────────┐
│  Obstacles   │   │     Cost     │   │     Goal     │  │  GoalAngle  │
│   Critic     │   │    Critic    │   │   Critic     │  │   Critic    │
├──────────────┤   ├──────────────┤   ├──────────────┤  ├─────────────┤
│ Footprint-   │   │ Costmap-     │   │ Position     │  │ Orientation │
│ based        │   │ inflation    │   │ distance to  │  │ alignment   │
│ collision    │   │ based        │   │ goal         │  │ with goal   │
│ checking     │   │ collision    │   │              │  │             │
└──────────────┘   └──────────────┘   └──────────────┘  └─────────────┘

        │                   │                   │              │
        ▼                   ▼                   ▼              ▼
┌──────────────┐   ┌──────────────┐   ┌──────────────┐  ┌─────────────┐
│ PathFollow   │   │  PathAlign   │   │  PathAngle   │  │ Constraint  │
│   Critic     │   │   Critic     │   │   Critic     │  │   Critic    │
├──────────────┤   ├──────────────┤   ├──────────────┤  ├─────────────┤
│ Approximate  │   │ Precise path │   │ Heading      │  │ Enforce     │
│ path         │   │ alignment    │   │ alignment    │  │ kinematic/  │
│ following    │   │              │   │ with path    │  │ dynamic     │
│              │   │              │   │              │  │ limits      │
└──────────────┘   └──────────────┘   └──────────────┘  └─────────────┘

        │                   │                   │
        ▼                   ▼                   ▼
┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│PreferForward │   │  Twirling    │   │  Velocity    │
│   Critic     │   │   Critic     │   │  Deadband    │
├──────────────┤   ├──────────────┤   ├──────────────┤
│ Bias toward  │   │ Prevent      │   │ Restrict     │
│ forward      │   │ spinning in  │   │ velocities   │
│ motion       │   │ place (omni) │   │ in deadband  │
└──────────────┘   └──────────────┘   └──────────────┘
```

#### 4. Data Models (Eigen-based, in models/ directory)
```
┌──────────────────────────────────────────────────────────────┐
│                     models::State                             │
├──────────────────────────────────────────────────────────────┤
│ Velocity States (Eigen arrays):                               │
│  - vx  : Eigen::ArrayXXf [time_steps × batch_size]           │
│  - vy  : Eigen::ArrayXXf [time_steps × batch_size]           │
│  - wz  : Eigen::ArrayXXf [time_steps × batch_size]           │
│                                                               │
│ Control Velocities (Eigen arrays):                            │
│  - cvx : Eigen::ArrayXXf [time_steps × batch_size]           │
│  - cvy : Eigen::ArrayXXf [time_steps × batch_size]           │
│  - cwz : Eigen::ArrayXXf [time_steps × batch_size]           │
│                                                               │
│ Current State (scalars):                                      │
│  - pose  : geometry_msgs::msg::PoseStamped                    │
│  - speed : geometry_msgs::msg::Twist                          │
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│                  models::Trajectories                         │
├──────────────────────────────────────────────────────────────┤
│  - x    : Eigen::ArrayXXf [time_steps × batch_size]          │
│  - y    : Eigen::ArrayXXf [time_steps × batch_size]          │
│  - yaws : Eigen::ArrayXXf [time_steps × batch_size]          │
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│                models::ControlSequence                        │
├──────────────────────────────────────────────────────────────┤
│  - vx : Eigen::ArrayXf [time_steps]                           │
│  - vy : Eigen::ArrayXf [time_steps]                           │
│  - wz : Eigen::ArrayXf [time_steps]                           │
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│                     models::Path                              │
├──────────────────────────────────────────────────────────────┤
│  - x    : Eigen::ArrayXf [path_length]                        │
│  - y    : Eigen::ArrayXf [path_length]                        │
│  - yaws : Eigen::ArrayXf [path_length]                        │
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│                   models::CriticData                          │
├──────────────────────────────────────────────────────────────┤
│ References passed to critics:                                 │
│  - state           : State&                                   │
│  - trajectories    : Trajectories&                            │
│  - path            : Path&                                    │
│  - costs           : Eigen::ArrayXf& [batch_size]             │
│  - model_dt        : float                                    │
│  - motion_model    : MotionModel*                             │
│  - goal            : geometry_msgs::msg::Pose                 │
│  - furthest_reached_path_point : Eigen::ArrayXf [batch_size]  │
└──────────────────────────────────────────────────────────────┘
```

#### 5. Utility Components
```
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│   PathHandler    │  │  NoiseGenerator  │  │ParametersHandler│
├──────────────────┤  ├──────────────────┤  ├──────────────────┤
│ • Transform path │  │ • Generate       │  │ • Template-based │
│   to local frame │  │   Gaussian noise │  │   parameter      │
│ • Prune based on │  │ • Runs in thread │  │   management     │
│   costmap bounds │  │ • Pre-generates  │  │ • Dynamic        │
│ • Handle path    │  │   noise samples  │  │   reconfigure    │
│   inversions     │  │                  │  │ • Thread-safe    │
└──────────────────┘  └──────────────────┘  └──────────────────┘

┌──────────────────┐  ┌──────────────────────────────────────┐
│ Trajectory       │  │            utils.hpp                  │
│  Visualizer      │  ├──────────────────────────────────────┤
├──────────────────┤  │ Helper Functions:                     │
│ • Publishes viz  │  │ • normalize_angle()                   │
│   markers        │  │ • shortest_angular_distance()         │
│ • Shows sampled  │  │ • findPathInversions()                │
│   trajectories   │  │ • savitzkyGolayFilter()               │
│ • Shows optimal  │  │ • transformPose()                     │
│   trajectory     │  │ • posePointAngle()                    │
│ • RViz debugging │  │                                       │
└──────────────────┘  └──────────────────────────────────────┘
```

## 2. Data Flow During One Control Cycle

```
TIME: t = 0  (Current timestep)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

┌────────────────────┐
│  ROS 2 Controller  │  Receives: /cmd_vel request
│      Server        │  At frequency: 30 Hz (typical)
└─────────┬──────────┘
          │
          │ calls every 33ms
          ▼
┌────────────────────────────────────────────────────┐
│  MPPIController::computeVelocityCommands()         │
│  Input: pose, velocity, goal                       │
└─────────┬──────────────────────────────────────────┘
          │
          │ 1. Get Data
          ▼
    ┌─────────────┐    ┌──────────────┐
    │ /odom       │    │ /global_plan │
    │ Robot state │    │ Path to goal │
    └──────┬──────┘    └──────┬───────┘
           │                  │
           └────────┬─────────┘
                    │
                    │ 2. Transform to local frame
                    ▼
          ┌──────────────────┐
          │  PathHandler     │
          │  • Prune path    │
          │  • Transform TF  │
          └────────┬─────────┘
                   │
                   │ 3. Optimize
                   ▼
┌────────────────────────────────────────────────────┐
│           Optimizer::evalControl()                 │
│  ╔════════════════════════════════════════════╗   │
│  ║  MPPI MAIN LOOP (typically 1 iteration)    ║   │
│  ╚════════════════════════════════════════════╝   │
└─────────┬──────────────────────────────────────────┘
          │
          │ ITERATION START
          │
          ▼
┌─────────────────────────────────────────────────────────┐
│  STEP 1: generateNoisedTrajectories()                   │
│  ┌────────────────────────────────────────────────┐    │
│  │  Previous optimal: u*(t) [time_steps × 1]       │    │
│  │  ┌──────┬──────┬──────┬──────┬─────┐           │    │
│  │  │ vx₀  │ vx₁  │ vx₂  │ vx₃  │ ... │           │    │
│  │  │ vy₀  │ vy₁  │ vy₂  │ vy₃  │ ... │           │    │
│  │  │ wz₀  │ wz₁  │ wz₂  │ wz₃  │ ... │           │    │
│  │  └──────┴──────┴──────┴──────┴─────┘           │    │
│  └────────────────────────────────────────────────┘    │
│           │                                             │
│           │ Add noise ε ~ N(0, Σ)                       │
│           ▼                                             │
│  ┌────────────────────────────────────────────────┐    │
│  │  K samples: U = [u₁, u₂, ..., uₖ]               │    │
│  │  Each: uᵢ = u* + εᵢ                              │    │
│  │                                                  │    │
│  │  Sample 1    Sample 2    ...    Sample K        │    │
│  │  ┌──────┐   ┌──────┐          ┌──────┐         │    │
│  │  │vx+ε₁ │   │vx+ε₂ │    ...   │vx+εₖ │         │    │
│  │  │vy+ε₁ │   │vy+ε₂ │    ...   │vy+εₖ │         │    │
│  │  │wz+ε₁ │   │wz+ε₂ │    ...   │wz+εₖ │         │    │
│  │  └──────┘   └──────┘          └──────┘         │    │
│  └────────────────────────────────────────────────┘    │
└─────────┬───────────────────────────────────────────────┘
          │
          │ PAPER: Sample u_i(t) = u*(t) + ε_i(t)
          ▼
┌─────────────────────────────────────────────────────────┐
│  STEP 2: MotionModel::predict()                         │
│  Forward simulate each sampled control sequence         │
│                                                         │
│  For k = 1 to batch_size:                              │
│    x₀ = current_pose                                   │
│    For t = 0 to time_steps:                           │
│      xₜ₊₁ = f(xₜ, uₖ(t))  ← Motion model             │
│                                                         │
│  ┌────────────────────────────────────────────────┐   │
│  │         Trajectory Samples                      │   │
│  │                                                 │   │
│  │   Sample 1:  x₁ → x₂ → x₃ → ... → x_T          │   │
│  │   Sample 2:  x₁ → x₂ → x₃ → ... → x_T          │   │
│  │   Sample 3:  x₁ → x₂ → x₃ → ... → x_T          │   │
│  │   ...                                           │   │
│  │   Sample K:  x₁ → x₂ → x₃ → ... → x_T          │   │
│  │                                                 │   │
│  │   Each xₜ contains: (x, y, θ, vx, vy, wz)      │   │
│  └────────────────────────────────────────────────┘   │
└─────────┬───────────────────────────────────────────────┘
          │
          │ PAPER: Integrate dx/dt = f(x,u)
          ▼
┌─────────────────────────────────────────────────────────┐
│  STEP 3: CriticManager::evalTrajectoriesScores()       │
│  Evaluate each trajectory through all critics           │
│                                                         │
│  For each critic c in [Goal, Obstacle, Path, ...]:     │
│    For each trajectory k:                              │
│      cost_k += critic_c.score(trajectory_k)            │
│                                                         │
│  ┌─────────────────────────────────────────────────┐  │
│  │   Trajectory 1: S₁ = 15.3                        │  │
│  │   Trajectory 2: S₂ = 22.7                        │  │
│  │   Trajectory 3: S₃ = 8.1   ← Best!              │  │
│  │   ...                                            │  │
│  │   Trajectory K: Sₖ = 45.2  ← Worst              │  │
│  └─────────────────────────────────────────────────┘  │
│                                                         │
│  Cost Breakdown Example (trajectory 1):                │
│  ┌──────────────────────────────┬──────────┐          │
│  │ Goal distance cost           │  5.2     │          │
│  │ Obstacle proximity cost      │  8.1     │          │
│  │ Path deviation cost          │  2.0     │          │
│  │ ────────────────────────────────────────│          │
│  │ Total: S₁                    │ 15.3     │          │
│  └──────────────────────────────┴──────────┘          │
└─────────┬───────────────────────────────────────────────┘
          │
          │ PAPER: Compute S_i = ∫ q(x,u)dt + φ(x_T)
          ▼
┌─────────────────────────────────────────────────────────┐
│  STEP 4: updateStateScores()                            │
│  Compute information-theoretic weights                  │
│                                                         │
│  For each trajectory k:                                │
│    w_k = exp(-S_k / λ)                                 │
│    η_k = w_k / Σⱼ w_j    (normalize)                   │
│                                                         │
│  ┌─────────────────────────────────────────────────┐  │
│  │ Costs:   [15.3, 22.7,  8.1, ..., 45.2]          │  │
│  │            ↓     ↓      ↓          ↓             │  │
│  │ exp(-S/λ) with λ=0.3:                            │  │
│  │            ↓     ↓      ↓          ↓             │  │
│  │ Weights: [0.02, 0.001, 0.15, ..., 0.0001]       │  │
│  │            ↓     ↓      ↓          ↓             │  │
│  │ Normalized: [0.11, 0.005, 0.78, ..., 0.0005]    │  │
│  │                         ↑                        │  │
│  │                    Best trajectory               │  │
│  │              gets highest weight!                │  │
│  └─────────────────────────────────────────────────┘  │
└─────────┬───────────────────────────────────────────────┘
          │
          │ PAPER: w_i = exp(-S_i/λ), η_i = w_i/Σw_j
          ▼
┌─────────────────────────────────────────────────────────┐
│  STEP 5: updateControlSequence()                        │
│  Compute weighted average of all controls               │
│                                                         │
│  For each timestep t:                                  │
│    u*(t) = Σₖ ηₖ · uₖ(t)                               │
│                                                         │
│  ┌─────────────────────────────────────────────────┐  │
│  │  Time t=0:                                       │  │
│  │  u*(0) = 0.78×u₃(0) + 0.11×u₁(0) + ...          │  │
│  │                                                  │  │
│  │  Time t=1:                                       │  │
│  │  u*(1) = 0.78×u₃(1) + 0.11×u₁(1) + ...          │  │
│  │  ...                                             │  │
│  │                                                  │  │
│  │  Result: New optimal control sequence            │  │
│  │  u* = [vx₀*, vx₁*, vx₂*, ..., vxₜ*]             │  │
│  │       [vy₀*, vy₁*, vy₂*, ..., vyₜ*]             │  │
│  │       [wz₀*, wz₁*, wz₂*, ..., wzₜ*]             │  │
│  └─────────────────────────────────────────────────┘  │
└─────────┬───────────────────────────────────────────────┘
          │
          │ PAPER: u* = Σ η_i u_i
          │
          │ Optional: Apply Savitzky-Golay smoothing
          ▼
┌─────────────────────────────────────────────────────────┐
│  STEP 6: Return u*(0) - MPC Principle                   │
│  Only use the first control!                           │
│                                                         │
│  ┌────────────────────────────────────────┐            │
│  │  Optimal sequence:                      │            │
│  │  [u*(0), u*(1), u*(2), ..., u*(T)]     │            │
│  │     ↓                                   │            │
│  │  Execute: u*(0)  ← Command to robot    │            │
│  │     ↓                                   │            │
│  │  geometry_msgs/TwistStamped:            │            │
│  │    linear.x  = vx*(0)                   │            │
│  │    linear.y  = vy*(0)                   │            │
│  │    angular.z = wz*(0)                   │            │
│  └────────────────────────────────────────┘            │
│                                                         │
│  For next cycle (t+1):                                 │
│  • Shift sequence: [u*(1), u*(2), ..., u*(T), 0]      │
│  • Use as new warm-start                               │
└─────────────────────────────────────────────────────────┘
          │
          │ Send to robot
          ▼
┌───────────────────┐
│  Robot executes   │  → Updates /odom
│  velocity command │     Cycle repeats at next timestep
└───────────────────┘

NEXT CYCLE: t = 1  (33ms later)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

## 3. Memory Layout and Data Structures

```
┌────────────────────────────────────────────────────────┐
│  models::State - Central Data Container               │
├────────────────────────────────────────────────────────┤
│                                                        │
│  SCALARS (Current robot state)                        │
│  ┌──────────────────────────────────────┐             │
│  │ pose: (x, y, θ)                      │             │
│  │ speed: (vx, vy, wz)                  │             │
│  └──────────────────────────────────────┘             │
│                                                        │
│  CONTROL MATRICES [time_steps × batch_size]           │
│  ┌─────────────────────────────────────────────────┐  │
│  │ cvx: Forward velocities                         │  │
│  │                                                 │  │
│  │        Sample 0  Sample 1  Sample 2  ... Sample K │
│  │  t=0  │  vx₀₀  │  vx₀₁  │  vx₀₂  │ ... │ vx₀ₖ │  │
│  │  t=1  │  vx₁₀  │  vx₁₁  │  vx₁₂  │ ... │ vx₁ₖ │  │
│  │  t=2  │  vx₂₀  │  vx₂₁  │  vx₂₂  │ ... │ vx₂ₖ │  │
│  │  ...  │   ...  │   ...  │   ...  │ ... │  ... │  │
│  │  t=T  │  vxₜ₀  │  vxₜ₁  │  vxₜ₂  │ ... │ vxₜₖ │  │
│  │                                                 │  │
│  │  Column 0: Optimal control from last iteration  │  │
│  │  Columns 1-K: Noised samples for this iteration │  │
│  └─────────────────────────────────────────────────┘  │
│                                                        │
│  ┌─────────────────────────────────────────────────┐  │
│  │ cvy: Lateral velocities (Omni only)             │  │
│  │      [time_steps × batch_size]                  │  │
│  └─────────────────────────────────────────────────┘  │
│                                                        │
│  ┌─────────────────────────────────────────────────┐  │
│  │ cwz: Angular velocities                         │  │
│  │      [time_steps × batch_size]                  │  │
│  └─────────────────────────────────────────────────┘  │
│                                                        │
│  TRAJECTORY MATRICES [time_steps × batch_size]        │
│  ┌─────────────────────────────────────────────────┐  │
│  │ trajectories.x: X positions                     │  │
│  │                                                 │  │
│  │        Sample 0  Sample 1  Sample 2  ... Sample K │
│  │  t=0  │   x₀₀  │   x₀₁  │   x₀₂  │ ... │  x₀ₖ │  │
│  │  t=1  │   x₁₀  │   x₁₁  │   x₁₂  │ ... │  x₁ₖ │  │
│  │  ...  │   ...  │   ...  │   ...  │ ... │  ... │  │
│  │  t=T  │   xₜ₀  │   xₜ₁  │   xₜ₂  │ ... │  xₜₖ │  │
│  └─────────────────────────────────────────────────┘  │
│                                                        │
│  ┌─────────────────────────────────────────────────┐  │
│  │ trajectories.y: Y positions                     │  │
│  │      [time_steps × batch_size]                  │  │
│  └─────────────────────────────────────────────────┘  │
│                                                        │
│  ┌─────────────────────────────────────────────────┐  │
│  │ trajectories.yaw: Orientations                  │  │
│  │      [time_steps × batch_size]                  │  │
│  └─────────────────────────────────────────────────┘  │
│                                                        │
│  COST VECTORS [batch_size]                            │
│  ┌─────────────────────────────────────────────────┐  │
│  │ costs: Total cost for each trajectory           │  │
│  │  [S₁, S₂, S₃, ..., Sₖ]                          │  │
│  └─────────────────────────────────────────────────┘  │
│                                                        │
│  ┌─────────────────────────────────────────────────┐  │
│  │ weights: Normalized weights for each trajectory │  │
│  │  [η₁, η₂, η₃, ..., ηₖ]                          │  │
│  └─────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────┘

Memory Access Pattern (for CPU optimization):
- Column-major: Access all samples for one timestep efficiently
- SIMD-friendly: Process 8 samples in parallel with AVX2
- Cache-friendly: Sequential access within columns
```

## 4. Critic Evaluation Pipeline

```
┌────────────────────────────────────────────────────────┐
│  CriticManager::evalTrajectoriesScores()               │
└────────────┬───────────────────────────────────────────┘
             │
             │ Initialize costs[K] = 0
             │
        ┌────┴────────────────────┐
        │ Loop over all critics   │
        └────┬────────────────────┘
             │
    ┌────────┴────────┐
    │  Critic Plugin  │
    │   Evaluation    │
    └────────┬────────┘
             │
        ╔════▼════════════════════════════════════════╗
        ║  Individual Critic Scoring Flow             ║
        ╚═════════════════════════════════════════════╝

Example: ObstaclesCritic
━━━━━━━━━━━━━━━━━━━━━━━━━
┌────────────────────────────────────────────┐
│  ObstaclesCritic::score()                  │
│                                            │
│  For each trajectory k = 1..batch_size:    │
│    For each pose in trajectory:            │
│                                            │
│      1. Get costmap value at (x, y)        │
│      ┌─────────────────────────────────┐  │
│      │  Costmap:                        │  │
│      │   254 = LETHAL (inside obstacle) │  │
│      │   253 = INSCRIBED (too close)    │  │
│      │   0-252 = Safe (scaled distance) │  │
│      │   255 = UNKNOWN                  │  │
│      └─────────────────────────────────┘  │
│                                            │
│      2. Compute cost based on value        │
│      if value >= INSCRIBED:                │
│        cost = HUGE_NUMBER  (reject!)       │
│      else:                                 │
│        cost = weight * (value/252)^power   │
│                                            │
│      3. Accumulate to trajectory cost      │
│      costs[k] += cost                      │
│                                            │
└────────────────────────────────────────────┘
             │
             │
    ┌────────▼────────┐
    │ Next Critic     │
    └────────┬────────┘
             │

Example: GoalCritic
━━━━━━━━━━━━━━━━━━━
┌────────────────────────────────────────────┐
│  GoalCritic::score()                       │
│                                            │
│  // Only evaluate if close to goal         │
│  if (dist_to_goal > threshold):            │
│    return  // Skip evaluation              │
│                                            │
│  For each trajectory k:                    │
│    // Get final pose (terminal state)      │
│    final_pose = trajectory[k].back()       │
│                                            │
│    // Euclidean distance to goal           │
│    dx = goal.x - final_pose.x              │
│    dy = goal.y - final_pose.y              │
│    dist = sqrt(dx² + dy²)                  │
│                                            │
│    // Add terminal cost                    │
│    costs[k] += weight * dist^power         │
│                                            │
└────────────────────────────────────────────┘
             │
             │
    ┌────────▼────────┐
    │ Next Critic     │
    └────────┬────────┘
             │

Example: PathAlignCritic
━━━━━━━━━━━━━━━━━━━━━━━
┌────────────────────────────────────────────┐
│  PathAlignCritic::score()                  │
│                                            │
│  For each trajectory k:                    │
│    // Find furthest point on path reached  │
│    furthest_idx = findFurthestReached(k)   │
│                                            │
│    // Skip if didn't reach far enough      │
│    if (furthest_idx < offset):             │
│      continue                              │
│                                            │
│    // Check alignment at multiple points   │
│    total_deviation = 0                     │
│    for each point along trajectory:        │
│      nearest_path_point = findNearest()    │
│      deviation = distance(point, path)     │
│      total_deviation += deviation          │
│                                            │
│    // Penalize deviation from path         │
│    costs[k] += weight * total_deviation^power │
│                                            │
└────────────────────────────────────────────┘
             │
             ▼
        ┌────────────────┐
        │ All critics    │
        │   complete     │
        └────────┬───────┘
                 │
                 ▼
    ┌────────────────────────────┐
    │ Final costs[K] ready       │
    │ for weighting step         │
    └────────────────────────────┘
```

## 5. Parameter Effect Visualization

```
Temperature Effect (λ)
━━━━━━━━━━━━━━━━━━━━━
Given costs: S = [10, 15, 8, 20, 12]

λ = 0.1 (Low temperature - Exploit)
────────────────────────────────────
Weights: exp(-S/0.1)
w = [4.5e-44, 3.1e-66, 3.0e-35, 5.5e-87, 6.1e-53]
Normalized: η ≈ [~1.0, 0, 0, 0, 0]
           ↑ Almost all weight on best!

Behavior: Very greedy, picks best trajectory
         Low exploration, fast convergence
         Risk: Can get stuck in local minima

λ = 1.0 (Medium temperature - Balanced)
────────────────────────────────────────
Weights: exp(-S/1.0)
w = [4.5e-5, 3.1e-7, 3.4e-4, 2.1e-9, 6.1e-6]
Normalized: η ≈ [0.13, 0.001, 0.85, 0, 0.016]
                              ↑ Best gets most
                  But others contribute

Behavior: Balanced exploration/exploitation
         Smooth, stable control
         Good for most applications

λ = 5.0 (High temperature - Explore)
────────────────────────────────────
Weights: exp(-S/5.0)
w = [0.135, 0.050, 0.202, 0.018, 0.090]
Normalized: η ≈ [0.27, 0.10, 0.41, 0.04, 0.18]
                 ↑     ↑     ↑     ↑     ↑
            All trajectories matter

Behavior: High exploration
         More noise in output
         Useful for difficult scenarios
         Risk: Slower, less efficient


Batch Size Effect (K)
━━━━━━━━━━━━━━━━━━━━
K = 100 (Small)
───────────────
Coverage:  ░░░░░░░░░░░░░░░░░░
Gaps:      ^^^^  ^^    ^^^
           Sparse sampling
           
Result: Fast computation
        May miss good solutions
        Less smooth control
        
K = 1000 (Medium)
─────────────────
Coverage:  ████░░██░████░░██
Gaps:      ^^  ^^^    ^^
           Good coverage
           
Result: Balanced performance
        Good solution quality
        50+ Hz possible
        
K = 2000 (Large)
────────────────
Coverage:  ██████████████████
Gaps:      (minimal)
           Dense sampling
           
Result: Best solution quality
        Smooth control
        Higher computation
        30-50 Hz typical


Noise Standard Deviation (σ)
━━━━━━━━━━━━━━━━━━━━━━━━━━━
vx_std = 0.1 (Small noise)
──────────────────────────
u* = 1.0 m/s
Samples:  
    |||||||   ← Tight cluster
    0.85  0.9  0.95  1.0  1.05  1.1  1.15
           u* samples

Result: Conservative
        Less exploration
        Small corrections
        
vx_std = 0.3 (Medium noise)
──────────────────────────
u* = 1.0 m/s
Samples:
  |  |  ||  |||  ||  |  |  ← Spread out
  0.4  0.6  0.8  1.0  1.2  1.4  1.6
              u* samples

Result: Balanced
        Good exploration
        Finds alternatives
        
vx_std = 0.5 (Large noise)
──────────────────────────
u* = 1.0 m/s
Samples:
| | | | | | | | | | | | |  ← Very spread
0.0  0.5  1.0  1.5  2.0  2.5
          u* samples

Result: Aggressive
        High exploration
        Can be unstable
```

## 6. Common Debugging Scenarios

```
Scenario 1: Robot Stops Unexpectedly
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────┐
│ Symptom: cmd_vel = 0    │
└──────────┬──────────────┘
           │
     Check ▼
┌──────────────────────────────────┐
│ All costs are very high?         │
│ → All trajectories collide       │
│ → Increase batch_size            │
│ → Reduce obstacle critic weight  │
└──────────┬───────────────────────┘
           │ No
     Check ▼
┌──────────────────────────────────┐
│ Path pruned away?                │
│ → prune_distance too small       │
│ → Path handler issue             │
└──────────┬───────────────────────┘
           │ No
     Check ▼
┌──────────────────────────────────┐
│ Velocity limits too restrictive? │
│ → vx_max/wz_max too small        │
│ → ax_max too small               │
└──────────────────────────────────┘


Scenario 2: Oscillating Behavior
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────┐
│ Symptom: Shaky motion   │
└──────────┬──────────────┘
           │
     Check ▼
┌──────────────────────────────────┐
│ Temperature too high?            │
│ → Reduce λ for more exploitation │
└──────────┬───────────────────────┘
           │ No
     Check ▼
┌──────────────────────────────────┐
│ Conflicting critics?             │
│ → PathAlign vs PathFollow        │
│ → Balance weights                │
└──────────┬───────────────────────┘
           │ No
     Check ▼
┌──────────────────────────────────┐
│ Batch size too small?            │
│ → Increase for smoother output   │
└──────────┬───────────────────────┘
           │ No
     Check ▼
┌──────────────────────────────────┐
│ No smoothing enabled?            │
│ → Use Savitzky-Golay filter      │
└──────────────────────────────────┘


Scenario 3: Not Following Path
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────┐
│ Symptom: Cuts corners   │
└──────────┬──────────────┘
           │
     Check ▼
┌──────────────────────────────────┐
│ Path critics weighted too low?   │
│ → Increase PathAlign weight      │
│ → Increase PathFollow weight     │
└──────────┬───────────────────────┘
           │ No
     Check ▼
┌──────────────────────────────────┐
│ Goal critic too aggressive?      │
│ → Reduce goal_cost_weight        │
│ → Increase threshold_to_consider │
└──────────┬───────────────────────┘
           │ No
     Check ▼
┌──────────────────────────────────┐
│ Prediction horizon too short?    │
│ → Increase time_steps            │
│ → Check prune_distance           │
└──────────────────────────────────┘
```

## 7. Code Reading Guide

**Start Here - Main Entry Points:**
```
1. src/controller.cpp
   └─ computeVelocityCommands()  [Line ~180]
       ↓
   └─ optimizer_->evalControl()

2. src/optimizer.cpp
   └─ evalControl()  [Line ~90]
       ├─ generateNoisedTrajectories()  [Line ~200]
       ├─ critics_->evalTrajectoriesScores()  [Line ~220]
       ├─ updateStateScores()  [Line ~250]
       └─ updateControlSequence()  [Line ~280]
```

**Deep Dive Paths:**

Path 1: Understand Sampling
```
src/optimizer.cpp::generateNoisedTrajectories()
  ↓
src/noise_generator.cpp::generateNextNoises()
  ↓
src/motion_models.cpp::predict()
  ↓
Check: How are controls perturbed?
Check: How is forward simulation done?
```

Path 2: Understand Cost Evaluation
```
src/critic_manager.cpp::evalTrajectoriesScores()
  ↓
src/critics/goal_critic.cpp::score()
  ↓
src/critics/obstacles_critic.cpp::score()
  ↓
Check: How costs accumulate?
Check: What makes a trajectory good/bad?
```

Path 3: Understand Weight Computation
```
src/optimizer.cpp::updateStateScores()
  ↓
Look for: exp(-cost/temperature)
Look for: Normalization step
  ↓
Check: How is temperature used?
Check: What happens to outlier costs?
```

**Header Files to Study:**
```
1. models/state.hpp
   → Central data structure
   → Understand matrix layouts

2. critic.hpp
   → Base interface
   → How critics plug in

3. motion_models.hpp
   → Robot dynamics interface
   → Different model types
```

This visualization guide complements the main theory-to-code document. Use it as a reference while studying the codebase!
