// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef NAV2_SMAC_PLANNER_CUSTOM__SMAC_PLANNER_HYBRID_HPP_
#define NAV2_SMAC_PLANNER_CUSTOM__SMAC_PLANNER_HYBRID_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <array>
#include <vector>
#include <string>

#include "nav2_smac_planner_custom/a_star.hpp"
#include "nav2_smac_planner_custom/smoother.hpp"
#include "nav2_smac_planner_custom/utils.hpp"
#include "nav2_smac_planner_custom/costmap_downsampler.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

namespace nav2_smac_planner_custom
{

class SmacPlannerHybrid : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  SmacPlannerHybrid();

  /**
   * @brief destructor
   */
  ~SmacPlannerHybrid();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @param cancel_checker Function to check if the action has been canceled
   * @return nav2_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker)  override;

protected:
  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Re-derive grid-cell parameters from the current costmap resolution
   * and rebuild the requested components. Caller must hold _mutex.
   */
  void reinitialize(
    bool reinit_collision_checker, bool reinit_a_star,
    bool reinit_downsampler, bool reinit_smoother);

  /**
   * @brief Rover's current gear from mocap pose deltas: signed displacement
   * along the current heading over the last motion_window seconds.
   * @param displacement output, meters (NaN when unknown)
   * @return +1 forward, -1 reverse, 0 at rest / unknown / stale
   */
  int currentGear(double & displacement);

  /**
   * @brief Advance the cusp-tail measurement (TailTracker) with a pose
   * sample. Caller holds _motion_mutex.
   */
  void updateTailTracker(double now, double x, double y, double yaw);

  /**
   * @brief Constant-curvature arc mode (fork-only): the single circular arc
   * that starts at the start pose, tangent to its heading (forward if the
   * goal is ahead, reverse if behind), and ends at the goal position.
   * @param plan output, filled only on success (world frame)
   * @param reason output, why the arc was rejected
   * @return true if the arc is feasible (radius, length, cost, gear)
   */
  bool tryArcPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav2_costmap_2d::Costmap2D * costmap, int moving_gear,
    nav_msgs::msg::Path & plan, std::string & reason);

  /**
   * @brief Arc hold: is the rover still on the last arc planned for this goal
   * (same goal, gear and turn direction, planned recently, start pose within
   * arc_hold_max_offset / arc_hold_max_heading_error of it)?
   */
  bool arcCommitted(
    double x, double y, double yaw, double gx, double gy, int gear, int turn,
    double now) const;

  std::unique_ptr<AStarAlgorithm<NodeHybrid>> _a_star;
  GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Logger _logger{rclcpp::get_logger("SmacPlannerHybrid")};
  nav2_costmap_2d::Costmap2D * _costmap;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros;
  std::unique_ptr<CostmapDownsampler> _costmap_downsampler;
  std::string _global_frame, _name;
  float _lookup_table_dim;
  float _tolerance;
  bool _downsample_costmap;
  int _downsampling_factor;
  double _angle_bin_size;
  unsigned int _angle_quantizations;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  int _terminal_checking_interval;
  SearchInfo _search_info;
  double _max_planning_time;
  double _lookup_table_size;
  double _minimum_turning_radius_global_coords;
  // Meter-valued params kept so grid-cell conversions can be redone if the
  // costmap resolution changes after configure() (e.g. a static map resizes
  // the costmap to its own resolution); _search_resolution is the resolution
  // the current conversions in _search_info were made at.
  double _momentum_zone_length_m{0.0};
  double _momentum_zone_min_radius_m{0.0};
  double _cusp_tail_length_m{0.0};
  double _analytic_expansion_max_length_m{3.0};
  double _motion_reversal_penalty_m{0.0};

  // Constant-curvature arc mode (fork-only, see tryArcPlan()). Negative
  // _arc_min_radius / _arc_max_cost mean "use minimum_turning_radius /
  // analytic_expansion_max_cost".
  bool _arc_mode_enabled{false};
  double _arc_min_radius{-1.0};
  double _arc_max_length{10.0};
  double _arc_max_cost{-1.0};
  double _arc_path_resolution{0.05};
  double _arc_max_sweep{180.0};  // degrees of heading change; >= 180 = no cap

  // Arc hold (fork-only, see arcCommitted()): once the rover is driving an
  // arc, re-fitted arcs for the same goal get looser limits, so a little
  // drift does not throw it into Hybrid-A* (and a three-point turn).
  // _arc_hold_min_radius <= 0 disables the hold (fresh limits always apply).
  double _arc_hold_min_radius{-1.0};
  double _arc_hold_max_sweep{-1.0};         // degrees; <0 = arc_max_sweep
  double _arc_hold_max_offset{0.35};        // m off the committed arc
  double _arc_hold_max_heading_error{25.0};  // degrees off the committed arc
  double _arc_hold_end_distance{0.0};       // m; closer to the goal: no radius limit
  double _arc_hold_timeout{3.0};            // s since the committed arc was planned
  struct ArcCommit
  {
    bool valid{false};
    double t{0.0}, gx{0.0}, gy{0.0};
    int gear{0}, turn{0};  // turn: sign of the curvature, 0 = straight
    std::vector<std::array<double, 3>> poses;  // x, y, heading
  };
  ArcCommit _arc_commit;

  // Direction-aware replanning (fork-only): pose samples from
  // motion_pose_topic, stamped with receive time, used by currentGear() to
  // seed the search start with the gear the rover is already moving in.
  struct MotionSample
  {
    double t, x, y, yaw;
  };
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _motion_pose_sub;
  std::deque<MotionSample> _motion_samples;
  std::mutex _motion_mutex;  // guards _motion_samples and the three params below
  double _motion_window{0.5};
  double _motion_threshold{0.02};
  double _motion_stale_timeout{0.5};
  // Cusp tails (see SearchInfo::cusp_tail_length): what the rover has already
  // driven in its current gear, measured from the same pose samples in 0.1 s
  // steps -- total distance since the last gear change, and the "straight
  // run" since the last gear change or tight turn. Seeds the start node so a
  // tail the rover is in the middle of is not demanded again on every replan.
  // Guarded by _motion_mutex.
  struct TailTracker
  {
    bool init{false};
    double t{0.0}, x{0.0}, y{0.0}, yaw{0.0};
    int gear{0};
    double gear_dist{0.0}, run{0.0};
  };
  TailTracker _tail;
  double _search_resolution{0.0};
  bool _debug_visualizations;
  std::string _motion_model_for_search;
  MotionModel _motion_model;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr _raw_plan_publisher;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    _planned_footprints_publisher;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    _smoothed_footprints_publisher;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr
    _expansions_publisher;
  std::mutex _mutex;
  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
  std::shared_ptr<rclcpp::ParameterEventHandler> _remote_param_subscriber;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> _remote_resolution_handler;
};

}  // namespace nav2_smac_planner_custom

#endif  // NAV2_SMAC_PLANNER_CUSTOM__SMAC_PLANNER_HYBRID_HPP_
