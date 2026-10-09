// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2023, Open Navigation LLC
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

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "Eigen/Core"
#include "angles/angles.h"
#include "nav2_smac_planner_custom/smac_planner_hybrid.hpp"

// #define BENCHMARK_TESTING

namespace nav2_smac_planner_custom
{

using namespace std::chrono;  // NOLINT
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

SmacPlannerHybrid::SmacPlannerHybrid()
: _a_star(nullptr),
  _collision_checker(nullptr, 1, nullptr),
  _smoother(nullptr),
  _costmap(nullptr),
  _costmap_ros(nullptr),
  _costmap_downsampler(nullptr)
{
}

SmacPlannerHybrid::~SmacPlannerHybrid()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type SmacPlannerHybrid",
    _name.c_str());
}

void SmacPlannerHybrid::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  _node = parent;
  auto node = parent.lock();
  _logger = node->get_logger();
  _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _costmap_ros = costmap_ros;
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(_logger, "Configuring %s of type SmacPlannerHybrid", name.c_str());

  int angle_quantizations;
  double analytic_expansion_max_length_m;
  bool smooth_path;

  // General planner params
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsample_costmap", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".downsample_costmap", _downsample_costmap);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
  node->get_parameter(name + ".downsampling_factor", _downsampling_factor);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".angle_quantization_bins", rclcpp::ParameterValue(72));
  node->get_parameter(name + ".angle_quantization_bins", angle_quantizations);
  _angle_bin_size = 2.0 * M_PI / angle_quantizations;
  _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(0.25));
  _tolerance = static_cast<float>(node->get_parameter(name + ".tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", _allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_iterations", rclcpp::ParameterValue(1000000));
  node->get_parameter(name + ".max_iterations", _max_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_on_approach_iterations", rclcpp::ParameterValue(1000));
  node->get_parameter(name + ".max_on_approach_iterations", _max_on_approach_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_checking_interval", rclcpp::ParameterValue(5000));
  node->get_parameter(name + ".terminal_checking_interval", _terminal_checking_interval);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_path", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".smooth_path", smooth_path);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.4));
  node->get_parameter(name + ".minimum_turning_radius", _minimum_turning_radius_global_coords);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_primitive_interpolation", rclcpp::ParameterValue(false));
  node->get_parameter(
    name + ".allow_primitive_interpolation", _search_info.allow_primitive_interpolation);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cache_obstacle_heuristic", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".cache_obstacle_heuristic", _search_info.cache_obstacle_heuristic);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".reverse_penalty", _search_info.reverse_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".change_penalty", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".change_penalty", _search_info.change_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".momentum_zone_length", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".momentum_zone_length", _search_info.momentum_zone_length);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".momentum_zone_penalty", rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".momentum_zone_penalty", _search_info.momentum_zone_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".momentum_zone_min_radius", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".momentum_zone_min_radius", _momentum_zone_min_radius_m);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cusp_tail_length", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".cusp_tail_length", _cusp_tail_length_m);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".curvature_penalty", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".curvature_penalty", _search_info.curvature_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".extra_direction_change_penalty", rclcpp::ParameterValue(1.0));
  node->get_parameter(
    name + ".extra_direction_change_penalty", _search_info.extra_direction_change_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".escalate_only_on_reset", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".escalate_only_on_reset", _search_info.escalate_only_on_reset);

  // Constant-curvature arc mode (fork-only, off by default): plan the single
  // arc tangent to the start heading through the goal when it is feasible,
  // Hybrid-A* otherwise -- see tryArcPlan().
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".arc_mode_enabled", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".arc_mode_enabled", _arc_mode_enabled);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".arc_min_radius", rclcpp::ParameterValue(-1.0));
  node->get_parameter(name + ".arc_min_radius", _arc_min_radius);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".arc_max_length", rclcpp::ParameterValue(10.0));
  node->get_parameter(name + ".arc_max_length", _arc_max_length);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".arc_max_cost", rclcpp::ParameterValue(-1.0));
  node->get_parameter(name + ".arc_max_cost", _arc_max_cost);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".arc_path_resolution", rclcpp::ParameterValue(0.05));
  node->get_parameter(name + ".arc_path_resolution", _arc_path_resolution);
  if (_arc_path_resolution <= 0.0) {
    _arc_path_resolution = 0.05;
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".arc_max_sweep", rclcpp::ParameterValue(180.0));
  node->get_parameter(name + ".arc_max_sweep", _arc_max_sweep);
  // Arc hold (fork-only, off unless arc_hold_min_radius > 0): see arcCommitted()
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".arc_hold_min_radius", rclcpp::ParameterValue(-1.0));
  node->get_parameter(name + ".arc_hold_min_radius", _arc_hold_min_radius);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".arc_hold_max_sweep", rclcpp::ParameterValue(-1.0));
  node->get_parameter(name + ".arc_hold_max_sweep", _arc_hold_max_sweep);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".arc_hold_max_offset", rclcpp::ParameterValue(0.35));
  node->get_parameter(name + ".arc_hold_max_offset", _arc_hold_max_offset);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".arc_hold_max_heading_error", rclcpp::ParameterValue(25.0));
  node->get_parameter(name + ".arc_hold_max_heading_error", _arc_hold_max_heading_error);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".arc_hold_end_distance", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".arc_hold_end_distance", _arc_hold_end_distance);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".arc_hold_timeout", rclcpp::ParameterValue(3.0));
  node->get_parameter(name + ".arc_hold_timeout", _arc_hold_timeout);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".ignore_goal_heading", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".ignore_goal_heading", _search_info.ignore_goal_heading);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".motion_reversal_penalty", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".motion_reversal_penalty", _motion_reversal_penalty_m);

  // Direction-aware replanning (fork-only, off unless motion_pose_topic is
  // set): seed each search's start with the gear the rover is already
  // moving in, measured from mocap pose deltas -- see currentGear().
  std::string motion_pose_topic;
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".motion_pose_topic", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name + ".motion_pose_topic", motion_pose_topic);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".motion_window", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".motion_window", _motion_window);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".motion_threshold", rclcpp::ParameterValue(0.02));
  node->get_parameter(name + ".motion_threshold", _motion_threshold);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".motion_stale_timeout", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".motion_stale_timeout", _motion_stale_timeout);
  if (!motion_pose_topic.empty()) {
    _motion_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      motion_pose_topic, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        // receive time, not header stamp: the mocap clock may not be synced
        const double now = _clock->now().seconds();
        std::lock_guard<std::mutex> lock(_motion_mutex);
        _motion_samples.push_back(
          {now, msg->pose.position.x, msg->pose.position.y, tf2::getYaw(msg->pose.orientation)});
        // keep one sample at or beyond the window edge so the delta spans it
        while (_motion_samples.size() > 2 && now - _motion_samples[1].t >= _motion_window) {
          _motion_samples.pop_front();
        }
        updateTailTracker(
          now, msg->pose.position.x, msg->pose.position.y, tf2::getYaw(msg->pose.orientation));
      });
    RCLCPP_INFO(
      _logger, "%s: direction-aware replanning from %s", name.c_str(), motion_pose_topic.c_str());
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.2));
  node->get_parameter(name + ".non_straight_penalty", _search_info.non_straight_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cost_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".cost_penalty", _search_info.cost_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".retrospective_penalty", rclcpp::ParameterValue(0.015));
  node->get_parameter(name + ".retrospective_penalty", _search_info.retrospective_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(3.5));
  node->get_parameter(name + ".analytic_expansion_ratio", _search_info.analytic_expansion_ratio);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_max_cost", rclcpp::ParameterValue(200.0));
  node->get_parameter(
    name + ".analytic_expansion_max_cost", _search_info.analytic_expansion_max_cost);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_max_cost_override", rclcpp::ParameterValue(false));
  node->get_parameter(
    name + ".analytic_expansion_max_cost_override",
    _search_info.analytic_expansion_max_cost_override);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".use_quadratic_cost_penalty", rclcpp::ParameterValue(false));
  node->get_parameter(
    name + ".use_quadratic_cost_penalty", _search_info.use_quadratic_cost_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsample_obstacle_heuristic", rclcpp::ParameterValue(true));
  node->get_parameter(
    name + ".downsample_obstacle_heuristic", _search_info.downsample_obstacle_heuristic);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_max_length", rclcpp::ParameterValue(3.0));
  node->get_parameter(name + ".analytic_expansion_max_length", analytic_expansion_max_length_m);
  _analytic_expansion_max_length_m = analytic_expansion_max_length_m;
  _search_info.analytic_expansion_max_length =
    analytic_expansion_max_length_m / _costmap->getResolution();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_planning_time", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".max_planning_time", _max_planning_time);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lookup_table_size", rclcpp::ParameterValue(20.0));
  node->get_parameter(name + ".lookup_table_size", _lookup_table_size);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".debug_visualizations", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".debug_visualizations", _debug_visualizations);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".motion_model_for_search", rclcpp::ParameterValue(std::string("DUBIN")));
  node->get_parameter(name + ".motion_model_for_search", _motion_model_for_search);
  _motion_model = fromString(_motion_model_for_search);
  if (_motion_model == MotionModel::UNKNOWN) {
    RCLCPP_WARN(
      _logger,
      "Unable to get MotionModel search type. Given '%s', "
      "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.",
      _motion_model_for_search.c_str());
  }

  if (_max_on_approach_iterations <= 0) {
    RCLCPP_WARN(
      _logger, "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    _max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (_max_iterations <= 0) {
    RCLCPP_WARN(
      _logger, "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    _max_iterations = std::numeric_limits<int>::max();
  }

  if (_minimum_turning_radius_global_coords < _costmap->getResolution() * _downsampling_factor) {
    RCLCPP_WARN(
      _logger, "Min turning radius cannot be less than the search grid cell resolution!");
    _minimum_turning_radius_global_coords = _costmap->getResolution() * _downsampling_factor;
  }

  // convert to grid coordinates
  if (!_downsample_costmap) {
    _downsampling_factor = 1;
  }
  _search_info.minimum_turning_radius =
    _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
  // momentum_zone_length is configured in meters but compared against
  // NodeHybrid's distance-since-reset, which accumulates in grid cells
  _momentum_zone_length_m = _search_info.momentum_zone_length;
  _search_info.momentum_zone_length =
    _momentum_zone_length_m / (_costmap->getResolution() * _downsampling_factor);
  _search_info.momentum_zone_min_radius = static_cast<float>(
    _momentum_zone_min_radius_m / (_costmap->getResolution() * _downsampling_factor));
  _search_info.cusp_tail_length = static_cast<float>(
    _cusp_tail_length_m / (_costmap->getResolution() * _downsampling_factor));
  _search_info.motion_reversal_penalty =
    _motion_reversal_penalty_m / (_costmap->getResolution() * _downsampling_factor);
  _search_resolution = _costmap->getResolution();
  RCLCPP_INFO(
    _logger, "%s: costmap resolution %.3f m, minimum_turning_radius %.2f cells, "
    "momentum_zone_length %.2f cells", _name.c_str(), _costmap->getResolution(),
    _search_info.minimum_turning_radius, _search_info.momentum_zone_length);
  _lookup_table_dim =
    static_cast<float>(_lookup_table_size) /
    static_cast<float>(_costmap->getResolution() * _downsampling_factor);

  // Make sure its a whole number
  _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

  // Make sure its an odd number
  if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
    RCLCPP_INFO(
      _logger,
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      _lookup_table_dim);
    _lookup_table_dim += 1.0;
  }

  // Initialize collision checker
  _collision_checker = GridCollisionChecker(_costmap_ros, _angle_quantizations, node);
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),
    _costmap_ros->getUseRadius(),
    findCircumscribedCost(_costmap_ros));

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
  _a_star->initialize(
    _allow_unknown,
    _max_iterations,
    _max_on_approach_iterations,
    _terminal_checking_interval,
    _max_planning_time,
    _lookup_table_dim,
    _angle_quantizations);

  // Initialize path smoother
  SmootherParams params;
  params.get(node, name);
  if (smooth_path) {
    _smoother = std::make_unique<Smoother>(params);
    _smoother->initialize(_minimum_turning_radius_global_coords);
  }

  // Initialize costmap downsampler
  if (_downsample_costmap && _downsampling_factor > 1) {
    _costmap_downsampler = std::make_unique<CostmapDownsampler>();
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler->on_configure(
      node, _global_frame, topic_name, _costmap, _downsampling_factor);
  }

  _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

  if (_debug_visualizations) {
    _expansions_publisher = node->create_publisher<geometry_msgs::msg::PoseArray>("expansions", 1);
    _planned_footprints_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "planned_footprints", 1);
    _smoothed_footprints_publisher =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "smoothed_footprints", 1);
  }

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type SmacPlannerHybrid with "
    "maximum iterations %i, max on approach iterations %i, and %s. Tolerance %.2f."
    "Using motion model: %s.",
    _name.c_str(), _max_iterations, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _tolerance, toString(_motion_model).c_str());
}

void SmacPlannerHybrid::activate()
{
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  if (_debug_visualizations) {
    _expansions_publisher->on_activate();
    _planned_footprints_publisher->on_activate();
    _smoothed_footprints_publisher->on_activate();
  }
  if (_costmap_downsampler) {
    _costmap_downsampler->on_activate();
  }
  auto node = _node.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(
    std::bind(&SmacPlannerHybrid::dynamicParametersCallback, this, _1));

  // Special case handling to obtain resolution changes in global costmap
  auto resolution_remote_cb = [this](const rclcpp::Parameter & p) {
      auto node = _node.lock();
      dynamicParametersCallback(
        {rclcpp::Parameter("resolution", rclcpp::ParameterValue(p.as_double()))});
    };
  _remote_param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(_node.lock());
  _remote_resolution_handler = _remote_param_subscriber->add_parameter_callback(
    "resolution", resolution_remote_cb, "global_costmap/global_costmap");
}

void SmacPlannerHybrid::deactivate()
{
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
  if (_debug_visualizations) {
    _expansions_publisher->on_deactivate();
    _planned_footprints_publisher->on_deactivate();
    _smoothed_footprints_publisher->on_deactivate();
  }
  if (_costmap_downsampler) {
    _costmap_downsampler->on_deactivate();
  }
  // shutdown dyn_param_handler
  auto node = _node.lock();
  if (_dyn_params_handler && node) {
    node->remove_on_set_parameters_callback(_dyn_params_handler.get());
  }
  _dyn_params_handler.reset();
}

void SmacPlannerHybrid::cleanup()
{
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  nav2_smac_planner_custom::NodeHybrid::destroyStaticAssets();
  _a_star.reset();
  _smoother.reset();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_cleanup();
    _costmap_downsampler.reset();
  }
  _raw_plan_publisher.reset();
  if (_debug_visualizations) {
    _expansions_publisher.reset();
    _planned_footprints_publisher.reset();
    _smoothed_footprints_publisher.reset();
  }
}

nav_msgs::msg::Path SmacPlannerHybrid::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  std::lock_guard<std::mutex> lock_reinit(_mutex);
  steady_clock::time_point a = steady_clock::now();

  // Grid-cell params (turning radius, momentum zone, analytic expansion
  // length, lookup table) are converted with the resolution the costmap had
  // when they were computed. A static layer resizes the costmap to the map's
  // own resolution after configure(), without changing the "resolution"
  // parameter the remote handler watches -- so re-derive them here.
  if (_costmap->getResolution() != _search_resolution) {
    RCLCPP_INFO(
      _logger, "%s: costmap resolution changed %.3f -> %.3f m since grid parameters "
      "were derived, reinitializing.", _name.c_str(), _search_resolution,
      _costmap->getResolution());
    reinitialize(true, true, true, _smoother != nullptr);
  }

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
    _collision_checker.setCostmap(costmap);
  }

  // Set collision checker and costmap information
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),
    _costmap_ros->getUseRadius(),
    findCircumscribedCost(_costmap_ros));
  _a_star->setCollisionChecker(&_collision_checker);

  // Set starting point, in A* bin search coordinates
  float mx_start, my_start, mx_goal, my_goal;
  if (!costmap->worldToMapContinuous(
    start.pose.position.x,
    start.pose.position.y,
    mx_start,
    my_start))
  {
    throw nav2_core::StartOutsideMapBounds(
            "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  double orientation_bin = std::round(tf2::getYaw(start.pose.orientation) / _angle_bin_size);
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  _a_star->setStart(mx_start, my_start, static_cast<unsigned int>(orientation_bin));

  // Direction-aware replanning: if the rover is already moving, start the
  // search in that gear with momentum, so a replan reversing it counts as a
  // real direction change (isMomentumReset(), motion_reversal_penalty, the
  // analytic-expansion junction check) instead of being free. At rest the
  // start keeps the "no previous primitive" sentinel, as before.
  int moving_gear = 0;
  if (_motion_pose_sub) {
    double displacement = 0.0;
    const int gear = currentGear(displacement);
    moving_gear = gear;
    NodeHybrid * start_node = _a_star->getStart();
    const TurnDirection wanted = gear > 0 ? TurnDirection::FORWARD : TurnDirection::REVERSE;
    unsigned int seed_index = std::numeric_limits<unsigned int>::max();
    if (gear != 0) {
      const auto & projections = NodeHybrid::motion_table.projections;
      for (unsigned int i = 0; i < projections.size(); ++i) {
        if (projections[i]._turn_dir == wanted) {
          seed_index = i;
          break;
        }
      }
    }
    // cusp tails: what the rover already drove in this gear (see TailTracker)
    double tail_run_m = 0.0, tail_gear_m = -1.0;
    if (seed_index != std::numeric_limits<unsigned int>::max()) {
      std::lock_guard<std::mutex> lock(_motion_mutex);
      if (_tail.gear == gear) {
        tail_run_m = _tail.run;
        tail_gear_m = _tail.gear_dist;
      }
    }
    const double cell_m = _costmap->getResolution() * _downsampling_factor;
    if (seed_index != std::numeric_limits<unsigned int>::max()) {
      start_node->setMotionPrimitiveIndex(seed_index, wanted);
      start_node->setDistanceSinceMomentumReset(NodeHybrid::motion_table.momentum_zone_length);
      start_node->setStraightRun(static_cast<float>(tail_run_m / cell_m));
      start_node->setDistanceSinceGearFlip(
        tail_gear_m >= 0.0 ? static_cast<float>(tail_gear_m / cell_m) : NodeHybrid::kNoGearFlip);
    } else {
      start_node->setMotionPrimitiveIndex(
        std::numeric_limits<unsigned int>::max(), TurnDirection::UNKNOWN);
      start_node->setDistanceSinceMomentumReset(0.0f);
      start_node->setStraightRun(0.0f);
      start_node->setDistanceSinceGearFlip(NodeHybrid::kNoGearFlip);
    }
    start_node->setDirectionChangeCount(0);
    std::string tail_msg;
    if (_cusp_tail_length_m > 0.0) {
      tail_msg = ", driven in gear " + std::to_string(std::max(0.0, tail_gear_m)) +
        " m, straight run " + std::to_string(tail_run_m) + " m";
    }
    RCLCPP_INFO(
      _logger, "%s: start gear: %s (d=%.3f m)%s", _name.c_str(),
      seed_index == std::numeric_limits<unsigned int>::max() ? "rest" :
      (gear > 0 ? "forward" : "reverse"), displacement, tail_msg.c_str());
  }

  // Set goal point, in A* bin search coordinates
  if (!costmap->worldToMapContinuous(
    goal.pose.position.x,
    goal.pose.position.y,
    mx_goal,
    my_goal))
  {
    throw nav2_core::GoalOutsideMapBounds(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was outside bounds");
  }
  orientation_bin = std::round(tf2::getYaw(goal.pose.orientation) / _angle_bin_size);
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  _a_star->setGoal(mx_goal, my_goal, static_cast<unsigned int>(orientation_bin));

  // Setup message
  nav_msgs::msg::Path plan;
  plan.header.stamp = _clock->now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Corner case of start and goal being on the same cell
  if (std::floor(mx_start) == std::floor(mx_goal) &&
    std::floor(my_start) == std::floor(my_goal))
  {
    pose.pose = start.pose;
    pose.pose.orientation = goal.pose.orientation;
    plan.poses.push_back(pose);

    // Publish raw path for debug
    if (_raw_plan_publisher->get_subscription_count() > 0) {
      _raw_plan_publisher->publish(plan);
    }

    return plan;
  }

  // Constant-curvature arc mode: one arc for the whole path when feasible
  if (_arc_mode_enabled) {
    std::string reason;
    if (tryArcPlan(start, goal, costmap, moving_gear, plan, reason)) {
      if (_raw_plan_publisher->get_subscription_count() > 0) {
        _raw_plan_publisher->publish(plan);
      }
      return plan;
    }
    _arc_commit.valid = false;  // off the arc: a new one must meet the fresh limits
    RCLCPP_INFO(_logger, "%s: arc rejected (%s), using Hybrid-A*", _name.c_str(), reason.c_str());
  }

  // Compute plan
  NodeHybrid::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  std::unique_ptr<std::vector<std::tuple<float, float, float>>> expansions = nullptr;
  if (_debug_visualizations) {
    expansions = std::make_unique<std::vector<std::tuple<float, float, float>>>();
  }
  // Note: All exceptions thrown are handled by the planner server and returned to the action
  if (!_a_star->createPath(
      path, num_iterations,
      _tolerance / static_cast<float>(costmap->getResolution()), cancel_checker, expansions.get()))
  {
    if (_debug_visualizations) {
      geometry_msgs::msg::PoseArray msg;
      geometry_msgs::msg::Pose msg_pose;
      msg.header.stamp = _clock->now();
      msg.header.frame_id = _global_frame;
      for (auto & e : *expansions) {
        msg_pose.position.x = std::get<0>(e);
        msg_pose.position.y = std::get<1>(e);
        msg_pose.orientation = getWorldOrientation(std::get<2>(e));
        msg.poses.push_back(msg_pose);
      }
      _expansions_publisher->publish(msg);
    }

    // Note: If the start is blocked only one iteration will occur before failure
    if (num_iterations == 1) {
      throw nav2_core::StartOccupied("Start occupied");
    }

    if (num_iterations < _a_star->getMaxIterations()) {
      throw nav2_core::NoValidPathCouldBeFound("no valid path found");
    } else {
      throw nav2_core::PlannerTimedOut("exceeded maximum iterations");
    }
  }

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
    pose.pose.orientation = getWorldOrientation(path[i].theta);
    plan.poses.push_back(pose);
  }

  // Publish raw path for debug
  if (_raw_plan_publisher->get_subscription_count() > 0) {
    _raw_plan_publisher->publish(plan);
  }

  if (_debug_visualizations) {
    // Publish expansions for debug
    auto now = _clock->now();
    geometry_msgs::msg::PoseArray msg;
    geometry_msgs::msg::Pose msg_pose;
    msg.header.stamp = now;
    msg.header.frame_id = _global_frame;
    for (auto & e : *expansions) {
      msg_pose.position.x = std::get<0>(e);
      msg_pose.position.y = std::get<1>(e);
      msg_pose.orientation = getWorldOrientation(std::get<2>(e));
      msg.poses.push_back(msg_pose);
    }
    _expansions_publisher->publish(msg);

    if (_planned_footprints_publisher->get_subscription_count() > 0) {
      // Clear all markers first
      auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      visualization_msgs::msg::Marker clear_all_marker;
      clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array->markers.push_back(clear_all_marker);
      _planned_footprints_publisher->publish(std::move(marker_array));

      // Publish smoothed footprints for debug
      marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      for (size_t i = 0; i < plan.poses.size(); i++) {
        const std::vector<geometry_msgs::msg::Point> edge =
          transformFootprintToEdges(plan.poses[i].pose, _costmap_ros->getRobotFootprint());
        marker_array->markers.push_back(createMarker(edge, i, _global_frame, now));
      }
      _planned_footprints_publisher->publish(std::move(marker_array));
    }
  }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());

#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  // Smooth plan
  if (_smoother && num_iterations > 1) {
    _smoother->smooth(plan, costmap, time_remaining);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point c = steady_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(c - b);
  std::cout << "It took " << time_span2.count() * 1000 <<
    " milliseconds to smooth path." << std::endl;
#endif

  if (_debug_visualizations) {
    if (_smoothed_footprints_publisher->get_subscription_count() > 0) {
      // Clear all markers first
      auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      visualization_msgs::msg::Marker clear_all_marker;
      clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array->markers.push_back(clear_all_marker);
      _smoothed_footprints_publisher->publish(std::move(marker_array));

      // Publish smoothed footprints for debug
      marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      auto now = _clock->now();
      for (size_t i = 0; i < plan.poses.size(); i++) {
        const std::vector<geometry_msgs::msg::Point> edge =
          transformFootprintToEdges(plan.poses[i].pose, _costmap_ros->getRobotFootprint());
        marker_array->markers.push_back(createMarker(edge, i, _global_frame, now));
      }
      _smoothed_footprints_publisher->publish(std::move(marker_array));
    }
  }

  return plan;
}

rcl_interfaces::msg::SetParametersResult
SmacPlannerHybrid::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(_mutex);

  bool reinit_collision_checker = false;
  bool reinit_a_star = false;
  bool reinit_downsampler = false;
  bool reinit_smoother = false;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == _name + ".max_planning_time") {
        reinit_a_star = true;
        _max_planning_time = parameter.as_double();
      } else if (name == _name + ".tolerance") {
        _tolerance = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".lookup_table_size") {
        reinit_a_star = true;
        _lookup_table_size = parameter.as_double();
      } else if (name == _name + ".minimum_turning_radius") {
        reinit_a_star = true;
        if (_smoother) {
          reinit_smoother = true;
        }

        if (parameter.as_double() < _costmap->getResolution() * _downsampling_factor) {
          RCLCPP_ERROR(
            _logger, "Min turning radius cannot be less than the search grid cell resolution!");
          result.successful = false;
        }

        _minimum_turning_radius_global_coords = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".reverse_penalty") {
        reinit_a_star = true;
        _search_info.reverse_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".change_penalty") {
        reinit_a_star = true;
        _search_info.change_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".momentum_zone_length") {
        reinit_a_star = true;
        _momentum_zone_length_m = parameter.as_double();  // -> cells in reinitialize()
      } else if (name == _name + ".motion_reversal_penalty") {
        reinit_a_star = true;
        _motion_reversal_penalty_m = parameter.as_double();  // -> cells in reinitialize()
      } else if (name == _name + ".motion_window") {
        std::lock_guard<std::mutex> lock(_motion_mutex);
        _motion_window = parameter.as_double();
      } else if (name == _name + ".motion_threshold") {
        std::lock_guard<std::mutex> lock(_motion_mutex);
        _motion_threshold = parameter.as_double();
      } else if (name == _name + ".motion_stale_timeout") {
        std::lock_guard<std::mutex> lock(_motion_mutex);
        _motion_stale_timeout = parameter.as_double();
      } else if (name == _name + ".curvature_penalty") {
        reinit_a_star = true;
        _search_info.curvature_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".cusp_tail_length") {
        reinit_a_star = true;
        _cusp_tail_length_m = parameter.as_double();  // -> cells in reinitialize()
      } else if (name == _name + ".momentum_zone_min_radius") {
        reinit_a_star = true;
        _momentum_zone_min_radius_m = parameter.as_double();  // -> cells in reinitialize()
      } else if (name == _name + ".momentum_zone_penalty") {
        reinit_a_star = true;
        _search_info.momentum_zone_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".extra_direction_change_penalty") {
        reinit_a_star = true;
        _search_info.extra_direction_change_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".arc_min_radius") {
        _arc_min_radius = parameter.as_double();
      } else if (name == _name + ".arc_max_length") {
        _arc_max_length = parameter.as_double();
      } else if (name == _name + ".arc_max_cost") {
        _arc_max_cost = parameter.as_double();
      } else if (name == _name + ".arc_max_sweep") {
        _arc_max_sweep = parameter.as_double();
      } else if (name == _name + ".arc_hold_min_radius") {
        _arc_hold_min_radius = parameter.as_double();
      } else if (name == _name + ".arc_hold_max_sweep") {
        _arc_hold_max_sweep = parameter.as_double();
      } else if (name == _name + ".arc_hold_max_offset") {
        _arc_hold_max_offset = parameter.as_double();
      } else if (name == _name + ".arc_hold_max_heading_error") {
        _arc_hold_max_heading_error = parameter.as_double();
      } else if (name == _name + ".arc_hold_end_distance") {
        _arc_hold_end_distance = parameter.as_double();
      } else if (name == _name + ".arc_hold_timeout") {
        _arc_hold_timeout = parameter.as_double();
      } else if (name == _name + ".arc_path_resolution") {
        if (parameter.as_double() > 0.0) {
          _arc_path_resolution = parameter.as_double();
        }
      } else if (name == _name + ".non_straight_penalty") {
        reinit_a_star = true;
        _search_info.non_straight_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".cost_penalty") {
        reinit_a_star = true;
        _search_info.cost_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".analytic_expansion_ratio") {
        reinit_a_star = true;
        _search_info.analytic_expansion_ratio = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".analytic_expansion_max_length") {
        reinit_a_star = true;
        _analytic_expansion_max_length_m = parameter.as_double();  // -> cells in reinitialize()
      } else if (name == _name + ".analytic_expansion_max_cost") {
        reinit_a_star = true;
        _search_info.analytic_expansion_max_cost = static_cast<float>(parameter.as_double());
      } else if (name == "resolution") {
        // Special case: When the costmap's resolution changes, need to reinitialize
        // the controller to have new resolution information
        RCLCPP_INFO(_logger, "Costmap resolution changed. Reinitializing SmacPlannerHybrid.");
        reinit_collision_checker = true;
        reinit_a_star = true;
        reinit_downsampler = true;
        reinit_smoother = true;
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == _name + ".downsample_costmap") {
        reinit_downsampler = true;
        _downsample_costmap = parameter.as_bool();
      } else if (name == _name + ".allow_unknown") {
        reinit_a_star = true;
        _allow_unknown = parameter.as_bool();
      } else if (name == _name + ".cache_obstacle_heuristic") {
        reinit_a_star = true;
        _search_info.cache_obstacle_heuristic = parameter.as_bool();
      } else if (name == _name + ".allow_primitive_interpolation") {
        _search_info.allow_primitive_interpolation = parameter.as_bool();
        reinit_a_star = true;
      } else if (name == _name + ".arc_mode_enabled") {
        _arc_mode_enabled = parameter.as_bool();
      } else if (name == _name + ".escalate_only_on_reset") {
        _search_info.escalate_only_on_reset = parameter.as_bool();
        reinit_a_star = true;
      } else if (name == _name + ".ignore_goal_heading") {
        _search_info.ignore_goal_heading = parameter.as_bool();
        reinit_a_star = true;
      } else if (name == _name + ".smooth_path") {
        if (parameter.as_bool()) {
          reinit_smoother = true;
        } else {
          _smoother.reset();
        }
      } else if (name == _name + ".analytic_expansion_max_cost_override") {
        _search_info.analytic_expansion_max_cost_override = parameter.as_bool();
        reinit_a_star = true;
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == _name + ".downsampling_factor") {
        reinit_a_star = true;
        reinit_downsampler = true;
        _downsampling_factor = parameter.as_int();
      } else if (name == _name + ".max_iterations") {
        reinit_a_star = true;
        _max_iterations = parameter.as_int();
        if (_max_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "maximum iteration selected as <= 0, "
            "disabling maximum iterations.");
          _max_iterations = std::numeric_limits<int>::max();
        }
      } else if (name == _name + ".max_on_approach_iterations") {
        reinit_a_star = true;
        _max_on_approach_iterations = parameter.as_int();
        if (_max_on_approach_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "On approach iteration selected as <= 0, "
            "disabling tolerance and on approach iterations.");
          _max_on_approach_iterations = std::numeric_limits<int>::max();
        }
      } else if (name == _name + ".terminal_checking_interval") {
        reinit_a_star = true;
        _terminal_checking_interval = parameter.as_int();
      } else if (name == _name + ".angle_quantization_bins") {
        reinit_collision_checker = true;
        reinit_a_star = true;
        int angle_quantizations = parameter.as_int();
        _angle_bin_size = 2.0 * M_PI / angle_quantizations;
        _angle_quantizations = static_cast<unsigned int>(angle_quantizations);
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == _name + ".motion_model_for_search") {
        reinit_a_star = true;
        _motion_model = fromString(parameter.as_string());
        if (_motion_model == MotionModel::UNKNOWN) {
          RCLCPP_WARN(
            _logger,
            "Unable to get MotionModel search type. Given '%s', "
            "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.",
            _motion_model_for_search.c_str());
        }
      }
    }
  }

  // Re-init if needed with mutex lock (to avoid re-init while creating a plan)
  reinitialize(reinit_collision_checker, reinit_a_star, reinit_downsampler, reinit_smoother);
  result.successful = true;
  return result;
}

bool SmacPlannerHybrid::tryArcPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav2_costmap_2d::Costmap2D * costmap, int moving_gear,
  nav_msgs::msg::Path & plan, std::string & reason)
{
  const double x0 = start.pose.position.x;
  const double y0 = start.pose.position.y;
  const double th0 = tf2::getYaw(start.pose.orientation);
  const double gx = goal.pose.position.x - x0;
  const double gy = goal.pose.position.y - y0;
  // goal in the start frame
  const double dx = std::cos(th0) * gx + std::sin(th0) * gy;
  const double dy = -std::sin(th0) * gx + std::cos(th0) * gy;
  const double d2 = dx * dx + dy * dy;
  if (d2 < 1e-6) {
    reason = "goal at start";
    return false;
  }
  const int gear = dx >= 0.0 ? 1 : -1;  // goal ahead: forward, behind: reverse
  if (moving_gear != 0 && moving_gear != gear) {
    reason = std::string("arc would reverse the rover, moving ") +
      (moving_gear > 0 ? "forward" : "in reverse");
    return false;
  }

  // Circle tangent to the heading through both points: curvature
  // k = 2 dy / d^2. Along it, pose(s) = (sin(ks)/k, (1 - cos(ks))/k, th0 + ks)
  // for signed arc length s (s < 0 in reverse); the goal is where
  // ks/2 = atan(dy/dx), so the heading sweep 2*atan(dy/dx) stays under 180 deg.
  const double k = 2.0 * dy / d2;
  const int turn = std::fabs(k) < 1e-3 ? 0 : (k > 0.0 ? 1 : -1);
  const double now = _clock->now().seconds();
  // Arc hold: on the arc we are already driving, re-fits may be tighter and
  // longer than a fresh arc, and near the goal the radius is not checked
  // (there it swings with every centimetre of drift)
  const bool held = arcCommitted(x0, y0, th0, goal.pose.position.x, goal.pose.position.y,
      gear, turn, now);
  const double fresh_min_radius =
    _arc_min_radius > 0.0 ? _arc_min_radius : _minimum_turning_radius_global_coords;
  const double min_radius = held ? std::min(_arc_hold_min_radius, fresh_min_radius) :
    fresh_min_radius;
  const double max_sweep = held && _arc_hold_max_sweep > 0.0 ?
    std::max(_arc_hold_max_sweep, _arc_max_sweep) : _arc_max_sweep;
  const bool end_zone = held && std::sqrt(d2) <= _arc_hold_end_distance;
  if (!end_zone && std::fabs(k) * min_radius > 1.0) {
    reason = "radius " + std::to_string(1.0 / std::fabs(k)) + " m < " +
      std::to_string(min_radius) + " m" + (held ? " (held)" : "");
    return false;
  }
  double s_goal;
  if (std::fabs(k) < 1e-9) {
    s_goal = dx;
  } else if (std::fabs(dx) < 1e-9) {
    reason = "goal abeam";  // half circle, already rejected by radius unless huge
    return false;
  } else {
    s_goal = 2.0 * std::atan(dy / dx) / k;
  }
  // heading change along the arc = 2 * the goal's angle off the driving
  // direction, so a cap of e.g. 90 deg keeps goals within +/-45 deg of the
  // nose (forward) or tail (reverse); wider swings go to Hybrid-A*, which can
  // pick a three-point turn instead
  const double sweep_deg = std::fabs(2.0 * std::atan(dy / dx)) * 180.0 / M_PI;
  if (sweep_deg > max_sweep) {
    reason = "sweep " + std::to_string(sweep_deg) + " deg > " + std::to_string(max_sweep) +
      " deg" + (held ? " (held)" : "");
    return false;
  }
  const double length = std::fabs(s_goal);
  if (length > _arc_max_length) {
    reason = "length " + std::to_string(length) + " m > " + std::to_string(_arc_max_length) + " m";
    return false;
  }

  const int n = std::max(1, static_cast<int>(std::ceil(length / _arc_path_resolution)));
  // sample i of n along the arc: world pose, map coords and angle bin
  auto sample = [&](int i, double & s, double & wx, double & wy, double & th,
      float & mx, float & my, float & bin) {
      s = s_goal * static_cast<double>(i) / static_cast<double>(n);
      double lx, ly;
      if (std::fabs(k) < 1e-9) {
        lx = s;
        ly = 0.0;
      } else {
        lx = std::sin(k * s) / k;
        ly = (1.0 - std::cos(k * s)) / k;
      }
      wx = x0 + std::cos(th0) * lx - std::sin(th0) * ly;
      wy = y0 + std::sin(th0) * lx + std::cos(th0) * ly;
      th = angles::normalize_angle(th0 + k * s);
      double b = (th < 0.0 ? th + 2.0 * M_PI : th) / _angle_bin_size;
      if (b >= static_cast<double>(_angle_quantizations)) {
        b -= static_cast<double>(_angle_quantizations);
      }
      bin = static_cast<float>(b);
      return costmap->worldToMapContinuous(wx, wy, mx, my);
    };

  // Cost cap: arc_max_cost, raised to the start's and goal's own cost so a
  // start or goal inside inflation does not by itself reject the arc (the
  // analytic expansion has a similar goal exemption). Collisions always reject.
  double max_cost =
    _arc_max_cost >= 0.0 ? _arc_max_cost : _search_info.analytic_expansion_max_cost;
  for (int i : {0, n}) {
    double s, wx, wy, th;
    float mx, my, bin;
    if (sample(i, s, wx, wy, th, mx, my, bin) &&
      !_collision_checker.inCollision(mx, my, bin, _allow_unknown))
    {
      max_cost = std::max(max_cost, static_cast<double>(_collision_checker.getCost()));
    }
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.reserve(n + 1);
  for (int i = 0; i <= n; ++i) {
    double s, wx, wy, th;
    float mx, my, bin;
    if (!sample(i, s, wx, wy, th, mx, my, bin)) {
      reason = "leaves the costmap";
      return false;
    }
    if (_collision_checker.inCollision(mx, my, bin, _allow_unknown)) {
      reason = "collision at s=" + std::to_string(std::fabs(s)) + " m";
      return false;
    }
    if (_collision_checker.getCost() > max_cost) {
      reason = "cost " + std::to_string(_collision_checker.getCost()) + " > " +
        std::to_string(max_cost) + " at s=" + std::to_string(std::fabs(s)) + " m";
      return false;
    }
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = getWorldOrientation(static_cast<float>(th));
    poses.push_back(pose);
  }

  // remember this arc: the next replan for this goal may hold on to it
  _arc_commit.valid = _arc_hold_min_radius > 0.0;
  _arc_commit.t = now;
  _arc_commit.gx = goal.pose.position.x;
  _arc_commit.gy = goal.pose.position.y;
  _arc_commit.gear = gear;
  _arc_commit.turn = turn;
  _arc_commit.poses.clear();
  _arc_commit.poses.reserve(poses.size());
  for (const auto & p : poses) {
    _arc_commit.poses.push_back(
      {p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation)});
  }

  plan.poses = std::move(poses);
  const double radius =
    std::fabs(k) < 1e-9 ? std::numeric_limits<double>::infinity() : 1.0 / std::fabs(k);
  RCLCPP_INFO(
    _logger, "%s: arc plan, %s, radius %.2f m, length %.2f m, sweep %.0f deg%s", _name.c_str(),
    gear > 0 ? "forward" : "reverse", radius, length, sweep_deg,
    end_zone ? " (held, end zone)" :
    (held && (radius < fresh_min_radius || sweep_deg > _arc_max_sweep)) ? " (held)" : "");
  return true;
}

bool SmacPlannerHybrid::arcCommitted(
  double x, double y, double yaw, double gx, double gy, int gear, int turn, double now) const
{
  const auto & c = _arc_commit;
  if (_arc_hold_min_radius <= 0.0 || !c.valid || c.poses.empty()) {
    return false;
  }
  if (now - c.t > _arc_hold_timeout || std::hypot(gx - c.gx, gy - c.gy) > 0.05 ||
    gear != c.gear || (turn != 0 && c.turn != 0 && turn != c.turn))
  {
    return false;
  }
  // nearest pose of the committed arc (poses are arc_path_resolution apart)
  double best = std::numeric_limits<double>::infinity();
  double best_yaw = 0.0;
  for (const auto & p : c.poses) {
    const double d = std::hypot(p[0] - x, p[1] - y);
    if (d < best) {
      best = d;
      best_yaw = p[2];
    }
  }
  const double heading_err =
    std::fabs(angles::shortest_angular_distance(best_yaw, yaw)) * 180.0 / M_PI;
  return best <= _arc_hold_max_offset && heading_err <= _arc_hold_max_heading_error;
}

void SmacPlannerHybrid::updateTailTracker(double now, double x, double y, double yaw)
{
  // caller holds _motion_mutex
  auto & tt = _tail;
  if (!tt.init) {
    tt.init = true;
    tt.t = now;
    tt.x = x;
    tt.y = y;
    tt.yaw = yaw;
    return;
  }
  if (now - tt.t < 0.1) {
    return;
  }
  // signed displacement along the heading over this ~0.1 s step
  const double d = (x - tt.x) * std::cos(yaw) + (y - tt.y) * std::sin(yaw);
  if (std::fabs(d) >= 0.005) {  // moving (>= ~5 cm/s)
    const int g = d > 0.0 ? 1 : -1;
    if (g != tt.gear) {
      tt.gear = g;
      tt.gear_dist = 0.0;
      tt.run = 0.0;
    } else {
      tt.gear_dist += std::fabs(d);
      // tight = tighter than the gentle radius the planner allows in tails
      const double dyaw = std::fabs(angles::shortest_angular_distance(tt.yaw, yaw));
      const double r_gentle = _momentum_zone_min_radius_m;
      const bool tight = r_gentle > 0.0 ? dyaw * r_gentle > std::fabs(d) : dyaw > 0.01;
      tt.run = tight ? 0.0 : tt.run + std::fabs(d);
    }
  }
  tt.t = now;
  tt.x = x;
  tt.y = y;
  tt.yaw = yaw;
}

int SmacPlannerHybrid::currentGear(double & displacement)
{
  displacement = std::numeric_limits<double>::quiet_NaN();
  const double now = _clock->now().seconds();
  std::lock_guard<std::mutex> lock(_motion_mutex);
  if (_motion_samples.empty() || now - _motion_samples.back().t > _motion_stale_timeout) {
    return 0;  // no recent pose: unknown, plan as from rest
  }
  const MotionSample & oldest = _motion_samples.front();
  const MotionSample & newest = _motion_samples.back();
  if (newest.t - oldest.t < 0.5 * _motion_window) {
    return 0;  // not enough history yet for a meaningful delta
  }
  // displacement over the window projected on the current heading (a delta,
  // not a velocity estimate): >0 driving forward, <0 reversing
  displacement = (newest.x - oldest.x) * std::cos(newest.yaw) +
    (newest.y - oldest.y) * std::sin(newest.yaw);
  if (displacement > _motion_threshold) {
    return 1;
  }
  if (displacement < -_motion_threshold) {
    return -1;
  }
  return 0;
}

void SmacPlannerHybrid::reinitialize(
  bool reinit_collision_checker, bool reinit_a_star,
  bool reinit_downsampler, bool reinit_smoother)
{
  if (reinit_a_star || reinit_downsampler || reinit_collision_checker || reinit_smoother) {
    // convert to grid coordinates
    if (!_downsample_costmap) {
      _downsampling_factor = 1;
    }
    _search_info.minimum_turning_radius =
      _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
    _search_info.momentum_zone_length = static_cast<float>(
      _momentum_zone_length_m / (_costmap->getResolution() * _downsampling_factor));
    _search_info.momentum_zone_min_radius = static_cast<float>(
      _momentum_zone_min_radius_m / (_costmap->getResolution() * _downsampling_factor));
    _search_info.cusp_tail_length = static_cast<float>(
      _cusp_tail_length_m / (_costmap->getResolution() * _downsampling_factor));
    _search_info.motion_reversal_penalty = static_cast<float>(
      _motion_reversal_penalty_m / (_costmap->getResolution() * _downsampling_factor));
    _search_info.analytic_expansion_max_length =
      static_cast<float>(_analytic_expansion_max_length_m / _costmap->getResolution());
    _search_resolution = _costmap->getResolution();
    _lookup_table_dim =
      static_cast<float>(_lookup_table_size) /
      static_cast<float>(_costmap->getResolution() * _downsampling_factor);

    // Make sure its a whole number
    _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

    // Make sure its an odd number
    if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
      RCLCPP_INFO(
        _logger,
        "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
        _lookup_table_dim);
      _lookup_table_dim += 1.0;
    }

    auto node = _node.lock();

    // Re-Initialize A* template
    if (reinit_a_star) {
      _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
      _a_star->initialize(
        _allow_unknown,
        _max_iterations,
        _max_on_approach_iterations,
        _terminal_checking_interval,
        _max_planning_time,
        _lookup_table_dim,
        _angle_quantizations);
    }

    // Re-Initialize costmap downsampler
    if (reinit_downsampler) {
      if (_downsample_costmap && _downsampling_factor > 1) {
        std::string topic_name = "downsampled_costmap";
        _costmap_downsampler = std::make_unique<CostmapDownsampler>();
        _costmap_downsampler->on_configure(
          node, _global_frame, topic_name, _costmap, _downsampling_factor);
      }
    }

    // Re-Initialize collision checker
    if (reinit_collision_checker) {
      _collision_checker = GridCollisionChecker(_costmap_ros, _angle_quantizations, node);
      _collision_checker.setFootprint(
        _costmap_ros->getRobotFootprint(),
        _costmap_ros->getUseRadius(),
        findCircumscribedCost(_costmap_ros));
    }

    // Re-Initialize smoother
    if (reinit_smoother) {
      SmootherParams params;
      params.get(node, _name);
      _smoother = std::make_unique<Smoother>(params);
      _smoother->initialize(_minimum_turning_radius_global_coords);
    }
  }
}

}  // namespace nav2_smac_planner_custom

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smac_planner_custom::SmacPlannerHybrid, nav2_core::GlobalPlanner)
