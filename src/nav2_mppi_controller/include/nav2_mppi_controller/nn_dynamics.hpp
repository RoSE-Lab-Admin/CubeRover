// Copyright (c) 2024 CubeRover Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef NAV2_MPPI_CONTROLLER__NN_DYNAMICS_HPP_
#define NAV2_MPPI_CONTROLLER__NN_DYNAMICS_HPP_

#include <deque>
#include <string>
#include <utility>

// Suppress warnings from torch headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"
#include <torch/script.h>
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xnoalias.hpp>
#pragma GCC diagnostic pop

#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"

namespace mppi
{

/**
 * @class NNDynamics
 * @brief Learned residual dynamics model for the MPPI trajectory integrator.
 *
 * Replaces (or corrects) the first nn_horizon steps of the kinematic unicycle
 * integration with a neural-network-predicted residual trained on real rover data.
 *
 * Architecture (current):
 *   input  : (batch, lookback + nn_horizon, 2)  — raw [vx_cmd, wz_cmd] sequence
 *              columns 0..lookback-1  : actual command history (same for all batch)
 *              columns lookback..end  : MPPI noised future commands (per trajectory)
 *   output : (batch, nn_horizon, 3)  — residual in robot frame [dx, dy, dtheta]
 *              full prediction = kinematics(future_cmds) + residual
 *
 * Future extension points (via Config):
 *   - nn_horizon     : change how many steps the NN covers
 *   - lookback       : change how many past steps are fed as context
 *   - input_mode     : "cmd_only" (current) or "cmd_and_state" (feed [x,y,theta,vx,wz])
 *   - model_path     : hot-swap architecture by pointing to a different .pt file
 *   - online_update  : set true and call updateWeights() after each trajectory
 */
class NNDynamics
{
public:
  /**
   * @brief Configuration loaded from ROS2 parameters.
   *
   * All fields can be extended; add new entries here and read them in
   * Optimizer::getParams() alongside the existing parameter calls.
   */
  struct Config
  {
    std::string model_path;  ///< absolute path to TorchScript .pt file
    int lookback{20};        ///< past command steps fed as context
    int nn_horizon{20};      ///< future steps the NN predicts
    bool use_cuda{true};     ///< attempt CUDA; falls back to CPU if unavailable
    // Future: std::string input_mode{"cmd_only"};
    // Future: bool online_update{false};
  };

  /**
   * @brief Construct and load the TorchScript model.
   * @param cfg   Configuration struct populated from ROS2 params
   * @param logger ROS2 logger for status/warning messages
   */
  NNDynamics(const Config & cfg, rclcpp::Logger logger);

  /**
   * @brief Record the command that was actually sent to the robot this cycle.
   *
   * Maintains a rolling window of length lookback.  Call once per control cycle
   * after getControlFromSequenceAsTwist(), BEFORE the next integrateWithNN().
   */
  void recordCommand(float vx_cmd, float wz_cmd);

  /**
   * @brief Reset command history to zeros.
   *
   * Call when starting a new trajectory or after optimizer reset so that
   * stale history from a previous run does not contaminate predictions.
   * Zero-padding is correct: the NN was trained on segments starting from rest.
   */
  void resetHistory();

  /**
   * @brief Integrate the first nn_horizon steps using the NN, remaining steps
   * with standard unicycle kinematics.
   *
   * Writes directly into trajectories.x, trajectories.y, trajectories.yaws.
   * If the NN is disabled (load failed), falls back entirely to pure kinematics.
   *
   * @param trajectories  Output trajectory arrays to fill (batch × time_steps)
   * @param state         Current MPPI state (pose, speed, noised cmd arrays)
   * @param model_dt      Timestep in seconds
   */
  void integrateTrajectories(
    models::Trajectories & trajectories,
    const models::State & state,
    float model_dt) const;

  int  getNNHorizon() const {return cfg_.nn_horizon;}
  int  getLookback()  const {return cfg_.lookback;}
  bool isEnabled()    const {return enabled_;}

private:
  /**
   * @brief Build the (batch, lookback+nn_horizon, 2) input tensor.
   *
   * Tiles the command history across the batch dimension, then copies the
   * first nn_horizon steps of each trajectory's noised commands.
   */
  torch::Tensor buildInput(const models::State & state, int batch_size) const;

  /**
   * @brief Pure unicycle forward integration for a slice of time steps.
   *
   * Used for: (a) NN-horizon steps as the kinematic baseline before adding
   * the residual, and (b) the remaining steps after the NN horizon.
   *
   * @param trajectories   Output arrays
   * @param state          MPPI state with vx/wz arrays
   * @param start_step     First step to integrate (inclusive)
   * @param end_step       Last step to integrate (exclusive)
   * @param model_dt       Timestep
   */
  void integrateKinematics(
    models::Trajectories & trajectories,
    const models::State & state,
    int start_step,
    int end_step,
    float model_dt) const;

  Config cfg_;
  torch::jit::Module module_;
  torch::Device device_{torch::kCPU};
  bool enabled_{false};
  std::deque<std::pair<float, float>> cmd_history_;  ///< rolling [vx, wz] history
  rclcpp::Logger logger_;
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__NN_DYNAMICS_HPP_
