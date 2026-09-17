// Copyright (c) 2024 CubeRover Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef NAV2_MPPI_CONTROLLER__NN_DYNAMICS_HPP_
#define NAV2_MPPI_CONTROLLER__NN_DYNAMICS_HPP_

#include <array>
#include <string>

// Suppress warnings from torch headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"
#include <torch/script.h>
#include <torch/cuda.h>
#include <c10/cuda/CUDAFunctions.h>
#include <c10/cuda/CUDAGuard.h>
#include <c10/cuda/CUDAStream.h>
#include <ATen/cuda/CUDAGraph.h>
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xnoalias.hpp>
#include <xtensor/xmanipulation.hpp>
#pragma GCC diagnostic pop

#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"

namespace mppi
{

/**
 * @class NNDynamics
 * @brief Selectable trajectory-integration dynamics for the MPPI controller.
 *
 * Three modes (Config::mode), all correcting the *full* rollout horizon
 * step-by-step (autoregressive), matching the offline-validated v11
 * "linear_ar_velocity" / "mlp*_ar_velocity_teacher" architecture family
 * (LOOKBACK=0: only the current step's commanded [vx, wz] is used, no
 * command history is needed):
 *
 *   Kinematics    : pure unicycle integration, no correction (default).
 *   Linear        : hardcoded 2x2 linear correction, weights loaded from
 *                   ROS2 params (no libtorch involved at all — cheapest).
 *   NeuralNetwork : per-step TorchScript MLP forward pass, replayed as a
 *                   captured CUDA graph when running on GPU (eliminates the
 *                   25x per-step kernel-launch overhead that otherwise
 *                   dominates autoregressive rollout cost; falls back to an
 *                   eager per-step loop on CPU or if capture is unavailable).
 *
 * Every mode operates in the *robot frame* (matching training-side
 * ar_rollout's convention: rollout starts at (0,0,0)) and is rotated into
 * the world frame once at the end using the current pose (x0,y0,yaw0),
 * since that varies every planning cycle and can't be baked into a captured
 * graph.
 */
class NNDynamics
{
public:
  enum class DynamicsMode { Kinematics, Linear, NeuralNetwork };

  /**
   * @brief Configuration loaded from ROS2 parameters.
   */
  struct Config
  {
    DynamicsMode mode{DynamicsMode::Kinematics};
    std::string model_path;        ///< TorchScript .pt, only used when mode==NeuralNetwork
    bool use_cuda{true};           ///< attempt CUDA; falls back to CPU if unavailable
    unsigned int batch_size{0};    ///< settings_.batch_size — sizes the captured CUDA graph
    int horizon{0};                ///< settings_.time_steps — AR now corrects the full horizon
    float model_dt{0.f};           ///< settings_.model_dt — baked into the NN path at capture
                                    ///< time (a CUDA graph needs a fixed constant, not a
                                    ///< per-call argument); unused by Kinematics/Linear, which
                                    ///< still take model_dt as a per-call argument

    // Normalization for [vx_cmd, wz_cmd] -- shared by Linear and NeuralNetwork
    // modes (both trained against the same v11 Normalizer fit).
    std::array<float, 2> lin_fmean{};
    std::array<float, 2> lin_fstd{};

    // Linear-mode weights only: correction = W @ normalize([vx_cmd, wz_cmd]) + b
    // W is row-major [out, in] (i.e. {W00, W01, W10, W11}).
    std::array<float, 4> lin_weight{};
    std::array<float, 2> lin_bias{};
  };

  /**
   * @brief Construct, and for NeuralNetwork mode, load the TorchScript model
   * and (on CUDA) capture the AR rollout as a CUDA graph.
   * @param cfg   Configuration struct populated from ROS2 params
   * @param logger ROS2 logger for status/warning messages
   */
  NNDynamics(const Config & cfg, rclcpp::Logger logger);

  /**
   * @brief Integrate the full horizon according to cfg_.mode.
   *
   * Writes directly into trajectories.x, trajectories.y, trajectories.yaws.
   *
   * @param trajectories  Output trajectory arrays to fill (batch × time_steps)
   * @param state         Current MPPI state (pose, speed, noised cmd arrays)
   * @param model_dt      Timestep in seconds
   */
  void integrateTrajectories(
    models::Trajectories & trajectories,
    const models::State & state,
    float model_dt) const;

  DynamicsMode getMode() const {return cfg_.mode;}
  bool isEnabled() const {return enabled_;}

private:
  /// Pure unicycle forward integration for a slice of time steps (also used
  /// as the Kinematics-mode implementation in full).
  void integrateKinematicsOnly(
    models::Trajectories & trajectories,
    const models::State & state,
    float model_dt) const;

  /// Hardcoded linear correction, vectorized across the batch with xtensor —
  /// no torch involved.
  void integrateLinear(
    models::Trajectories & trajectories,
    const models::State & state,
    float model_dt) const;

  /// Per-step TorchScript MLP correction, via captured CUDA graph on GPU or
  /// an eager per-step loop otherwise.
  void integrateNeuralNetwork(
    models::Trajectories & trajectories,
    const models::State & state,
    float model_dt) const;

  /// Runs the H-step AR loop (robot frame) using module_, writing a
  /// (batch, horizon, 3) [rx, ry, rth] tensor. Shared by graph capture,
  /// warmup, and the CPU eager fallback so there is one source of truth for
  /// the math.
  torch::Tensor arLoop(const torch::Tensor & cmd) const;

  /// One-time CUDA graph capture of arLoop() over static_input_. Must be
  /// followed by exactly one replay() before static_output_ is valid — the
  /// buffer captured inside capture_begin/capture_end is not yet
  /// materialized with real results until the graph has been replayed once.
  void captureGraph();

  Config cfg_;
  mutable torch::jit::Module module_;  // forward() is non-const in libtorch
  torch::Device device_{torch::kCPU};
  bool enabled_{false};

  // Persistent on-device copies of cfg_.lin_fmean/lin_fstd, built once at
  // construction. arLoop() must never build a fresh tensor from a host
  // initializer list itself -- doing that from *inside* a captured CUDA graph
  // region depends on a fresh, non-persistent host pointer each call, which
  // CUDA graph capture cannot safely replay.
  torch::Tensor fmean_dev_, fstd_dev_;

  mutable at::cuda::CUDAGraph graph_;
  mutable torch::Tensor static_input_;   // (batch, horizon, 2), copy_() target each call
  mutable torch::Tensor static_output_;  // (batch, horizon, 3) [rx, ry, rth]
  bool graph_captured_{false};

  rclcpp::Logger logger_;
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__NN_DYNAMICS_HPP_
