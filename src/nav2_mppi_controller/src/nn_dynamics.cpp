// Copyright (c) 2024 CubeRover Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Compile without -ffast-math so libtorch NaN/Inf checks are not optimised away.
#pragma GCC optimize("no-fast-math")

#include "nav2_mppi_controller/nn_dynamics.hpp"

#include <cmath>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#include <xtensor/xadapt.hpp>
#pragma GCC diagnostic pop

#include "tf2/utils.h"

namespace mppi
{

NNDynamics::NNDynamics(const Config & cfg, rclcpp::Logger logger)
: cfg_(cfg), logger_(logger)
{
  cmd_history_.assign(cfg_.lookback, {0.0f, 0.0f});

  if (cfg_.use_cuda && c10::cuda::device_count() > 0) {
    device_ = torch::Device(torch::kCUDA);
    RCLCPP_INFO(
      logger_, "NNDynamics: CUDA available — %d device(s) detected",
      static_cast<int>(c10::cuda::device_count()));
  } else {
    device_ = torch::Device(torch::kCPU);
    if (cfg_.use_cuda) {
      RCLCPP_WARN(logger_, "NNDynamics: CUDA requested but not available, falling back to CPU");
    }
  }

  try {
    module_ = torch::jit::load(cfg_.model_path, device_);
    module_.eval();

    // Warm-up so CUDA kernels are JIT-compiled before the first real call
    {
      torch::NoGradGuard no_grad;
      auto dummy = torch::zeros(
        {4, cfg_.lookback + cfg_.nn_horizon, 2},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));
      module_.forward({dummy});
    }

    enabled_ = true;
    RCLCPP_INFO(
      logger_, "NNDynamics: loaded '%s'  lookback=%d  nn_horizon=%d",
      cfg_.model_path.c_str(), cfg_.lookback, cfg_.nn_horizon);
  } catch (const c10::Error & e) {
    RCLCPP_ERROR(
      logger_,
      "NNDynamics: failed to load model from '%s': %s — will use pure kinematics",
      cfg_.model_path.c_str(), e.what());
    enabled_ = false;
  }
}

void NNDynamics::recordCommand(float vx_cmd, float wz_cmd)
{
  cmd_history_.push_back({vx_cmd, wz_cmd});
  if (static_cast<int>(cmd_history_.size()) > cfg_.lookback) {
    cmd_history_.pop_front();
  }
}

void NNDynamics::resetHistory()
{
  cmd_history_.assign(cfg_.lookback, {0.0f, 0.0f});
  RCLCPP_DEBUG(logger_, "NNDynamics: command history reset to zeros");
}

torch::Tensor NNDynamics::buildInput(
  const models::State & state, int batch_size) const
{
  const int total = cfg_.lookback + cfg_.nn_horizon;

  auto input = torch::zeros(
    {batch_size, total, 2},
    torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU));

  // History columns — same value broadcast across the batch dimension
  for (int t = 0; t < cfg_.lookback; ++t) {
    float vx_h = cmd_history_[t].first;
    float wz_h = cmd_history_[t].second;
    input.select(1, t).select(1, 0).fill_(vx_h);
    input.select(1, t).select(1, 1).fill_(wz_h);
  }

  // Future command columns — different per trajectory; use strided from_blob.
  // Use state.vx/wz (acceleration-constrained) to match the kinematic baseline.
  // state.vx is row-major [batch, time_steps]; stride {time_steps, 1} selects
  // the first nn_horizon columns without a copy.
  const long time_steps = static_cast<long>(state.vx.shape(1));
  const long B          = static_cast<long>(batch_size);
  const long H          = static_cast<long>(cfg_.nn_horizon);

  auto cvx_blob = torch::from_blob(
    const_cast<float *>(state.vx.data()),
    {B, H}, {time_steps, 1L}, torch::kFloat32);

  auto cwz_blob = torch::from_blob(
    const_cast<float *>(state.wz.data()),
    {B, H}, {time_steps, 1L}, torch::kFloat32);

  // Copy into the future slice: input[:, lookback:, :]
  auto future = input.slice(1, cfg_.lookback, total);
  future.select(2, 0).copy_(cvx_blob);
  future.select(2, 1).copy_(cwz_blob);

  return input;
}

void NNDynamics::integrateKinematics(
  models::Trajectories & trajectories,
  const models::State & state,
  int start_step,
  int end_step,
  float model_dt) const
{
  // Vectorised unicycle Euler step for each time step, across all batch elements.
  // Caller is responsible for seeding step (start_step - 1) before calling.
  for (int k = start_step; k < end_step; ++k) {
    auto yaw_prev = xt::view(trajectories.yaws, xt::all(), k - 1);
    auto vx_k     = xt::view(state.vx,          xt::all(), k);
    auto wz_k     = xt::view(state.wz,          xt::all(), k);

    xt::view(trajectories.yaws, xt::all(), k) = yaw_prev + wz_k * model_dt;
    xt::view(trajectories.x,    xt::all(), k) =
      xt::view(trajectories.x, xt::all(), k - 1) + vx_k * xt::cos(yaw_prev) * model_dt;
    xt::view(trajectories.y,    xt::all(), k) =
      xt::view(trajectories.y, xt::all(), k - 1) + vx_k * xt::sin(yaw_prev) * model_dt;
  }
}

void NNDynamics::integrateTrajectories(
  models::Trajectories & trajectories,
  const models::State & state,
  float model_dt) const
{
  const int   batch_size = static_cast<int>(state.vx.shape(0));
  const int   time_steps = static_cast<int>(state.vx.shape(1));
  const float x0         = static_cast<float>(state.pose.pose.position.x);
  const float y0         = static_cast<float>(state.pose.pose.position.y);
  const float yaw0       = static_cast<float>(tf2::getYaw(state.pose.pose.orientation));
  const float c0         = std::cos(yaw0);
  const float s0         = std::sin(yaw0);

  // ── Pure-kinematics fallback ─────────────────────────────────────────────
  if (!enabled_ || !cfg_.use_nn) {
    // Seed step 0: use MPPI-sampled command at step 0, current heading
    auto vx_0 = xt::view(state.vx, xt::all(), 0);
    auto wz_0 = xt::view(state.wz, xt::all(), 0);
    xt::view(trajectories.yaws, xt::all(), 0) = yaw0 + wz_0 * model_dt;
    xt::view(trajectories.x,    xt::all(), 0) = x0 + xt::eval(vx_0) * c0 * model_dt;
    xt::view(trajectories.y,    xt::all(), 0) = y0 + xt::eval(vx_0) * s0 * model_dt;
    integrateKinematics(trajectories, state, 1, time_steps, model_dt);
    return;
  }

  // ── Step 1: NN inference ─────────────────────────────────────────────────
  torch::Tensor residuals_cpu;
  {
    torch::NoGradGuard no_grad;
    auto input_cpu = buildInput(state, batch_size);
    auto input_dev = input_cpu.to(device_);
    residuals_cpu  = module_.forward({input_dev}).toTensor().to(torch::kCPU).contiguous();
  }
  // residuals_cpu: [batch, nn_horizon, 3]  robot-frame residuals [dx, dy, dtheta]

  const int H = cfg_.nn_horizon;

  // ── Step 2: Robot-frame kinematic baseline for steps 0..H-1 ─────────────
  // Mirrors _batch_kinematics from the training script.
  // Robot frame: starts at (0, 0, 0).  heading_for_position_k = sum(wz[0..k-1])*dt.

  auto nn_vx = xt::view(state.vx, xt::all(), xt::range(0, H));  // [batch, H]
  auto nn_wz = xt::view(state.wz, xt::all(), xt::range(0, H));

  // Cumulative robot-frame heading after each step: ths[k] = sum(wz[0..k])*dt
  auto kin_rth  = xt::eval(xt::cumsum(nn_wz * model_dt, {1}));  // [batch, H]
  auto kin_cos  = xt::eval(xt::cos(kin_rth));
  auto kin_sin  = xt::eval(xt::sin(kin_rth));

  // Heading used for position update at step k is ths[k-1] (before wz[k] is applied).
  // Equivalent to rolling kin_cos right by 1 along axis 1, then setting col 0 = 1.
  // Use explicit xtensor type — xt::zeros with a shape arg returns a lazy broadcast
  // (read-only); assigning to xt::xtensor<float,2> materialises it immediately.
  xt::xtensor<float, 2> cos_prev = xt::zeros<float>({(std::size_t)batch_size, (std::size_t)H});
  xt::xtensor<float, 2> sin_prev = xt::zeros<float>({(std::size_t)batch_size, (std::size_t)H});
  if (H > 1) {
    xt::view(cos_prev, xt::all(), xt::range(1, H)) =
      xt::view(kin_cos, xt::all(), xt::range(0, H - 1));
    xt::view(sin_prev, xt::all(), xt::range(1, H)) =
      xt::view(kin_sin, xt::all(), xt::range(0, H - 1));
  }
  xt::view(cos_prev, xt::all(), 0) = 1.0f;  // cos(0) — robot starts aligned
  xt::view(sin_prev, xt::all(), 0) = 0.0f;  // sin(0)

  auto nn_vx_eval = xt::eval(nn_vx);
  auto kin_rx = xt::eval(xt::cumsum(nn_vx_eval * cos_prev * model_dt, {1}));  // [batch, H]
  auto kin_ry = xt::eval(xt::cumsum(nn_vx_eval * sin_prev * model_dt, {1}));

  // ── Step 3: Add NN residuals; rotate robot-frame → world-frame ───────────
  // Wrap residuals_cpu as a zero-copy xtensor adaptor
  auto res_view = xt::adapt(
    residuals_cpu.data_ptr<float>(),
    static_cast<std::size_t>(batch_size * H * 3),
    xt::no_ownership(),
    std::vector<std::size_t>{
      static_cast<std::size_t>(batch_size),
      static_cast<std::size_t>(H),
      3u});

  // full robot-frame displacement = kinematic + NN residual
  auto rx  = xt::eval(kin_rx  + xt::view(res_view, xt::all(), xt::all(), 0));
  auto ry  = xt::eval(kin_ry  + xt::view(res_view, xt::all(), xt::all(), 1));
  auto rth = xt::eval(kin_rth + xt::view(res_view, xt::all(), xt::all(), 2));

  xt::view(trajectories.x,    xt::all(), xt::range(0, H)) = x0 + c0 * rx - s0 * ry;
  xt::view(trajectories.y,    xt::all(), xt::range(0, H)) = y0 + s0 * rx + c0 * ry;
  xt::view(trajectories.yaws, xt::all(), xt::range(0, H)) = yaw0 + rth;

  // ── Step 4: Unicycle continuation from the NN's last predicted state ──────
  if (H < time_steps) {
    integrateKinematics(trajectories, state, H, time_steps, model_dt);
  }
}

}  // namespace mppi
