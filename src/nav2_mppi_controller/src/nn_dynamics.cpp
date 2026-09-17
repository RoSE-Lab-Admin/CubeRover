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


namespace mppi
{

NNDynamics::NNDynamics(const Config & cfg, rclcpp::Logger logger)
: cfg_(cfg), logger_(logger)
{
  if (cfg_.mode == DynamicsMode::Kinematics) {
    enabled_ = true;
    RCLCPP_INFO(logger_, "NNDynamics: mode=kinematics (no correction)");
    return;
  }

  if (cfg_.mode == DynamicsMode::Linear) {
    enabled_ = true;
    RCLCPP_INFO(
      logger_,
      "NNDynamics: mode=linear  weight=[%.5f %.5f %.5f %.5f]  bias=[%.5f %.5f]",
      cfg_.lin_weight[0], cfg_.lin_weight[1], cfg_.lin_weight[2], cfg_.lin_weight[3],
      cfg_.lin_bias[0], cfg_.lin_bias[1]);
    return;
  }

  // ── mode == NeuralNetwork ────────────────────────────────────────────────
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

    // Persistent on-device normalization constants -- built once here, never
    // re-created inside arLoop (see header comment on fmean_dev_/fstd_dev_).
    fmean_dev_ = torch::tensor({cfg_.lin_fmean[0], cfg_.lin_fmean[1]}, torch::kFloat32).to(device_);
    fstd_dev_  = torch::tensor({cfg_.lin_fstd[0],  cfg_.lin_fstd[1]},  torch::kFloat32).to(device_);

    // Warm-up so CUDA kernels are JIT-compiled before graph capture / first real call.
    {
      torch::NoGradGuard no_grad;
      auto dummy = torch::zeros(
        {static_cast<int64_t>(cfg_.batch_size), static_cast<int64_t>(cfg_.horizon), 2},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));
      arLoop(dummy);
    }

    enabled_ = true;
    RCLCPP_INFO(
      logger_, "NNDynamics: mode=neural_network  loaded '%s'  batch=%u  horizon=%d  device=%s",
      cfg_.model_path.c_str(), cfg_.batch_size, cfg_.horizon,
      device_.is_cuda() ? "cuda" : "cpu");
  } catch (const c10::Error & e) {
    RCLCPP_ERROR(
      logger_,
      "NNDynamics: failed to load model from '%s': %s — will use pure kinematics",
      cfg_.model_path.c_str(), e.what());
    enabled_ = false;
    return;
  }

  if (device_.is_cuda()) {
    try {
      captureGraph();
      RCLCPP_INFO(logger_, "NNDynamics: CUDA graph captured successfully");
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        logger_,
        "NNDynamics: CUDA graph capture failed (%s) — falling back to eager GPU inference",
        e.what());
      graph_captured_ = false;
    }
  }
}

torch::Tensor NNDynamics::arLoop(const torch::Tensor & cmd) const
{
  // cmd: (batch, horizon, 2) raw [vx_cmd, wz_cmd]. Returns (batch, horizon, 3)
  // robot-frame [rx, ry, rth], matching the training-side ar_rollout's
  // velocity-style convention exactly (rollout starts at (0,0,0); each
  // step's correction is rotated through the model's own accumulated
  // heading, not a precomputed reference).
  const int64_t B = cmd.size(0);
  const int64_t H = cmd.size(1);
  auto opts = cmd.options();

  auto xs  = torch::zeros({B}, opts);
  auto ys  = torch::zeros({B}, opts);
  auto ths = torch::zeros({B}, opts);

  std::vector<torch::Tensor> out;
  out.reserve(static_cast<size_t>(H));

  for (int64_t k = 0; k < H; ++k) {
    auto cmd_k = cmd.select(1, k);                  // (B,2)
    auto x_in  = (cmd_k - fmean_dev_) / fstd_dev_;   // (B,2) normalized
    auto pred  = module_.forward({x_in}).toTensor(); // (B,2) [corr_fwd, corr_yaw]

    auto corrected_fwd = cmd_k.select(1, 0) * cfg_.model_dt + pred.select(1, 0);
    auto corrected_yaw = cmd_k.select(1, 1) * cfg_.model_dt + pred.select(1, 1);

    xs  = xs + corrected_fwd * torch::cos(ths);
    ys  = ys + corrected_fwd * torch::sin(ths);
    ths = ths + corrected_yaw;

    out.push_back(torch::stack({xs, ys, ths}, /*dim=*/1));  // (B,3)
  }

  return torch::stack(out, /*dim=*/1);  // (B,H,3)
}

void NNDynamics::captureGraph()
{
  static_input_ = torch::zeros(
    {static_cast<int64_t>(cfg_.batch_size), static_cast<int64_t>(cfg_.horizon), 2},
    torch::TensorOptions().dtype(torch::kFloat32).device(device_));

  torch::NoGradGuard no_grad;
  c10::cuda::CUDAStream capture_stream = c10::cuda::getStreamFromPool();
  c10::cuda::CUDAStreamGuard guard(capture_stream);

  for (int i = 0; i < 3; ++i) {
    arLoop(static_input_);
  }
  torch::cuda::synchronize();

  graph_.capture_begin();
  static_output_ = arLoop(static_input_);
  graph_.capture_end();

  // The output buffer captured inside capture_begin/capture_end is NOT yet
  // materialized with real results -- confirmed empirically while prototyping
  // this in Python (plotting_data/run_cuda_graph_timing.py): comparing the
  // pre-replay tensor against an eager reference showed an O(1) diff; after
  // exactly one replay() the diff was exactly 0.0. So: replay once here,
  // before trusting static_output_ on the first real call.
  graph_.replay();
  torch::cuda::synchronize();
  graph_captured_ = true;
}

void NNDynamics::integrateKinematicsOnly(
  models::Trajectories & trajectories,
  const models::State & state,
  float model_dt) const
{
  const int time_steps = static_cast<int>(state.vx.shape(1));
  const float x0 = static_cast<float>(state.pose.pose.position.x);
  const float y0 = static_cast<float>(state.pose.pose.position.y);
  const auto & q = state.pose.pose.orientation;
  const float yaw0 = static_cast<float>(
    std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
  const float c0 = std::cos(yaw0);
  const float s0 = std::sin(yaw0);

  auto vx_0 = xt::view(state.vx, xt::all(), 0);
  auto wz_0 = xt::view(state.wz, xt::all(), 0);
  xt::view(trajectories.yaws, xt::all(), 0) = yaw0 + wz_0 * model_dt;
  xt::view(trajectories.x,    xt::all(), 0) = x0 + xt::eval(vx_0) * c0 * model_dt;
  xt::view(trajectories.y,    xt::all(), 0) = y0 + xt::eval(vx_0) * s0 * model_dt;

  for (int k = 1; k < time_steps; ++k) {
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

void NNDynamics::integrateLinear(
  models::Trajectories & trajectories,
  const models::State & state,
  float model_dt) const
{
  const int batch_size = static_cast<int>(state.vx.shape(0));
  const int horizon    = cfg_.horizon;
  const float x0 = static_cast<float>(state.pose.pose.position.x);
  const float y0 = static_cast<float>(state.pose.pose.position.y);
  const auto & q = state.pose.pose.orientation;
  const float yaw0 = static_cast<float>(
    std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
  const float c0 = std::cos(yaw0);
  const float s0 = std::sin(yaw0);

  const float w00 = cfg_.lin_weight[0], w01 = cfg_.lin_weight[1];
  const float w10 = cfg_.lin_weight[2], w11 = cfg_.lin_weight[3];
  const float b0 = cfg_.lin_bias[0], b1 = cfg_.lin_bias[1];
  const float fm0 = cfg_.lin_fmean[0], fm1 = cfg_.lin_fmean[1];
  const float fs0 = cfg_.lin_fstd[0], fs1 = cfg_.lin_fstd[1];

  // Robot-frame running accumulators [xs, ys, ths], starting at (0,0,0) --
  // matches ar_rollout's velocity-style convention.
  auto xs  = xt::eval(xt::zeros<float>({static_cast<std::size_t>(batch_size)}));
  auto ys  = xt::eval(xt::zeros<float>({static_cast<std::size_t>(batch_size)}));
  auto ths = xt::eval(xt::zeros<float>({static_cast<std::size_t>(batch_size)}));

  for (int k = 0; k < horizon; ++k) {
    auto vx_k = xt::view(state.vx, xt::all(), k);
    auto wz_k = xt::view(state.wz, xt::all(), k);

    auto vxn = xt::eval((vx_k - fm0) / fs0);
    auto wzn = xt::eval((wz_k - fm1) / fs1);

    auto corrected_fwd = xt::eval(vx_k * model_dt + (w00 * vxn + w01 * wzn + b0));
    auto corrected_yaw = xt::eval(wz_k * model_dt + (w10 * vxn + w11 * wzn + b1));

    auto cos_ths = xt::eval(xt::cos(ths));
    auto sin_ths = xt::eval(xt::sin(ths));

    xs  = xt::eval(xs + corrected_fwd * cos_ths);
    ys  = xt::eval(ys + corrected_fwd * sin_ths);
    ths = xt::eval(ths + corrected_yaw);

    xt::view(trajectories.x,    xt::all(), k) = x0 + c0 * xs - s0 * ys;
    xt::view(trajectories.y,    xt::all(), k) = y0 + s0 * xs + c0 * ys;
    xt::view(trajectories.yaws, xt::all(), k) = yaw0 + ths;
  }
}

void NNDynamics::integrateNeuralNetwork(
  models::Trajectories & trajectories,
  const models::State & state,
  float model_dt) const
{
  (void)model_dt;  // baked into cfg_.model_dt at construction time (required for CUDA-graph
                    // capture, where the correction step must be a fixed constant, not a
                    // per-call argument); settings_.model_dt is itself a static param so this
                    // is equivalent in practice.

  const int batch_size = static_cast<int>(state.vx.shape(0));
  const int time_steps = static_cast<int>(state.vx.shape(1));
  const int H = cfg_.horizon;
  const float x0 = static_cast<float>(state.pose.pose.position.x);
  const float y0 = static_cast<float>(state.pose.pose.position.y);
  const auto & q = state.pose.pose.orientation;
  const float yaw0 = static_cast<float>(
    std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
  const float c0 = std::cos(yaw0);
  const float s0 = std::sin(yaw0);

  const long B_l = static_cast<long>(batch_size);
  const long ts_l = static_cast<long>(time_steps);
  auto cvx_blob = torch::from_blob(
    const_cast<float *>(state.vx.data()), {B_l, static_cast<long>(H)}, {ts_l, 1L}, torch::kFloat32);
  auto cwz_blob = torch::from_blob(
    const_cast<float *>(state.wz.data()), {B_l, static_cast<long>(H)}, {ts_l, 1L}, torch::kFloat32);
  auto cmd_cpu = torch::stack({cvx_blob, cwz_blob}, /*dim=*/2).contiguous();  // (B,H,2)

  torch::Tensor rxyz_cpu;
  {
    torch::NoGradGuard no_grad;
    if (graph_captured_) {
      static_input_.copy_(cmd_cpu.to(device_));
      graph_.replay();
      rxyz_cpu = static_output_.to(torch::kCPU).contiguous();
    } else {
      auto cmd_dev = cmd_cpu.to(device_);
      rxyz_cpu = arLoop(cmd_dev).to(torch::kCPU).contiguous();
    }
  }

  auto res_view = xt::adapt(
    rxyz_cpu.data_ptr<float>(),
    static_cast<std::size_t>(batch_size * H * 3),
    xt::no_ownership(),
    std::vector<std::size_t>{
      static_cast<std::size_t>(batch_size),
      static_cast<std::size_t>(H),
      3u});

  auto rx  = xt::view(res_view, xt::all(), xt::all(), 0);
  auto ry  = xt::view(res_view, xt::all(), xt::all(), 1);
  auto rth = xt::view(res_view, xt::all(), xt::all(), 2);

  xt::view(trajectories.x,    xt::all(), xt::range(0, H)) = x0 + c0 * rx - s0 * ry;
  xt::view(trajectories.y,    xt::all(), xt::range(0, H)) = y0 + s0 * rx + c0 * ry;
  xt::view(trajectories.yaws, xt::all(), xt::range(0, H)) = yaw0 + rth;
}

void NNDynamics::integrateTrajectories(
  models::Trajectories & trajectories,
  const models::State & state,
  float model_dt) const
{
  switch (cfg_.mode) {
    case DynamicsMode::Linear:
      integrateLinear(trajectories, state, model_dt);
      return;
    case DynamicsMode::NeuralNetwork:
      if (enabled_) {
        integrateNeuralNetwork(trajectories, state, model_dt);
        return;
      }
      // model failed to load -- fall through to pure kinematics
      [[fallthrough]];
    case DynamicsMode::Kinematics:
    default:
      integrateKinematicsOnly(trajectories, state, model_dt);
      return;
  }
}

}  // namespace mppi
