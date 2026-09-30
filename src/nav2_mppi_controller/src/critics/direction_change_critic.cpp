// Copyright (c) 2026 CubeRover Project
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
// limitations under the License.

#include <cmath>

#include "nav2_mppi_controller/critics/direction_change_critic.hpp"

namespace mppi::critics
{

void DirectionChangeCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 8.0f);
  getParam(time_constant_, "time_constant", 1.0f);
  getParam(motion_threshold_, "motion_threshold", 0.02f);
  // Purely a computational short-circuit (skip the exp() once genuinely
  // negligible), not a separate user-facing cutoff -- decay is already
  // ~0.007 by 5 time constants.
  cutoff_time_ = 5.0f * time_constant_;

  RCLCPP_INFO(
    logger_,
    "DirectionChangeCritic instantiated with %d power, %f weight, "
    "%f time_constant, %f motion_threshold.",
    power_, weight_, time_constant_, motion_threshold_);
}

void DirectionChangeCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_) {
    return;
  }

  // Update persistent, real (non-rollout) direction-tracking state once per
  // real control cycle, using the robot's actual measured velocity (NOT any
  // candidate trajectory) -- score() is called once per real control cycle
  // (controller_frequency == 1/model_dt in this deployment), not once per
  // candidate trajectory.
  const float current_vx = static_cast<float>(data.state.speed.linear.x);
  float current_sign = 0.0f;
  if (current_vx > motion_threshold_) {
    current_sign = 1.0f;
  } else if (current_vx < -motion_threshold_) {
    current_sign = -1.0f;
  }

  if (current_sign != 0.0f) {
    if (current_sign == last_sign_) {
      time_since_reversal_ += data.model_dt;
    } else {
      // Fresh start from a stop, or an actual reversal -- both restart the
      // "how long have we been committed to this direction" clock.
      time_since_reversal_ = 0.0f;
      last_sign_ = current_sign;
    }
  }
  // current_sign == 0 (robot effectively stationary, within
  // motion_threshold_): leave last_sign_ and time_since_reversal_ untouched
  // -- momentum neither grows nor resets while genuinely stopped.

  if (last_sign_ == 0.0f || time_since_reversal_ >= cutoff_time_) {
    // No established direction yet (very first cycles), or fully decayed --
    // no penalty.
    return;
  }

  const float decay = std::exp(-time_since_reversal_ / time_constant_);

  // Candidate rollout steps proposing motion opposite to last_sign_. Folded
  // into a single expression (-last_sign_ * vx is -vx when last_sign_=+1,
  // vx when last_sign_=-1) rather than a ternary between xt::maximum(-vx, 0)
  // and xt::maximum(vx, 0) -- those are different xtensor lazy-expression
  // template types (one wraps a negate xfunction, one doesn't), so `auto`
  // can't deduce a single type across a ternary between them (compile
  // error, confirmed live).
  auto opposing = xt::maximum(-last_sign_ * data.state.vx, 0);

  if (power_ > 1u) {
    data.costs += xt::pow(
      xt::sum(std::move(opposing) * data.model_dt, {1}, immediate) * weight_ * decay,
      power_);
  } else {
    data.costs += xt::sum(std::move(opposing) * data.model_dt, {1}, immediate) * weight_ * decay;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::DirectionChangeCritic,
  mppi::critics::CriticFunction)
