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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__DIRECTION_CHANGE_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__DIRECTION_CHANGE_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::DirectionChangeCritic
 * @brief Critic objective function that discourages commanding velocity in
 * the opposite direction from the robot's recently-established direction of
 * travel (penalizes "back-and-forth" wiggling / premature reversals), with
 * an exponentially-decaying weight: strong immediately after establishing a
 * direction (fresh start from a stop, or an actual reversal), decaying to
 * ~0 once the robot has been committed to that direction for a few seconds
 * (time_constant_ controls the decay rate -- see score()).
 *
 * Tracks the robot's actual measured direction (CriticData::state.speed),
 * not a candidate rollout's direction -- this is real, persistent,
 * cross-cycle state carried on the critic instance itself, updated once per
 * real control cycle (score() is called once per cycle, not once per
 * candidate trajectory).
 */
class DirectionChangeCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost for commanding a trajectory that opposes the
   * robot's recently-established direction of travel.
   *
   * @param costs [out] add direction-change cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float time_constant_{0};
  float motion_threshold_{0};
  float cutoff_time_{0};

  // Persistent, real (non-rollout) state -- the robot's established
  // direction and how long it's been held. Updated once per real control
  // cycle in score(), using CriticData::state.speed (the robot's actual
  // measured velocity) and CriticData::model_dt as the real cycle period
  // (true in this deployment: controller_frequency == 1/model_dt).
  float last_sign_{0.0f};
  float time_since_reversal_{0.0f};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__DIRECTION_CHANGE_CRITIC_HPP_
