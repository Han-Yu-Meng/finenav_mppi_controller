// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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
#include "finenav_mppi_controller/critics/cost_critic.hpp"


namespace mppi::critics
{
using namespace finenav_mppi_controller;

void CostCritic::initialize()
{
  fins::ParamLoader config("finenav_mppi_controller.CostCritic");
  enabled_ = config.get("enabled", true)
                   .with_description("Enable CostCritic (uses costmap inflation cost)");
  power_   = config.get("cost_power", 2)
                   .with_description("Power exponent applied to cost");
  weight_  = config.get("cost_weight", 3.81f)
                   .with_description("Weight for costmap cost (internally normalized by 254)");
  critical_cost_ = config.get("critical_cost", 300.0f)
                   .with_description("Penalty added when pose cost >= 253 (near-lethal)");
  collision_cost_ = config.get("collision_cost", 1000000.0f)
                   .with_description("Penalty for trajectories in collision");
  near_goal_distance_ = config.get("near_goal_distance", 1.0f)
                   .with_description("Distance from goal where repulsion is disabled (m)");
  // consider_footprint...

  // Normalized by cost value to put in same regime as other weights
  weight_ /= 254.0f;
}



void CostCritic::score(CriticData & data,const IMapView& map_view)
{
  using xt::evaluation_strategy::immediate;
  if (!this->enabled_) {
    return;
  }


  // If near the goal, don't apply the preferential term since the goal is near obstacles
  bool near_goal = false;
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
    near_goal = true;
  }

  auto && repulsive_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  repulsive_cost.fill(0.0);

  const size_t traj_len = data.trajectories.x.shape(1);
  bool all_trajectories_collide = true;
  for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
    bool trajectory_collide = false;
    const auto & traj = data.trajectories;
    float pose_cost;

    for (size_t j = 0; j < traj_len; j++) {
      // The costAtPose doesn't use orientation
      // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
      // So the center point has more information than the footprint
      pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), map_view);
      if (pose_cost < 1.0f) {continue;}  // In free space

      if (inCollision(traj.x(i, j), traj.y(i, j), traj.yaws(i, j), map_view)) {
        trajectory_collide = true;
        break;
      }

      // Let near-collision trajectory points be punished severely
      // Note that we collision check based on the footprint actual,
      // but score based on the center-point cost regardless
      if (pose_cost >= 253) {
        repulsive_cost[i] += critical_cost_;
      } else if (!near_goal) {  // Generally prefer trajectories further from obstacles
        repulsive_cost[i] += pose_cost;
      }
    }

    if (!trajectory_collide) {
      all_trajectories_collide = false;
    } else {
      repulsive_cost[i] = std::numeric_limits<float>::max();
    }
  }

  data.costs += xt::pow((weight_ * repulsive_cost / traj_len), power_);
  data.fail_flag = all_trajectories_collide;
  // RCLCPP_DEBUG(logger_, "Scored");
}


}  // namespace mppi::critics

