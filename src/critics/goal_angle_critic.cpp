// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "finenav_mppi_controller/critics/goal_angle_critic.hpp"

namespace mppi::critics
{
using namespace finenav_mppi_controller;

void GoalAngleCritic::initialize()
{
  fins::ParamLoader config("finenav_mppi_controller.GoalAngleCritic");
  enabled_ = config.get("enabled", true)
                   .with_description("Enable GoalAngleCritic");
  power_   = config.get("cost_power", 1)
                   .with_description("Power exponent applied to cost");
  weight_  = config.get("cost_weight", 5.0f)
                   .with_description("Weight for goal heading alignment cost");
  threshold_to_consider_ = config.get("threshold_to_consider", 1.4f)
                   .with_description("Distance from goal at which critic activates (m)");
}

void GoalAngleCritic::score(CriticData & data, const IMapView& map_view)
{
  if (!this->enabled_ )    //TODO:旧有的检查与距离的逻辑会触发，为什么
  {
    return;
  }

  const auto goal_idx = data.path.x.shape(0) - 1;
  const float goal_yaw = data.path.yaws(goal_idx);

  data.costs += xt::pow(
    xt::mean(xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw)), {1}) *
    weight_, power_);
  // RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics

