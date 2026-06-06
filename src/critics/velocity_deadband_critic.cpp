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

#include "finenav_mppi_controller/critics/velocity_deadband_critic.hpp"

namespace mppi::critics
{
using namespace finenav_mppi_controller;

void VelocityDeadbandCritic::initialize()
{
  fins::ParamLoader config("finenav_mppi_controller.VelocityDeadbandCritic");
  enabled_ = config.get("enabled", false)
                   .with_description("Enable VelocityDeadbandCritic");
  power_   = config.get("cost_power", 1)
                   .with_description("Power exponent applied to cost");
  weight_  = config.get("cost_weight", 35.0f)
                   .with_description("Weight for velocity deadband cost");

  auto dv = config.get("deadband_velocities", std::vector<double>{0.05, 0.05, 0.05})
                  .with_description("Deadband thresholds [vx, vy, wz]; velocities below are penalized");
  deadband_velocities_.resize(3);
  for (size_t i = 0; i < 3 && i < dv.size(); ++i) {
    deadband_velocities_[i] = static_cast<float>(dv[i]);
  }
  
//  RCLCPP_INFO_STREAM(
//    logger_, "VelocityDeadbandCritic instantiated with "
//      << power_ << " power, " << weight_ << " weight, deadband_velocity ["
//      << deadband_velocities_.at(0) << "," << deadband_velocities_.at(1) << ","
//      << deadband_velocities_.at(2) << "]");
}


void VelocityDeadbandCritic::score(CriticData & data, const IMapView& map_view)
{
  using xt::evaluation_strategy::immediate;

  if (!this->enabled_) {
    return;
  }

  auto & vx = data.state.vx;
  auto & wz = data.state.wz;

  if (data.motion_model->isHolonomic()) {
    auto & vy = data.state.vy;
    if (power_ > 1u) {
      data.costs += xt::pow(
        xt::sum(
          std::move(
            xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
            xt::maximum(fabs(deadband_velocities_.at(1)) - xt::fabs(vy), 0) +
            xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0)) *
          data.model_dt,
          {1}, immediate) *
        weight_,
        power_);
    } else {
      data.costs += xt::sum(
        (std::move(
          xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
          xt::maximum(fabs(deadband_velocities_.at(1)) - xt::fabs(vy), 0) +
          xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0))) *
        data.model_dt,
        {1}, immediate) *
        weight_;
    }
    return;
  }

  if (power_ > 1u) {
    data.costs += xt::pow(
      xt::sum(
        std::move(
          xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
          xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0)) *
        data.model_dt,
        {1}, immediate) *
      weight_,
      power_);
  } else {
    data.costs += xt::sum(
      (std::move(
        xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
        xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0))) *
      data.model_dt,
      {1}, immediate) *
      weight_;
  }
  return;
}

}  // namespace mppi::critics

