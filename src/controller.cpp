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

#include <string>
#include <memory>
#include <fins/node.hpp>

#include "finenav_mppi_controller/models/constraints.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

#include "finenav_mppi_controller/optimizer.hpp"
#include "finenav_mppi_controller/map_view_def.hpp"

namespace finenav_mppi_controller
{

using namespace mppi;

class MPPIControllerNode : public fins::Node
{
public:
    void define() override {
        set_name("MPPIControllerNode");
        set_description("Standalone MPPI Controller Node");
        set_category("Navigation");

        register_input<nav_msgs::msg::Path>("reference_path", &MPPIControllerNode::onReferencePath);
        register_input<geometry_msgs::msg::PoseStamped>("current_pose", &MPPIControllerNode::onCurrentPose);
        register_input<geometry_msgs::msg::Twist>("current_velocity", &MPPIControllerNode::onCurrentVelocity);

        register_output<geometry_msgs::msg::Twist>("cmd_vel");
        register_output<nav_msgs::msg::Path>("optimal_path");

        map_view_.isTrackingUnknown = register_client<bool(void)>("is_tracking_unknown");
        map_view_.considerFootprint = register_client<bool(void)>("consider_footprint");
        map_view_.isCollision       = register_client<bool(float, float, float)>("is_collision");
        map_view_.getRadius         = register_client<float(void)>("get_radius");
        map_view_.getCost           = register_client<int(const Position3D&)>("get_cost");
        map_view_.costAtPose        = register_client<float(float, float, float)>("cost_at_pose");
        map_view_.getBaseFrameID    = register_client<std::string(void)>("get_base_frame_id");
    }

    void initialize() override {
        initCritics();
        optimizer_.initialize("MPPIController", critics_);
    }

    void run() override {
        logger->info("MPPI Controller running.");
    }

    void pause() override {}
    void reset() override {
        optimizer_.reset();
    }

private:
    void onReferencePath(const nav_msgs::msg::Path& path) {
        reference_path_ = path;
    }

    void onCurrentVelocity(const geometry_msgs::msg::Twist& vel) {
        current_velocity_ = vel;
    }

    void onCurrentPose(const geometry_msgs::msg::PoseStamped& pose) {
        current_pose_ = pose;

        if (reference_path_.poses.empty()) {
            return;
        }

        optimizer_.evalControl(current_pose_, current_velocity_, reference_path_, map_view_);

        geometry_msgs::msg::Twist cmd_vel = getImmediateCommand();
        send("cmd_vel", cmd_vel);

        nav_msgs::msg::Path opt_path = getPredictedPath();
        send("optimal_path", opt_path);
    }


    void initCritics() {
        critics_.push_back(std::make_unique<critics::ConstraintCritic>());
        critics_.push_back(std::make_unique<critics::CostCritic>());
        critics_.push_back(std::make_unique<critics::GoalCritic>());
        critics_.push_back(std::make_unique<critics::ObstaclesCritic>());
        critics_.push_back(std::make_unique<critics::PathAlignCritic>());
        critics_.push_back(std::make_unique<critics::PathAlignLegacyCritic>());
        critics_.push_back(std::make_unique<critics::PathAngleCritic>());
        critics_.push_back(std::make_unique<critics::PathFollowCritic>());
        critics_.push_back(std::make_unique<critics::PreferForwardCritic>());
        critics_.push_back(std::make_unique<critics::TwirlingCritic>());
        critics_.push_back(std::make_unique<critics::VelocityDeadbandCritic>());
    }

    geometry_msgs::msg::Twist getImmediateCommand() {
        auto control_seq = optimizer_.getControlSequence();
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = control_seq(0, 0);
        cmd.angular.z = control_seq(0, 1);
        cmd.linear.y  = control_seq(0, 2);
        return cmd;
    }

    nav_msgs::msg::Path getPredictedPath() {
        auto optimized_trajectory = optimizer_.getOptimizedTrajectory();
        nav_msgs::msg::Path path;
        path.header.frame_id = map_view_.getBaseFrameID();
        path.header.stamp = rclcpp::Clock().now();

        const size_t n = optimized_trajectory.shape(0);
        path.poses.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = optimized_trajectory(i, 0);
            ps.pose.position.y = optimized_trajectory(i, 1);
            
            const float yaw = optimized_trajectory(i, 2);
            ps.pose.orientation.z = std::sin(yaw * 0.5f);
            ps.pose.orientation.w = std::cos(yaw * 0.5f);
            path.poses.push_back(ps);
        }
        return path;
    }

private:
    nav_msgs::msg::Path reference_path_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Twist current_velocity_;

    Optimizer optimizer_;
    std::vector<std::unique_ptr<critics::CriticFunction>> critics_;
    IMapView map_view_;
};

EXPORT_NODE(MPPIControllerNode)

} // namespace mppi

DEFINE_PLUGIN_ENTRY(fins::STATELESS);