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
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <nav_msgs/msg/odometry.hpp>
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
    MPPIControllerNode() : stop_thread_(false), mppi_frequency_(10.0) {}
    ~MPPIControllerNode() {
        stop_thread_ = true;
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    void define() override {
        set_name("MPPIControllerNode");
        set_description("Standalone MPPI Controller Node");
        set_category("Navigation");

        register_input<nav_msgs::msg::Path>("reference_path", &MPPIControllerNode::onReferencePath);
        register_input<nav_msgs::msg::Odometry>("odom", &MPPIControllerNode::onOdometry);

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

        fins::ParamLoader config("finenav_mppi_controller");
        mppi_frequency_ = config.get("freq", 10);
        
        stop_thread_ = false;
        worker_thread_ = std::thread(&MPPIControllerNode::controlLoop, this);
    }

    void run() override {
        logger->info("MPPI Controller running at {} Hz.", mppi_frequency_);
    }

    void pause() override {}
    void reset() override {
        std::lock_guard<std::mutex> lock(data_mutex_);
        optimizer_.reset();
    }

private:
    void onReferencePath(const nav_msgs::msg::Path& path) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        reference_path_ = path;
    }

    void onOdometry(const nav_msgs::msg::Odometry& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_pose_.header = msg.header;
        current_pose_.pose = msg.pose.pose;
        current_velocity_ = msg.twist.twist;
    }

    void controlLoop() {
        while (!stop_thread_) {
            auto start_time = std::chrono::steady_clock::now();
            
            bool has_path = false;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                has_path = !reference_path_.poses.empty();
            }

            if (has_path) {
                nav_msgs::msg::Path ref_path;
                geometry_msgs::msg::PoseStamped curr_pose;
                geometry_msgs::msg::Twist curr_vel;
                
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    ref_path = reference_path_;
                    curr_pose = current_pose_;
                    curr_vel = current_velocity_;
                }

                auto start = std::chrono::steady_clock::now();
                optimizer_.evalControl(curr_pose, curr_vel, ref_path, map_view_);
                auto end = std::chrono::steady_clock::now();
                std::chrono::duration<double, std::milli> duration = end - start;
                logger->info("MPPI calculation time: {} ms", duration.count());

                geometry_msgs::msg::Twist cmd_vel = getImmediateCommand();
                send("cmd_vel", cmd_vel);

                nav_msgs::msg::Path opt_path = getPredictedPath();
                send("optimal_path", opt_path);
            }

            auto end_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            auto period = std::chrono::microseconds(static_cast<long long>(1000000.0 / mppi_frequency_));
            
            if (elapsed < period) {
                std::this_thread::sleep_for(period - elapsed);
            }
        }
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

    double mppi_frequency_;
    std::thread worker_thread_;
    std::atomic<bool> stop_thread_;
    std::mutex data_mutex_;
};

EXPORT_NODE(MPPIControllerNode)

} // namespace mppi

DEFINE_PLUGIN_ENTRY(fins::STATELESS);