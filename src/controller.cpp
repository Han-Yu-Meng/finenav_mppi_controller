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

/**
 * @class mppi::MPPIController
 * @brief Main plugin controller for MPPI Controller
 */
class MPPIController : public fins::Node
{
public:

    using IMapView = finenav_mppi_controller::IMapView;
    using Context = finenav::PlanningContext<MPPIController>;

    void define() override {
        set_name("MPPIController");
        set_description("MPPI Controller");
        set_category("FineNav>Planners");

        map_view_.isTrackingUnknown = registerer_client<bool(void)>("is_tracking_unknown");
        map_view_.considerFootprint = registerer_client<bool(void)>("consider_footprint");
        map_view_.isCollision = registerer_client<bool(float, float, float)>("is_collision");
        map_view_.getRadius = registerer_client<float(void)>("get_radius");
        map_view_.getCost = registerer_client<int(const Position3D&)>("get_cost");
        map_view_.costAtPose = registerer_client<float(float, float, float)>("cost_at_pose");
        map_view_.getBaseFrameID = registerer_client<std::string(void)>("get_base_frame_id");
    }

    void initialize() override { 
        registerCritic(std::make_unique<critics::ConstraintCritic>());
        registerCritic(std::make_unique<critics::CostCritic>());
        registerCritic(std::make_unique<critics::GoalAngleCritic>());
        registerCritic(std::make_unique<critics::GoalCritic>());
        registerCritic(std::make_unique<critics::ObstaclesCritic>());
        registerCritic(std::make_unique<critics::PathAlignCritic>());
        registerCritic(std::make_unique<critics::PathAlignLegacyCritic>());
        registerCritic(std::make_unique<critics::PathAngleCritic>());
        registerCritic(std::make_unique<critics::PathFollowCritic>());
        registerCritic(std::make_unique<critics::PreferForwardCritic>());
        registerCritic(std::make_unique<critics::TwirlingCritic>());
        registerCritic(std::make_unique<critics::VelocityDeadbandCritic>());

        std::cout<<"[finenav_mppi_controller] Critics registered. Initializing optimizer..."<<std::endl;
        std::cout<<"[finenav_mppi_controller] Number of critics: "<<critics_.size()<<std::endl;

        optimizer_.initialize("MPPIController", critics_);
    }

     finenav::DataPacket<OutputProfile> plan(const finenav::PlanningContext<MPPIController>& ctx) {

        finenav::DataPacket<OutputProfile> traj;
        // 从框架注入的 RobotState 中取出 pose 和 twist，无需 TF 查询
        const auto& robot_state = ctx.robot_state;
        auto goal_pose = ctx.goal_pose;

        nav_msgs::msg::Path path = convertDataPacketToPath(ctx.ref_traj, "map");
        this->setPlan(path);

        geometry_msgs::msg::PoseStamped start_pose_stamped;
        start_pose_stamped.pose = robot_state.pose;
        start_pose_stamped.header.stamp = rclcpp::Clock().now();
        start_pose_stamped.header.frame_id = "map";

        computeVelocityCommands(
            start_pose_stamped,
            robot_state.twist,  // 直接使用当前实际速度
            map_view_
        );
        // 装填输出轨迹

        // 获取最优轨迹
        auto optimized_trajectory = optimizer_.getOptimizedTrajectory();
        auto control_seq = optimizer_.getControlSequence();
        // 填充轨迹点 (pose + twist)
        const size_t n = optimized_trajectory.shape(0);
        traj.poses.reserve(n);
        traj.twists.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            // pose: x, y, yaw -> quaternion
            geometry_msgs::msg::Pose pose;
            pose.position.x = optimized_trajectory(i, 0);
            pose.position.y = optimized_trajectory(i, 1);
            pose.position.z = 0.0;
            const float yaw = optimized_trajectory(i, 2);
            pose.orientation.z = std::sin(yaw * 0.5f);
            pose.orientation.w = std::cos(yaw * 0.5f);
            traj.poses.push_back(pose);

            // twist: vx, vy, wz from control sequence
            geometry_msgs::msg::Twist twist;
            twist.linear.x  = control_seq(i, 0);
            twist.angular.z = control_seq(i, 1);
            twist.linear.y  = control_seq(i, 2);
            traj.twists.push_back(twist);
        }

        return traj;
    }


    /**
        * @brief Set new reference path to track
        * @param path Path to track
        */
    void setPlan(const nav_msgs::msg::Path & path)
    {
        path_ = path;
    }

    /**
        * @brief Set new speed limit from callback
        * @param speed_limit Speed limit to use
        * @param percentage Bool if the speed limit is absolute or relative
        */
    void setSpeedLimit(const double & speed_limit, const bool & percentage)
    {
        optimizer_.setSpeedLimit(speed_limit, percentage);
    }

    /**
    * @brief Reset optimizer state at Episode boundary (new goal).
    *        Called by ControlLayer whenever a new Action Goal arrives.
    *        Clears control_sequence_ so the next plan() call starts from
    *        a zero warm-start, independent of any previous planning direction.
    */
    void reset() {
        optimizer_.reset();
    }

    void registerCritic(std::unique_ptr<critics::CriticFunction> critic) {
        critics_.push_back(std::move(critic));
    }


    template <typename DataPacketT>
    nav_msgs::msg::Path convertDataPacketToPath(
        const DataPacketT & data,
        const std::string & frame_id)
    {
        nav_msgs::msg::Path path;

        path.header.frame_id = frame_id;
        path.header.stamp = rclcpp::Clock().now();

        if constexpr (requires { data.poses; }) {

            path.poses.reserve(data.poses.size());

            for (const auto & pose : data.poses) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header = path.header;
                ps.pose = pose;

                path.poses.push_back(ps);
            }
        }

        return path;
    }


protected:
    /**
    * @brief Visualize trajectories
    * @param transformed_plan Transformed input plan
    */
//    void visualize(
//        nav_msgs::msg::Path transformed_plan,
//        const builtin_interfaces::msg::Time & cmd_stamp,
//        double z_height);

    /**
    * @brief Main method to compute velocities using the optimizer
    * @param robot_pose Robot pose
    * @param robot_speed Robot speed
    * @param goal_checker Pointer to the goal checker for awareness if completed task
    */
    void computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & robot_pose,
          const geometry_msgs::msg::Twist & robot_speed)
    {
        optimizer_.evalControl(robot_pose, robot_speed, path_ ,map_view_);
    }

    nav_msgs::msg::Path path_;
    Optimizer optimizer_;
    std::vector<std::unique_ptr<critics::CriticFunction>> critics_;
    IMapView map_view_;

    bool visualize_;
};

}