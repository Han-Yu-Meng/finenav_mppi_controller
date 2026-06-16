#pragma once

#include <Eigen/Core>
#include <fins/node.hpp>

using Position3D = Eigen::Vector3d;

struct RollingGridData {
    std::vector<uint8_t> grid;
    int map_size = 0;
    float resolution = 0.0f;
    float origin_x = 0.0f;
    float origin_y = 0.0f;
    bool is_tracking_unknown = false;
    bool consider_footprint = false;
    float l_half = 0.0f;
    float w_half = 0.0f;
    float robot_radius = 0.0f;
};

namespace finenav_mppi_controller {

    class IMapView {
    public:
        fins::Client<RollingGridData(void)> getRollingGridData_;
        fins::Client<std::string(void)> getBaseFrameID_;

        RollingGridData cached_data;
        std::string cached_base_frame_id;

        void updateCache() {
            cached_data = getRollingGridData_();
            cached_base_frame_id = getBaseFrameID_();
        }

        bool isTrackingUnknown() const { return cached_data.is_tracking_unknown; }
        bool considerFootprint() const { return cached_data.consider_footprint; }
        float getRadius() const { return cached_data.robot_radius; }
        std::string getBaseFrameID() const { return cached_base_frame_id; }

        int getCost(const Position3D& pos) const {
            float x = static_cast<float>(pos.x());
            float y = static_cast<float>(pos.y());
            if (!isInside(x, y)) {
                return cached_data.is_tracking_unknown ? 0 : 255;
            }
            return cached_data.grid[getIndexY(y) * cached_data.map_size + getIndexX(x)];
        }

        bool isCollision(float x, float y, float theta) const {
            return checkCollisionFootprint(x, y, theta);
        }

        float costAtPose(float x, float y, float theta) const {
            int max_cost = 0;
            const auto pts = getFootprintPoints(x, y, theta);
            for (const auto& pt : pts) {
                if (isInside(pt.first, pt.second)) {
                    int ix = getIndexX(pt.first);
                    int iy = getIndexY(pt.second);
                    max_cost = std::max(max_cost, static_cast<int>(cached_data.grid[iy * cached_data.map_size + ix]));
                }
            }
            return static_cast<float>(max_cost);
        }

    private:
        inline bool isInside(float x, float y) const {
            float dx = x - cached_data.origin_x;
            float dy = y - cached_data.origin_y;
            float max_range = static_cast<float>(cached_data.map_size / 2) * cached_data.resolution;
            return (std::abs(dx) < max_range && std::abs(dy) < max_range);
        }

        inline int getIndexX(float x) const {
            int map_half_size = cached_data.map_size / 2;
            float inv_res = 1.0f / cached_data.resolution;
            return std::clamp(static_cast<int>((x - cached_data.origin_x) * inv_res) + map_half_size, 0, cached_data.map_size - 1);
        }

        inline int getIndexY(float y) const {
            int map_half_size = cached_data.map_size / 2;
            float inv_res = 1.0f / cached_data.resolution;
            return std::clamp(static_cast<int>((y - cached_data.origin_y) * inv_res) + map_half_size, 0, cached_data.map_size - 1);
        }

        bool checkCollisionFootprint(float x, float y, float theta) const {
            const auto pts = getFootprintPoints(x, y, theta);
            for (const auto& pt : pts) {
                if (!isInside(pt.first, pt.second)) {
                    if (!cached_data.is_tracking_unknown) return true;
                    continue;
                }
                int ix = getIndexX(pt.first);
                int iy = getIndexY(pt.second);
                if (cached_data.grid[iy * cached_data.map_size + ix] >= 253) {
                    return true;
                }
            }
            return false;
        }

        std::array<std::pair<float, float>, 5> getFootprintPoints(float x, float y, float theta) const {
            const std::array<std::pair<float, float>, 5> offsets = {{
                {0.0f, 0.0f},
                {cached_data.l_half, cached_data.w_half}, {cached_data.l_half, -cached_data.w_half},
                {-cached_data.l_half, cached_data.w_half}, {-cached_data.l_half, -cached_data.w_half}
            }};

            float cos_t = std::cos(theta);
            float sin_t = std::sin(theta);

            std::array<std::pair<float, float>, 5> pts;
            for (size_t i = 0; i < 5; ++i) {
                pts[i].first  = offsets[i].first * cos_t - offsets[i].second * sin_t + x;
                pts[i].second = offsets[i].first * sin_t + offsets[i].second * cos_t + y;
            }
            return pts;
        }
    };

}