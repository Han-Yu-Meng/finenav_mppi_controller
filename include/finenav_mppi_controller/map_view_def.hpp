#pragma once

#include <Eigen/Core>

using Position3D = Eigen::Vector3d;
namespace finenav_mppi_controller {
    class IMapView {
    public:
      std::function<bool(void)> isTrackingUnknown;
      std::function<bool(void)> considerFootprint;
      std::function<bool(float, float, float)> isCollision;
      std::function<float(void)> getRadius;
      std::function<int(const Position3D&)> getCost;
      std::function<float(float, float, float)> costAtPose;
      std::function<std::string(void)> getBaseFrameID;
    }
}