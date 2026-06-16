#pragma once

#include <Eigen/Core>
#include <fins/node.hpp>

using Position3D = Eigen::Vector3d;
namespace finenav_mppi_controller {
    class IMapView {
    public:
      fins::Client<bool(void)> isTrackingUnknown;
      fins::Client<bool(void)> considerFootprint;
      fins::Client<bool(float, float, float)> isCollision;
      fins::Client<float(void)> getRadius;
      fins::Client<int(const Position3D&)> getCost;
      fins::Client<float(float, float, float)> costAtPose;
      fins::Client<std::string(void)> getBaseFrameID;
    };
}