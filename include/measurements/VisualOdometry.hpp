#pragma once

#include <Eigen/Dense>
#include <sophus/so3.hpp>

namespace manifold_ukf
{
namespace gnc
{
namespace measurements
{
/*
 * Stores the necessary info we need from a visual odometry measurement
 */
struct VisualOdometryMeasurement
{
    Eigen::Vector3d translation;
    Sophus::SO3d rotation;
    uint64_t time_usec;
};

}  // namespace measurements
}  // namespace gnc
}  // namespace manifold_ukf
