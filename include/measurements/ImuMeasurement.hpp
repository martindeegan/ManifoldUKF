#pragma once

#include <Eigen/Dense>

namespace manifold_ukf
{
namespace gnc
{
namespace measurements
{
/*
 * Stores the necessary info we need from an IMU message.
 */
struct ImuMeasurement
{
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angular_rates;
    Eigen::Vector3d magnetometer;
    uint64_t time_usec;
};

std::ostream& operator<<(std::ostream& os, const ImuMeasurement& meas);

}  // namespace measurements
}  // namespace gnc
}  // namespace manifold_ukf
