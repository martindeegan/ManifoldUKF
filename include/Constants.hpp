#pragma once

#include <cstdint>

#include <Eigen/Dense>

namespace maav
{
namespace gnc
{
namespace constants
{
/**
 * Nominal average acceleration from earths gravity
 */
constexpr double STANDARD_GRAVITY = 9.80665;

constexpr double SEC_TO_USEC = 1000000.0;
constexpr double USEC_TO_SEC = 1.0 / SEC_TO_USEC;

constexpr double DEG_TO_RAD = M_PI / 180;
constexpr double RAD_TO_DEG = 1 / DEG_TO_RAD;

const Eigen::Vector3d ANN_ARBOR_MAGNETIC_FIELD = {18949.7, -2331.2, 49992.7};

}  // namespace constants
}  // namespace gnc
}  // namespace maav
