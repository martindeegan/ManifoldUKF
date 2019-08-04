#ifndef EXTRINSICS_HPP
#define EXTRINSICS_HPP

#include <yaml-cpp/yaml.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <State.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
/**
 * @class Represents a relative pose of a sensor with respect to the IMU.
 *
 * State state of our quadcopter is represented by the state of our IMU. In order to correctly add
 * corrections to the state, our sensor model must take into account its relative pose.
 */
class Extrinsics
{
public:
    explicit Extrinsics(YAML::Node config);

    /**
     * @brief Relative pose of the sensor to the IMU
     */
    const Sophus::SE3d& pose() const;

    /**
     * @brief Relative attitude of the sensor to the IMU
     */
    const Sophus::SO3d& rotation() const;

    /**
     * @brief Relative position of the sensor to the IMU
     */
    const Eigen::Vector3d& position() const;

    /**
     * @brief Computes the state (world frame) of the sensor given the state of the IMU
     * @param state IMU state
     */
    State operator()(const State& state) const;

private:
    const Sophus::SE3d::Tangent twist_;
    const Sophus::SE3d pose_;
    const Sophus::SO3d rot_;
    const Eigen::Vector3d pos_;
};
}  // namespace kalman
}  // namespace gnc
}  // namespace maav

#endif
