
#include <iostream>

#include <kalman/Extrinsics.hpp>
#include <yaml_matrix/yaml_matrix.hpp>

namespace manifold_ukf
{
namespace gnc
{
namespace kalman
{
Extrinsics::Extrinsics(YAML::Node config)
    : twist_(config.as<Sophus::SE3d::Tangent>()),
      pose_(Sophus::SE3d::exp(twist_)),
      rot_(pose_.so3()),
      pos_(pose_.translation())
{
}

State Extrinsics::operator()(const State& state) const
{
    State sensor_state = state;
    sensor_state.setPose(state.getPose() * pose_);
    return sensor_state;
}

const Sophus::SE3d& Extrinsics::pose() const { return pose_; }
const Sophus::SO3d& Extrinsics::rotation() const { return rot_; }
const Eigen::Vector3d& Extrinsics::position() const { return pos_; }
}  // namespace kalman
}  // namespace gnc
}  // namespace manifold_ukf