#include <kalman/updates/LidarUpdate.hpp>

using manifold_ukf::gnc::measurements::LidarMeasurement;

namespace manifold_ukf
{
namespace gnc
{
namespace kalman
{
LidarUpdate::LidarUpdate(YAML::Node config)
    : BaseUpdate(config["lidar"]),
      bias_(config["lidar"]["lidar_bias"].as<double>()),
      imu_height_(config["imu_height"].as<double>())
{
}

LidarMeasurement LidarUpdate::predicted(const State& state)
{
    /*
        Here, we predict the length of a lidar beam given our state. With some trigonometry, you can
       observe that the length of a beam going down is -z / cos(theta). We rotate a unit z vector
       and dot it with the unit z vector to find cos(theta)
    */
    LidarMeasurement predicted_measurement;
    const Eigen::Vector3d vertical_vec = Eigen::Vector3d::UnitZ();
    double cos_theta = (state.attitude() * vertical_vec).dot(vertical_vec);
    predicted_measurement.distance()(0) = -state.position().z() / cos_theta + bias_ + imu_height_ -
                                          BaseUpdate::extrinsics_.position().z();
    // std::cout << "Pred lid: " << predicted_measurement.distance() << std::endl;

    return predicted_measurement;
}

LidarMeasurement LidarUpdate::measured(const measurements::Measurement& meas)
{
    return *(meas.lidar);
}

void LidarUpdate::operator()(History::Snapshot& snapshot)
{
    // Check the validity of the lidar measurements
    if (!snapshot.measurement.lidar) return;
    double distance = snapshot.measurement.lidar->distance()(0);
    if (std::isfinite(distance) && distance >= 0 && distance <= 1.25)
    {
        correct(snapshot);
    }
}
}  // namespace kalman
}  // namespace gnc
}  // namespace manifold_ukf