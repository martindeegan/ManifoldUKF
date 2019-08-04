#ifndef LIDAR_UPDATE_HPP
#define LIDAR_UPDATE_HPP

#include <yaml-cpp/yaml.h>

#include <kalman/BaseUpdate.hpp>
#include <measurements/LidarMeasurement.hpp>

namespace manifold_ukf
{
namespace gnc
{
namespace kalman
{
/**
 * A correction step for a downward facing lidar on a flat surface
 */
class LidarUpdate : public BaseUpdate<measurements::LidarMeasurement>
{
public:
    /**
     *  @param config This yaml node must have a "lidar" key with unscented transform parameters and
     * a sensor covariance matrix, R
     */
    LidarUpdate(YAML::Node config);

    /**
     * @breif Computes the observation model (h(x)) for a state provided by an UnscentedTransform
     * @param state A sigma point proved by an UnscentedTransform
     * @return The predicted lidar observation
     */
    measurements::LidarMeasurement predicted(const State& state);

    /**
     * @param meas
     * @return The relevant lidar measurement from the list of measurements
     */
    measurements::LidarMeasurement measured(const measurements::Measurement& meas);

    /**
     * @brief Performs the correction step for a lidar
     * @param snapshot A mutable reference to a point in time. The state will be updated according
     * to the sensor model
     */
    void operator()(History::Snapshot& snapshot);

private:
    using BaseUpdate<measurements::LidarMeasurement>::correct;

    double bias_;
    double imu_height_;
};
}  // namespace kalman
}  // namespace gnc
}  // namespace manifold_ukf

#endif