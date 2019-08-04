#ifndef GLOBAL_UPDATE_HPP
#define GLOBAL_UPDATE_HPP

#include <yaml-cpp/yaml.h>
#include <sophus/se3.hpp>

#include <kalman/BaseUpdate.hpp>
#include <measurements/GlobalUpdateMeasurement.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
/**
 * A correction step for a downward facing lidar on a flat surface
 */
class GlobalUpdate : public BaseUpdate<measurements::GlobalUpdateMeasurement>
{
public:
    /**
     *  @param config This yaml node must have a "lidar" key with unscented transform parameters and
     * a sensor covariance matrix, R
     */
    GlobalUpdate(YAML::Node config);

    /**
     * @breif Computes the observation model (h(x)) for a state provided by an UnscentedTransform
     * @param state A sigma point proved by an UnscentedTransform
     * @return The predicted lidar observation
     */
    measurements::GlobalUpdateMeasurement predicted(const State& state);

    /**
     * @param meas
     * @return The relevant lidar measurement from the list of measurements
     */
    measurements::GlobalUpdateMeasurement measured(const measurements::Measurement& meas);

    /**
     * @brief Performs the correction step for a lidar
     * @param snapshot A mutable reference to a point in time. The state will be updated according
     * to the sensor model
     */
    void operator()(History::Snapshot& snapshot);

private:
    using BaseUpdate<measurements::GlobalUpdateMeasurement>::correct;

    bool initialized_pose_;
    Sophus::SE3d starting_pose_;
    Sophus::SE3d starting_pose_inverse_;
};
}  // namespace kalman
}  // namespace gnc
}  // namespace maav

#endif
