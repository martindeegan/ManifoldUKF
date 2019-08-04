#include <kalman/updates/GlobalUpdate.hpp>

using manifold_ukf::gnc::measurements::GlobalUpdateMeasurement;

namespace manifold_ukf
{
namespace gnc
{
namespace kalman
{
GlobalUpdate::GlobalUpdate(YAML::Node config) : BaseUpdate(config["global_update"]) {}
GlobalUpdateMeasurement GlobalUpdate::predicted(const State& state)
{
    if (!initialized_pose_)
    {
        starting_pose_ = state.getPose();
        starting_pose_inverse_ = starting_pose_.inverse();
        initialized_pose_ = true;
    }

    Sophus::SE3d pose = starting_pose_inverse_ * state.getPose();

    /*
    Here, we predict the position and attitude of our quadcopter
    */

    GlobalUpdateMeasurement predicted_measurement;
    predicted_measurement.pose() = pose;

    return predicted_measurement;
}

GlobalUpdateMeasurement GlobalUpdate::measured(const measurements::Measurement& meas)
{
    return *(meas.global_update);
}

void GlobalUpdate::operator()(History::Snapshot& snapshot)
{
    // Check the validity of the lidar measurements
    if (snapshot.measurement.global_update)
    {
        correct(snapshot);
    }
}
}  // namespace kalman
}  // namespace gnc
}  // namespace manifold_ukf