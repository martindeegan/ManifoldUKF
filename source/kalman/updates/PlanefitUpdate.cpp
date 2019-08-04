#include <kalman/updates/PlanefitUpdate.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
using ErrorStateVector = PFSensorMeasurement::ErrorStateVector;
using CovarianceMatrix = PFSensorMeasurement::CovarianceMatrix;
using SensorVector = PFSensorMeasurement::SensorVector;

ErrorStateVector PFSensorMeasurement::operator-(const PFSensorMeasurement& other) const
{
    return readings_ - other.readings_;
}

PFSensorMeasurement& PFSensorMeasurement::operator+=(const ErrorStateVector& other)
{
    readings_ += other;
    return *this;
}

const CovarianceMatrix& PFSensorMeasurement::covariance() const { return covariance_; }
CovarianceMatrix& PFSensorMeasurement::covariance() { return covariance_; }
const SensorVector& PFSensorMeasurement::readings() const { return readings_; }
SensorVector& PFSensorMeasurement::readings() { return readings_; }
PlaneFitUpdate::PlaneFitUpdate(YAML::Node config) : BaseUpdate(config["planefit"]) {}
PFSensorMeasurement PlaneFitUpdate::predicted(const State& state)
{
    const Eigen::Vector3d euler =
        state.attitude().unit_quaternion().toRotationMatrix().eulerAngles(2, 1, 0);
    double roll = euler[2];
    double pitch = euler[1];
    double height_z = state.position().z();
    double z_dot = state.velocity().z();

    PFSensorMeasurement predicted_measurement;
    predicted_measurement.readings()(0) = roll;
    predicted_measurement.readings()(1) = pitch;
    predicted_measurement.readings()(2) = height_z;
    predicted_measurement.readings()(3) = z_dot;

    return predicted_measurement;
}

PFSensorMeasurement PlaneFitUpdate::measured(const measurements::Measurement& meas)
{
    PFSensorMeasurement sensor_measurement;
    sensor_measurement.readings()(0) = meas.plane_fit->roll;
    sensor_measurement.readings()(1) = meas.plane_fit->pitch;
    sensor_measurement.readings()(2) = meas.plane_fit->height;
    sensor_measurement.readings()(3) = meas.plane_fit->vertical_speed;
    return sensor_measurement;
}

void PlaneFitUpdate::operator()(History::Snapshot& snapshot)
{
    // Check the validity of the plane_fit measurements
    if (snapshot.measurement.plane_fit) correct(snapshot);
}

}  // namespace kalman
}  // namespace gnc
}  // namespace maav