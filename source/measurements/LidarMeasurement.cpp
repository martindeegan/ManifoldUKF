#include <measurements/LidarMeasurement.hpp>

namespace maav
{
namespace gnc
{
namespace measurements
{
using ErrorStateVector = LidarMeasurement::ErrorStateVector;
using CovarianceMatrix = LidarMeasurement::CovarianceMatrix;
using SensorVector = LidarMeasurement::SensorVector;

ErrorStateVector LidarMeasurement::operator-(const LidarMeasurement& other) const
{
    return distance_ - other.distance_;
}

LidarMeasurement& LidarMeasurement::operator+=(const ErrorStateVector& other)
{
    distance_ += other;
    return *this;
}

const CovarianceMatrix& LidarMeasurement::covariance() const { return covariance_; }
CovarianceMatrix& LidarMeasurement::covariance() { return covariance_; }
const SensorVector& LidarMeasurement::distance() const { return distance_; }
SensorVector& LidarMeasurement::distance() { return distance_; }
uint64_t LidarMeasurement::timeUSec() const { return time_usec_; }
void LidarMeasurement::setTime(uint64_t time_usec) { time_usec_ = time_usec; }
std::ostream& operator<<(std::ostream& os, const LidarMeasurement& meas)
{
    os << "Lidar - Time: " << meas.timeUSec() << '\n';
    os << "Lidar Dist: " << meas.distance() << '\n';
    return os;
}
}
}
}