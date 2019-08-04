#include <measurements/GlobalUpdateMeasurement.hpp>

namespace maav
{
namespace gnc
{
namespace measurements
{
using ErrorStateVector = GlobalUpdateMeasurement::ErrorStateVector;
using CovarianceMatrix = GlobalUpdateMeasurement::CovarianceMatrix;

ErrorStateVector GlobalUpdateMeasurement::operator-(const GlobalUpdateMeasurement& other) const
{
    const Sophus::SE3d pose_error = other.pose().inverse() * pose();
    return pose_error.log();
}

GlobalUpdateMeasurement& GlobalUpdateMeasurement::operator+=(const ErrorStateVector& other)
{
    pose() = pose() * Sophus::SE3d::exp(other);
    return *this;
}

const Sophus::SE3d& GlobalUpdateMeasurement::pose() const { return pose_; }
Sophus::SE3d& GlobalUpdateMeasurement::pose() { return pose_; }

const CovarianceMatrix& GlobalUpdateMeasurement::covariance() const { return covariance_; }
CovarianceMatrix& GlobalUpdateMeasurement::covariance() { return covariance_; }

uint64_t GlobalUpdateMeasurement::timeUSec() const { return time_usec_; }
void GlobalUpdateMeasurement::setTime(uint64_t time_usec) { time_usec_ = time_usec; }

std::ostream& operator<<(std::ostream& os, const GlobalUpdateMeasurement& meas)
{
    std::cout << "Global - Time: " << meas.timeUSec() << '\n';
    const Eigen::Quaterniond& q = meas.pose().so3().unit_quaternion();
    std::cout << "Global - Attitude: " << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z()
              << '\n';
    std::cout << "Global - Position: " << meas.pose().translation().transpose() << '\n';
    return os;
}
}  // namespace measurements
}  // namespace gnc
}  // namespace maav