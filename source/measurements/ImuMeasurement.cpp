#include <measurements/ImuMeasurement.hpp>

namespace maav
{
namespace gnc
{
namespace measurements
{
std::ostream& operator<<(std::ostream& os, const ImuMeasurement& meas)
{
    os << "IMU - Time: " << meas.time_usec << '\n';
    os << "IMU - Acceleration: " << meas.acceleration.transpose() << '\n';
    os << "IMU - Angular Rate: " << meas.angular_rates.transpose() << '\n';
    return os;
}
}
}
}