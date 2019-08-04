#include <measurements/PlaneFitMeasurement.hpp>

namespace manifold_ukf
{
namespace gnc
{
namespace measurements
{
std::ostream& operator<<(std::ostream& os, const PlaneFitMeasurement& meas)
{
    os << "PlaneFit - Time: " << meas.time_usec << '\n';
    os << "PlaneFit - roll, pitch: " << meas.roll << ", " << meas.pitch << '\n';
    os << "PlaneFit - z, z_vel: " << meas.height << ", " << meas.vertical_speed << '\n';
    return os;
}
}  // namespace measurements
}  // namespace gnc
}  // namespace manifold_ukf