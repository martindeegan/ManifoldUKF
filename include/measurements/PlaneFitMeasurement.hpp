#pragma once

#include <ostream>

namespace manifold_ukf
{
namespace gnc
{
namespace measurements
{
/*
 * Stores the necessary info we need from a plane fitting message.
 */
struct PlaneFitMeasurement
{
    double height;
    double vertical_speed;
    double roll;
    double pitch;
    uint64_t time_usec;
};

std::ostream& operator<<(std::ostream& os, const PlaneFitMeasurement& meas);

}  // namespace measurements
}  // namespace gnc
}  // namespace manifold_ukf
