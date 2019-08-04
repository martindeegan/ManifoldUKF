#include <cmath>

#include <Estimator.hpp>
#include <State.hpp>

using maav::gnc::measurements::MeasurementSet;

namespace maav
{
namespace gnc
{
using namespace kalman;

Estimator::Estimator(YAML::Node config)
    : empty_state_(0),
      history_(config["history"], config["state"]),
      prediction_(config["prediction"]),
      lidar_update_(config["updates"]),
      planefit_update_(config["updates"]),
      global_update_(config["updates"])
{
    std::cout << "Lidar Update:     ";
    if (lidar_update_.enabled())
        std::cout << "ENABLED\n";
    else
        std::cout << "DISABLED\n";

    std::cout << "Plane Fit Update: ";
    if (planefit_update_.enabled())
        std::cout << "ENABLED\n";
    else
        std::cout << "DISABLED\n";

    std::cout << "Global Update:    ";
    if (global_update_.enabled())
        std::cout << "ENABLED\n";
    else
        std::cout << "DISABLED\n";
}

const State& Estimator::add_measurement_set(const MeasurementSet& meas)
{
    auto it_pair = history_.add_measurement(meas);
    History::Iterator begin = it_pair.first;
    History::Iterator end = it_pair.second;

    if (begin == end)
    {
        return empty_state_;
    }

    History::Iterator prev, next;
    next = prev = begin;
    next++;

    while (next != end)
    {
        prediction_(prev, next);

        lidar_update_(*next);
        planefit_update_(*next);
        global_update_(*next);

        prev++;
        next++;
    }
    return prev->state;
}

void Estimator::setBiases(const Eigen::Vector3d& gyro_bias, const Eigen::Vector3d& accel_bias)
{
    history_.setInitialBiases(gyro_bias, accel_bias);
}
}  // namespace gnc
}  // namespace maav
