#pragma once

#include <cstdlib>
#include <list>

#include <yaml-cpp/yaml.h>

#include <State.hpp>
#include <measurements/Measurement.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
/**
 * Time series of sensor measurements and their corresponding estimated
 * states.
 */
class History
{
public:
    explicit History(YAML::Node config, YAML::Node initial_state_config);

public:
    /**
     * Snapshots store all information about a state at a specific time.
     */
    struct Snapshot
    {
        Snapshot(const State& state_, const measurements::Measurement& meas_)
            : state(state_), measurement(meas_)
        {
        }

        State state;
        measurements::Measurement measurement;

        uint64_t get_time() const { return state.timeUSec(); }
    };

    using Iterator = std::list<History::Snapshot>::iterator;
    using ConstIterator = std::list<History::Snapshot>::const_iterator;

    /**
     * Takes a MeasurementSet of all new measurements. Adds all of those
     * into the history, and returns a pair of iterators to the first
     * element that was changed and the end.
     *
     * IMU must be populated
     */
    std::pair<Iterator, Iterator> add_measurement(const measurements::MeasurementSet& measurements);

    size_t size();

    void setInitialBiases(const Eigen::Vector3d& gyro_bias, const Eigen::Vector3d& accel_bias);

private:
    void resize(Iterator last_modified);

    /***
     * Finds a snapshot within _tolerance of the given time or creates a new
     * snapshot with an interpolated IMU measurement.
     * @param time
     * @return
     */
    Iterator find_snapshot(uint64_t time);

    measurements::ImuMeasurement interpolate_imu(const measurements::ImuMeasurement& prev,
        const measurements::ImuMeasurement& next, uint64_t interp_time) const;

    Iterator set_last_modified(const Iterator last_modified, const Iterator modified) const;

    void moveMeasurements(const measurements::MeasurementSet& measurements);

private:
    size_t _size;
    uint64_t _tolerance;

    std::list<History::Snapshot> _history;

    State _initial_state;

    measurements::MeasurementSet _insert;
};

}  // namespace kalman
}  // namespace gnc
}  // namespace maav