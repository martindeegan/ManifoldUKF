#include <iostream>
#include <iterator>
#include <memory>

#include <kalman/History.hpp>
#include <yaml_matrix/yaml_matrix.hpp>

using manifold_ukf::gnc::measurements::ImuMeasurement;
using manifold_ukf::gnc::measurements::Measurement;
using manifold_ukf::gnc::measurements::MeasurementSet;
using std::pair;

namespace manifold_ukf
{
namespace gnc
{
namespace kalman
{
History::History(YAML::Node config, YAML::Node initial_state_config)
    : _size(config["size"].as<size_t>()),
      _tolerance(config["tolerance"].as<uint64_t>()),
      _initial_state(State::zero(0))
{
    Eigen::Vector4d initial_attitude = initial_state_config["attitude"].as<Eigen::Vector4d>();
    Eigen::Quaterniond initial_quat = {
        initial_attitude.x(), initial_attitude.y(), initial_attitude.z(), initial_attitude.w()};
    Eigen::Vector3d initial_position = initial_state_config["position"].as<Eigen::Vector3d>();
    Eigen::Vector3d initial_velocity = initial_state_config["velocity"].as<Eigen::Vector3d>();
    Eigen::Matrix<double, State::DoF, 1> diag_covariance =
        initial_state_config["covariance"].as<Eigen::Matrix<double, State::DoF, 1>>();

    _initial_state.attitude() = Sophus::SO3d(Eigen::Quaterniond(initial_quat));
    _initial_state.position() = initial_position;
    _initial_state.velocity() = initial_velocity;

    _initial_state.covariance() = Eigen::DiagonalMatrix<double, State::DoF>(diag_covariance);
}

void printTimingWarning(std::string sensor)
{
    std::cout << "WARNING: " << sensor << " measurement was overriden. Are your timings correct?"
              << std::endl;
}

void History::moveMeasurements(const MeasurementSet &measurements)
{
    if (measurements.lidar)
    {
        if (_insert.lidar) printTimingWarning("lidar");
        _insert.lidar = measurements.lidar;
    }
    if (measurements.plane_fit)
    {
        if (_insert.plane_fit) printTimingWarning("plane fitting");
        _insert.plane_fit = measurements.plane_fit;
    }
    if (measurements.visual_odometry)
    {
        if (_insert.visual_odometry) printTimingWarning("visual odometry");
        _insert.visual_odometry = measurements.visual_odometry;
    }
    if (measurements.global_update)
    {
        if (_insert.global_update) printTimingWarning("global update");
        _insert.global_update = measurements.global_update;
    }
}

pair<History::Iterator, History::Iterator> History::add_measurement(
    const MeasurementSet &measurements)
{
    // Insert zero state if empty
    if (_history.empty())
    {
        uint64_t start_time = measurements.imu->time_usec;
        Measurement start_measurement;
        start_measurement.imu = measurements.imu;
        _initial_state.setTime(start_time);

        _history.emplace_back(_initial_state, start_measurement);
        return {_history.end(), _history.end()};
    }

    // Insert IMU first
    uint64_t imu_time = measurements.imu->time_usec;
    Measurement imu_measurement;
    imu_measurement.imu = measurements.imu;
    _history.emplace_back(State(imu_time), imu_measurement);
    auto last_modified = std::prev(_history.end());

    // Microsecond _tolerance to merge measurements
    // IMU will be running on a 10000 microsecond period

    moveMeasurements(measurements);
    if (_insert.lidar)
    {
        uint64_t lidar_time = _insert.lidar->timeUSec();
        auto snap_iter = find_snapshot(lidar_time);
        if (snap_iter != _history.end())
        {
            snap_iter->measurement.lidar = _insert.lidar;
            _insert.lidar = nullptr;
            last_modified = set_last_modified(last_modified, snap_iter);
        }
    }

    if (_insert.plane_fit)
    {
        uint64_t plane_fit_time = _insert.plane_fit->time_usec;
        auto snap_iter = find_snapshot(plane_fit_time);
        if (snap_iter != _history.end())
        {
            snap_iter->measurement.plane_fit = _insert.plane_fit;
            _insert.plane_fit = nullptr;
            last_modified = set_last_modified(last_modified, snap_iter);
        }
    }

    if (_insert.visual_odometry)
    {
        uint64_t vo_time = _insert.visual_odometry->time_usec;
        auto snap_iter = find_snapshot(vo_time);
        if (snap_iter != _history.end())
        {
            snap_iter->measurement.visual_odometry = _insert.visual_odometry;
            _insert.visual_odometry = nullptr;
            last_modified = set_last_modified(last_modified, snap_iter);
        }
    }

    if (_insert.global_update)
    {
        uint64_t gu_time = _insert.global_update->timeUSec();
        auto snap_iter = find_snapshot(gu_time);
        if (snap_iter != _history.end())
        {
            snap_iter->measurement.global_update = _insert.global_update;
            _insert.global_update = nullptr;
            last_modified = set_last_modified(last_modified, snap_iter);
        }
    }

    if (last_modified != _history.begin())
    {
        std::advance(last_modified, -1);
    }
    resize(last_modified);

    return {last_modified, _history.end()};
}

void History::resize(Iterator last_modified)
{
    while (_history.size() > _size)
    {
        auto iter = _history.begin();
        if (iter == last_modified)
        {
            break;
        }
        else
        {
            _history.pop_front();
        }
    }
}

void printLateWarning()
{
    std::cout << "Warning: measurement was not inserted because it arrived too late to be placed "
                 "in the history."
              << std::endl;
}

History::Iterator History::find_snapshot(uint64_t time)
{
    auto end = _history.rbegin();
    if (time > end->get_time())
    {
        if (time - end->get_time() < _tolerance)
        {
            // If within _tolerance of last IMU, add it
            return std::prev(_history.end());
        }
        else
        {
            // Otherwise, it is too new and IMU needs to be interpolated
            return _history.end();
        }
    }

    for (auto r_iter = std::next(end); r_iter != _history.rend(); std::advance(r_iter, 1))
    {
        uint64_t snapshot_time = r_iter->get_time();
        if (time > snapshot_time)
        {
            auto next_iter = r_iter.base();
            auto prev_iter = std::prev(next_iter);

            uint64_t next_t = next_iter->get_time();
            uint64_t prev_t = prev_iter->get_time();

            // Check tolerances and insert interpolated IMU if necessary
            if (time - prev_t <= _tolerance)
            {
                return prev_iter;
            }
            else if (next_t - time <= _tolerance)
            {
                return next_iter;
            }
            else
            {
                ImuMeasurement interp_imu = interpolate_imu(
                    *(prev_iter->measurement.imu), *(next_iter->measurement.imu), time);
                Measurement interp_measurement;
                interp_measurement.imu = std::make_shared<ImuMeasurement>(interp_imu);
                Snapshot interp_snapshot{State(time), interp_measurement};
                return _history.insert(next_iter, interp_snapshot);
            }
        }
    }

    printLateWarning();
    // If older than oldest measurement, discard
    return _history.end();
}

ImuMeasurement History::interpolate_imu(
    const ImuMeasurement &prev, const ImuMeasurement &next, uint64_t interp_time) const
{
    ImuMeasurement interpolated = prev;
    uint64_t prev_t = prev.time_usec;
    uint64_t next_t = next.time_usec;

    double weight = static_cast<double>(interp_time - prev_t);
    weight /= static_cast<double>(next_t - prev_t);

    interpolated.time_usec = interp_time;
    interpolated.angular_rates += weight * (next.angular_rates - prev.angular_rates);
    interpolated.acceleration += weight * (next.acceleration - prev.acceleration);
    interpolated.magnetometer += weight * (next.magnetometer - prev.magnetometer);

    return interpolated;
}

History::Iterator History::set_last_modified(
    const History::Iterator last_modified, const History::Iterator modified) const
{
    if (modified->get_time() < last_modified->get_time())
    {
        return modified;
    }
    else
    {
        return last_modified;
    }
}

size_t History::size() { return _history.size(); }
void History::setInitialBiases(const Eigen::Vector3d &gyro_bias, const Eigen::Vector3d &accel_bias)
{
    _initial_state.gyroBias() = gyro_bias;
    _initial_state.accelBias() = accel_bias;
}

}  // namespace kalman
}  // namespace gnc
}  // namespace manifold_ukf