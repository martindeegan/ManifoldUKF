#define BOOST_TEST_MODULE HistoryTest

#include <iostream>
#include <vector>

#include <Eigen/Eigen>
#include <boost/test/unit_test.hpp>

#include "kalman/History.hpp"
#include "measurements/GlobalUpdateMeasurement.hpp"
#include "measurements/ImuMeasurement.hpp"
#include "measurements/LidarMeasurement.hpp"
#include "measurements/Measurement.hpp"
#include "measurements/PlaneFitMeasurement.hpp"
#include "measurements/VisualOdometry.hpp"

using maav::gnc::kalman::History;
using maav::gnc::measurements::GlobalUpdateMeasurement;
using maav::gnc::measurements::ImuMeasurement;
using maav::gnc::measurements::LidarMeasurement;
using maav::gnc::measurements::Measurement;
using maav::gnc::measurements::MeasurementSet;
using maav::gnc::measurements::PlaneFitMeasurement;
using maav::gnc::measurements::VisualOdometryMeasurement;

constexpr size_t HISTORY_SIZE = 3;

BOOST_AUTO_TEST_CASE(HistoryInitializeTest)
{
    YAML::Node config = YAML::Load("size: 3\ntolerance: 1000");
    YAML::Node initial_state_config = YAML::Load(
        "# Starting state\nattitude: [1, 0, 0, 0]\nposition: [0, 0, 0] # [m]\nvelocity: [0, 0, 0] "
        "# [m/s]\n# Starting covariance\ncovariance: [0.00001, 0.00001, 0.00001, 0.001, 0.001, "
        "0.001, 0.0001, 0.0001, 0.0001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001]");
    History history(config, initial_state_config);
    MeasurementSet set;
    ImuMeasurement imu;
    imu.time_usec = 0;
    LidarMeasurement lidar;
    lidar.setTime(1500);

    set.imu = std::make_shared<ImuMeasurement>(imu);
    set.lidar = std::make_shared<LidarMeasurement>(lidar);

    auto iter_pair = history.add_measurement(set);

    // First insertion should return two end iterators
    BOOST_CHECK(iter_pair.first == iter_pair.second);
}

BOOST_AUTO_TEST_CASE(HistorySimpleTest)
{
    // Create first measurement set
    YAML::Node config = YAML::Load("size: 3\ntolerance: 1000");
    YAML::Node initial_state_config = YAML::Load(
        "# Starting state\nattitude: [1, 0, 0, 0]\nposition: [0, 0, 0] # [m]\nvelocity: [0, 0, 0] "
        "# [m/s]\n# Starting covariance\ncovariance: [0.00001, 0.00001, 0.00001, 0.001, 0.001, "
        "0.001, 0.0001, 0.0001, 0.0001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001]");
    History history(config, initial_state_config);
    MeasurementSet set1;
    ImuMeasurement imu1;
    imu1.time_usec = 0;
    LidarMeasurement lidar1;
    lidar1.setTime(0);
    set1.imu = std::make_shared<const ImuMeasurement>(imu1);
    set1.lidar = std::make_shared<const LidarMeasurement>(lidar1);

    // Create seconds measurement set
    MeasurementSet set2;
    ImuMeasurement imu2;
    imu2.time_usec = 10000;
    set2.imu = std::make_shared<const ImuMeasurement>(imu2);

    // Insert both sets
    history.add_measurement(set1);
    auto iter_pair = history.add_measurement(set2);

    std::vector<History::Snapshot> history_vec(iter_pair.first, iter_pair.second);
    BOOST_CHECK_EQUAL(history_vec.size(), 2);
    // Initial disregards anthing but IMU, seconds has only IMU
    BOOST_CHECK(history_vec[0].measurement.lidar == nullptr);
    BOOST_CHECK(history_vec[1].measurement.lidar == nullptr);
    // Check tines
    BOOST_CHECK_EQUAL(history_vec[0].get_time(), 0);
    BOOST_CHECK_EQUAL(history_vec[1].get_time(), 10000);
}

BOOST_AUTO_TEST_CASE(HistoryOverflowTest)
{
    YAML::Node config = YAML::Load("size: 3\ntolerance: 1000");
    YAML::Node initial_state_config = YAML::Load(
        "# Starting state\nattitude: [1, 0, 0, 0]\nposition: [0, 0, 0] # [m]\nvelocity: [0, 0, 0] "
        "# [m/s]\n# Starting covariance\ncovariance: [0.00001, 0.00001, 0.00001, 0.001, 0.001, "
        "0.001, 0.0001, 0.0001, 0.0001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001]");
    History history(config, initial_state_config);

    // Add 10 measurements
    for (size_t i = 0; i < 10; i++)
    {
        MeasurementSet set;
        ImuMeasurement imu;
        imu.time_usec = 10000 * i;
        set.imu = std::make_shared<ImuMeasurement>(imu);

        history.add_measurement(set);
    }

    // Should still be length 3
    BOOST_CHECK_EQUAL(history.size(), 3);
}

BOOST_AUTO_TEST_CASE(HistoryTolerance)
{
    YAML::Node config = YAML::Load("size: 3\ntolerance: 1000");
    YAML::Node initial_state_config = YAML::Load(
        "# Starting state\nattitude: [1, 0, 0, 0]\nposition: [0, 0, 0] # [m]\nvelocity: [0, 0, 0] "
        "# [m/s]\n# Starting covariance\ncovariance: [0.00001, 0.00001, 0.00001, 0.001, 0.001, "
        "0.001, 0.0001, 0.0001, 0.0001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001]");
    History history(config, initial_state_config);

    // Add 4 measurements
    for (size_t i = 0; i < 4; i++)
    {
        MeasurementSet set;
        ImuMeasurement imu;
        imu.time_usec = 10000 * i;
        set.imu = std::make_shared<ImuMeasurement>(imu);

        history.add_measurement(set);
    }

    // History should have times 10000 - 30000

    // Add set with lidar at 30900us
    MeasurementSet set1;
    ImuMeasurement imu1;
    LidarMeasurement lidar1;
    imu1.time_usec = 40000;
    lidar1.setTime(30900);
    set1.imu = std::make_shared<ImuMeasurement>(imu1);
    set1.lidar = std::make_shared<LidarMeasurement>(lidar1);
    auto iter_pair1 = history.add_measurement(set1);
    std::vector<History::Snapshot> history_vec1(iter_pair1.first, iter_pair1.second);
    BOOST_CHECK_EQUAL(history_vec1.size(), 3);
    BOOST_CHECK(history_vec1[0].measurement.lidar == nullptr);
    BOOST_CHECK(history_vec1[1].measurement.lidar != nullptr);
    BOOST_CHECK(history_vec1[2].measurement.lidar == nullptr);
    BOOST_CHECK_EQUAL(history_vec1[1].measurement.lidar->timeUSec(), 30900);

    // Add set with lidar at 39900
    MeasurementSet set2;
    ImuMeasurement imu2;
    LidarMeasurement lidar2;
    imu2.time_usec = 50000;
    lidar2.setTime(39900);
    set2.imu = std::make_shared<ImuMeasurement>(imu2);
    set2.lidar = std::make_shared<LidarMeasurement>(lidar2);

    // Should return the entire history
    auto iter_pair2 = history.add_measurement(set2);
    std::vector<History::Snapshot> history_vec2(iter_pair2.first, iter_pair2.second);
    BOOST_CHECK_EQUAL(history_vec2.size(), 3);
    BOOST_CHECK(history_vec2[0].measurement.lidar != nullptr);
    BOOST_CHECK(history_vec2[1].measurement.lidar != nullptr);
    BOOST_CHECK(history_vec2[2].measurement.lidar == nullptr);
    BOOST_CHECK_EQUAL(history_vec2[0].measurement.lidar->timeUSec(), 30900);
    BOOST_CHECK_EQUAL(history_vec2[1].measurement.lidar->timeUSec(), 39900);
}

BOOST_AUTO_TEST_CASE(HistoryInerpolateTest)
{
    YAML::Node config = YAML::Load("size: 3\ntolerance: 1000");
    YAML::Node initial_state_config = YAML::Load(
        "# Starting state\nattitude: [1, 0, 0, 0]\nposition: [0, 0, 0] # [m]\nvelocity: [0, 0, 0] "
        "# [m/s]\n# Starting covariance\ncovariance: [0.00001, 0.00001, 0.00001, 0.001, 0.001, "
        "0.001, 0.0001, 0.0001, 0.0001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001]");
    History history(config, initial_state_config);

    // Add 4 measurements
    for (size_t i = 0; i < 4; i++)
    {
        MeasurementSet set;
        ImuMeasurement imu;
        imu.angular_rates = Eigen::Vector3d(1, 1, 1) * i;
        imu.time_usec = 10000 * i;
        set.imu = std::make_shared<ImuMeasurement>(imu);

        history.add_measurement(set);
    }

    // History should have times 10000 - 30000

    // Add final measurement and get iterators
    MeasurementSet set;
    ImuMeasurement imu;
    LidarMeasurement lidar;
    imu.time_usec = 40000;
    imu.angular_rates = Eigen::Vector3d(1, 1, 1) * 4;
    lidar.setTime(24000);
    set.imu = std::make_shared<ImuMeasurement>(imu);
    set.lidar = std::make_shared<LidarMeasurement>(lidar);

    // Should return the entire history
    auto iter_pair = history.add_measurement(set);
    std::vector<History::Snapshot> history_vec(iter_pair.first, iter_pair.second);

    // Entire history is length 3
    BOOST_CHECK_EQUAL(history_vec.size(), 4);
    BOOST_CHECK_EQUAL(history_vec[0].get_time(), 20000);
    BOOST_CHECK_EQUAL(history_vec[1].get_time(), 24000);
    BOOST_CHECK_EQUAL(history_vec[2].get_time(), 30000);
    BOOST_CHECK_EQUAL(history_vec[3].get_time(), 40000);

    const ImuMeasurement &imu_meas = *(history_vec[1].measurement.imu);
    BOOST_CHECK_EQUAL(imu_meas.angular_rates, Eigen::Vector3d(2.4, 2.4, 2.4));
}

MeasurementSet create_meas(uint64_t imu_t, uint64_t lid_t)
{
    MeasurementSet set;
    ImuMeasurement imu;
    LidarMeasurement lidar;
    imu.time_usec = imu_t;
    lidar.setTime(lid_t);
    set.imu = std::make_shared<ImuMeasurement>(imu);
    set.lidar = std::make_shared<LidarMeasurement>(lidar);
    return set;
}

BOOST_AUTO_TEST_CASE(HistoryOlderTest)
{
    YAML::Node config = YAML::Load("size: 3\ntolerance: 1000");
    YAML::Node initial_state_config = YAML::Load(
        "# Starting state\nattitude: [1, 0, 0, 0]\nposition: [0, 0, 0] # [m]\nvelocity: [0, 0, 0] "
        "# [m/s]\n# Starting covariance\ncovariance: [0.00001, 0.00001, 0.00001, 0.001, 0.001, "
        "0.001, 0.0001, 0.0001, 0.0001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001]");
    History history(config, initial_state_config);

    auto set1 = create_meas(0, 0);
    history.add_measurement(set1);

    auto set2 = create_meas(10000, 10000);
    history.add_measurement(set2);

    auto set3 = create_meas(20000, 20500);
    auto it_pair3 = history.add_measurement(set3);
    std::vector<History::Snapshot> history_vec3(it_pair3.first, it_pair3.second);

    BOOST_CHECK_EQUAL(history_vec3[0].measurement.imu->time_usec, 10000);
    BOOST_CHECK_EQUAL(history_vec3[1].measurement.imu->time_usec, 20000);
    BOOST_CHECK_EQUAL(history_vec3[1].measurement.lidar->timeUSec(), 20500);

    auto set4 = create_meas(30000, 34000);
    auto it_pair4 = history.add_measurement(set4);
    std::vector<History::Snapshot> history_vec4(it_pair4.first, it_pair4.second);

    BOOST_CHECK_EQUAL(history_vec4[0].measurement.imu->time_usec, 20000);
    BOOST_CHECK_EQUAL(history_vec4[1].measurement.imu->time_usec, 30000);
    BOOST_CHECK(history_vec4[1].measurement.lidar == nullptr);

    ImuMeasurement imu;
    imu.time_usec = 40000;
    set4.imu = std::make_shared<ImuMeasurement>(imu);
    auto it_pair5 = history.add_measurement(set4);
    std::vector<History::Snapshot> history_vec5(it_pair5.first, it_pair5.second);

    BOOST_CHECK_EQUAL(history_vec5.size(), 3);
    BOOST_CHECK_EQUAL(history_vec5[0].measurement.imu->time_usec, 30000);
    BOOST_CHECK_EQUAL(history_vec5[1].measurement.imu->time_usec, 34000);
    BOOST_CHECK_EQUAL(history_vec5[1].measurement.lidar->timeUSec(), 34000);
    BOOST_CHECK_EQUAL(history_vec5[2].measurement.imu->time_usec, 40000);
}
