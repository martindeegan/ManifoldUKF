#define BOOST_TEST_MODULE LidarTest
/**
 * Tests for both State and Kalman State
 */

/**
 * =============================================
 * TEST IS BROKEN FROM QUICK DEVELOPMENT
 * TODO FIX
 * =============================================
 */


#include <cmath>
#include <vector>

#include <Eigen/Eigen>
#include <boost/test/unit_test.hpp>
#include <sophus/so3.hpp>

#include <Constants.hpp>
#include <kalman/updates/LidarUpdate.hpp>

using namespace boost::unit_test;
using namespace Eigen;

using namespace maav::gnc::kalman;
using namespace maav::gnc;
using namespace maav::gnc::measurements;

BOOST_AUTO_TEST_CASE(RunTest)
{
    State state(1000);
    state.attitude() = Sophus::SO3d(Quaterniond{1, 0, 0, 0});
    state.position() = {0, 1, -1};
    state.velocity() = Vector3d::Zero();
    state.angularVelocity() = Vector3d::Zero();
    state.acceleration() = Vector3d::Zero();

    measurements::Measurement measurement;
    measurements::ImuMeasurement imu;
    imu.time_usec = 1000;
    imu.acceleration = {0, 0, -maav::gnc::constants::STANDARD_GRAVITY};
    imu.angular_rates = Vector3d::Zero();
    imu.magnetometer = {0, 0, 1};

    measurement.imu = std::make_shared<measurements::ImuMeasurement>(imu);

    LidarMeasurement lidar;
    lidar.distance() = LidarMeasurement::SensorVector{1};
    lidar.setTime(1000);
    measurement.lidar = std::make_shared<measurements::LidarMeasurement>(lidar);

    History::Snapshot snapshot(state, measurement);

    LidarUpdate update(YAML::Load(
        "imu_height: 0.0\nlidar:\n    enabled: true\n    enable_outliers: false\n    lidar_bias: "
        "0.0\n    UT:\n      alpha: 0.1\n      beta: 2.0\n      kappa: 0.0\n    R: [0.01]\n    "
        "extrinsics:\n      rot: [1, 0, 0, 0]\n      pos: [0, 0, 0]"));
    update(snapshot);
}

BOOST_AUTO_TEST_CASE(SimpleSensorModelTest)
{
    State state(1000);
    state.attitude() = Sophus::SO3d(Quaterniond{1, 0, 0, 0});
    state.position() = {0, 1, -1};
    state.velocity() = Vector3d::Zero();
    state.angularVelocity() = Vector3d::Zero();
    state.acceleration() = Vector3d::Zero();

    measurements::Measurement measurement;
    measurements::ImuMeasurement imu;
    imu.time_usec = 1000;
    imu.acceleration = {0, 0, -maav::gnc::constants::STANDARD_GRAVITY};
    imu.angular_rates = Vector3d::Zero();
    imu.magnetometer = {0, 0, 1};

    measurement.imu = std::make_shared<measurements::ImuMeasurement>(imu);

    measurements::LidarMeasurement lidar;
    lidar.distance() = LidarMeasurement::SensorVector{1};
    lidar.setTime(1000);
    measurement.lidar = std::make_shared<measurements::LidarMeasurement>(lidar);

    History::Snapshot snapshot(state, measurement);

    LidarUpdate update(YAML::Load(
        "imu_height: 0.0\nlidar:\n    enabled: true\n    enable_outliers: false\n    lidar_bias: "
        "0.0\n    UT:\n      alpha: 0.1\n      beta: 2.0\n      kappa: 0.0\n    R: [0.01]\n    "
        "extrinsics:\n      rot: [1, 0, 0, 0]\n      pos: [0, 0, 0]"));

    double pred = update.predicted(state).distance()(0);
    double correct_pred = 1;
    BOOST_CHECK_EQUAL(pred, correct_pred);
}

BOOST_AUTO_TEST_CASE(AdvancedSensorModelTest)
{
    State state(1000);
    state.attitude() = Sophus::SO3d(Quaterniond{0.982, 0, 0.191, 0});
    state.attitude().normalize();
    state.position() = {10.1, -23.2, -3.25};
    state.velocity() = Vector3d::Zero();
    state.angularVelocity() = Vector3d::Zero();
    state.acceleration() = Vector3d::Zero();

    measurements::Measurement measurement;
    measurements::ImuMeasurement imu;
    imu.time_usec = 1000;
    imu.acceleration = {0, 0, -maav::gnc::constants::STANDARD_GRAVITY};
    imu.angular_rates = Vector3d::Zero();
    imu.magnetometer = {0, 0, 1};

    measurement.imu = std::make_shared<measurements::ImuMeasurement>(imu);

    measurements::LidarMeasurement lidar;
    lidar.distance() = LidarMeasurement::SensorVector{1};
    lidar.setTime(1000);
    measurement.lidar = std::make_shared<measurements::LidarMeasurement>(lidar);

    History::Snapshot snapshot(state, measurement);

    LidarUpdate update(YAML::Load(
        "imu_height: 0.0\nlidar:\n    enabled: true\n    enable_outliers: false\n    lidar_bias: "
        "0.0\n    UT:\n      alpha: 0.1\n      beta: 2.0\n      kappa: 0.0\n    R: [0.01]\n    "
        "extrinsics:\n      rot: [1, 0, 0, 0]\n      pos: [0, 0, 0]"));

    double pred = update.predicted(state).distance()(0);
    double correct_pred = 3.5052379137;
    BOOST_CHECK_CLOSE(pred, correct_pred, 0.01);
}

BOOST_AUTO_TEST_CASE(SensorMeasuredTest)
{
    State state(1000);
    state.attitude() = Sophus::SO3d(Quaterniond{1, 0, 0, 0});
    state.position() = {0, 1, -1};
    state.velocity() = Vector3d::Zero();
    state.angularVelocity() = Vector3d::Zero();
    state.acceleration() = Vector3d::Zero();

    measurements::Measurement measurement;
    measurements::ImuMeasurement imu;
    imu.time_usec = 1000;
    imu.acceleration = {0, 0, -maav::gnc::constants::STANDARD_GRAVITY};
    imu.angular_rates = Vector3d::Zero();
    imu.magnetometer = {0, 0, 1};

    measurement.imu = std::make_shared<measurements::ImuMeasurement>(imu);

    measurements::LidarMeasurement lidar;
    lidar.distance() = LidarMeasurement::SensorVector{1};
    lidar.setTime(1000);
    measurement.lidar = std::make_shared<measurements::LidarMeasurement>(lidar);

    History::Snapshot snapshot(state, measurement);

    LidarUpdate update(YAML::Load(
        "imu_height: 0.0\nlidar:\n    enabled: true\n    enable_outliers: false\n    lidar_bias: "
        "0.0\n    UT:\n      alpha: 0.1\n      beta: 2.0\n      kappa: 0.0\n    R: [0.01]\n    "
        "extrinsics:\n      rot: [1, 0, 0, 0]\n      pos: [0, 0, 0]"));

    double pred = update.measured(measurement).distance()(0);
    double correct_pred = 1;
    BOOST_CHECK_EQUAL(pred, correct_pred);
}
