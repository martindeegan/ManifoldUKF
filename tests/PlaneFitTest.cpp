#define BOOST_TEST_MODULE PlaneFitTest
/**
 * Tests for both BaseState and Kalman State
 */

#include <cmath>
#include <vector>

#include <Eigen/Eigen>
#include <boost/test/unit_test.hpp>
#include <sophus/so3.hpp>

#include <Constants.hpp>
#include <kalman/updates/PlanefitUpdate.hpp>

using namespace boost::unit_test;
using namespace Eigen;

using namespace maav::gnc::kalman;
using namespace maav::gnc;
using namespace maav::gnc::measurements;

/**
 * =============================================
 * TEST IS BROKEN FROM QUICK DEVELOPMENT
 * TODO FIX
 * =============================================
 */


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

    measurements::PlaneFitMeasurement planefit;
    planefit.height = -1;
    planefit.vertical_speed = 0;
    planefit.roll = 0;
    planefit.pitch = 0;
    planefit.time_usec = 1000;
    measurement.plane_fit = std::make_shared<measurements::PlaneFitMeasurement>(planefit);

    History::Snapshot snapshot(state, measurement);

    PlaneFitUpdate update(
        YAML::Load("planefit:\n    enabled: true\n    enable_outliers: false\n    UT:\n      "
                   "alpha: 0.1\n      beta: 2.0\n     "
                   " kappa: 0.0\n    "
                   "R: [0.000001, 0.000001, 0.000001, 0.000001]\n    extrinsics:\n      rot: [1, "
                   "0, 0, 0]\n      pos: [0, 0, 0]"));
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

    measurements::PlaneFitMeasurement planefit;
    planefit.time_usec = 1000;
    measurement.plane_fit = std::make_shared<measurements::PlaneFitMeasurement>(planefit);

    History::Snapshot snapshot(state, measurement);

    PlaneFitUpdate update(
        YAML::Load("planefit:\n    enabled: true\n    enable_outliers: false\n    UT:\n      "
                   "alpha: 0.1\n      beta: 2.0\n     "
                   " kappa: 0.0\n    "
                   "R: [0.000001, 0.000001, 0.000001, 0.000001]\n    extrinsics:\n      rot: [1, "
                   "0, 0, 0]\n      pos: [0, 0, 0]"));

    Eigen::Matrix<double, 4, 1> pred = update.predicted(state).readings();
    Eigen::Matrix<double, 4, 1> correct_pred;
    correct_pred(0) = 0;
    correct_pred(1) = 0;
    correct_pred(2) = -1;
    correct_pred(3) = 0;
    BOOST_CHECK_LE((pred - correct_pred).norm(), 0.000001);
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

    measurements::PlaneFitMeasurement planefit;
    planefit.time_usec = 1000;
    measurement.plane_fit = std::make_shared<measurements::PlaneFitMeasurement>(planefit);

    History::Snapshot snapshot(state, measurement);

    PlaneFitUpdate update(
        YAML::Load("planefit:\n    enabled: true\n    enable_outliers: false\n    UT:\n      "
                   "alpha: 0.1\n      beta: 2.0\n     "
                   " kappa: 0.0\n    "
                   "R: [0.000001, 0.000001, 0.000001, 0.000001]\n    extrinsics:\n      rot: [1, "
                   "0, 0, 0]\n      pos: [0, 0, 0]"));

    Eigen::Matrix<double, 4, 1> pred = update.predicted(state).readings();
    Eigen::Matrix<double, 4, 1> correct_pred;
    correct_pred(0) = 0;
    correct_pred(1) = 0.383972;
    correct_pred(2) = -3.25;
    correct_pred(3) = 0;
    BOOST_CHECK_LE((pred - correct_pred).norm(), 0.0005);
}
