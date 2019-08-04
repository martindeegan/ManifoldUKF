#define BOOST_TEST_MODULE StateTest
/**
 * Tests for both State and Kalman State
 */

#include <cmath>
#include <list>
#include <vector>

#include <Eigen/Eigen>
#include <boost/test/unit_test.hpp>
#include <sophus/so3.hpp>

#include <Constants.hpp>
#include <kalman/Prediction.hpp>
#include "TestHelpers.hpp"

using namespace boost::unit_test;
using namespace Eigen;

using namespace manifold_ukf::gnc::kalman;
using namespace manifold_ukf::gnc;

class TestUkfPrediction : public UkfPrediction
{
public:
    using UkfPrediction::UkfPrediction;
    using UkfPrediction::operator();
    const State& get_transformed_mean()
    {
        const auto& ut = UkfPrediction::getUT();
        return ut.last_transformed_points()[0];
    }
};

BOOST_AUTO_TEST_CASE(RunTest)
{
    YAML::Node config = YAML::Load(
        "# Unscented transform params\nUT:\n  alpha: 0.1\n  beta: 2.0\n  kappa: 0.0\n# Process "
        "noise covariance\nQ_i: [0.00005, 0.00005, 0.00005, 0.00002, 0.00002, 0.00002, 0.00001, "
        "0.00001, 0.00001, 0.00001, 0.00001, 0.00001]\n");

    UkfPrediction pred(config);
}

BOOST_AUTO_TEST_CASE(PredictionStep)
{
    YAML::Node config = YAML::Load(
        "# Unscented transform params\nUT:\n  alpha: 0.1\n  beta: 2.0\n  kappa: 0.0\n# Process "
        "noise covariance\nQ_i: [0.00005, 0.00005, 0.00005, 0.00002, 0.00002, 0.00002, 0.00001, "
        "0.00001, 0.00001, 0.00001, 0.00001, 0.00001]\n");

    measurements::ImuMeasurement imu1;
    imu1.time_usec = 0;
    imu1.angular_rates = Eigen::Vector3d::Zero();
    imu1.acceleration = {0, 0, -constants::STANDARD_GRAVITY};
    imu1.magnetometer = Eigen::Vector3d::Zero();

    measurements::ImuMeasurement imu2;
    imu2.time_usec = 1000000;
    imu2.angular_rates = {0.23, -0.012, 0.89};
    imu2.acceleration = {2.0, -1.012, -9.72};
    imu2.magnetometer = Eigen::Vector3d::Zero();

    State state1 = State::zero(0);
    state1.covariance() *= 0.05;

    State state2 = State::zero(1000000);

    measurements::Measurement meas1;
    meas1.imu = std::make_shared<measurements::ImuMeasurement>(imu1);
    measurements::Measurement meas2;
    meas2.imu = std::make_shared<measurements::ImuMeasurement>(imu2);

    History::Snapshot snap1(state1, meas1);
    History::Snapshot snap2(state2, meas2);

    std::list<History::Snapshot> history;
    history.push_back(snap1);
    history.push_back(snap2);

    TestUkfPrediction pred(config);
    pred(history.begin(), ++history.begin());

    const State& s2 = pred.get_transformed_mean();

    Eigen::Vector3d next_velocity = {1.0262, 0.5215, 0.0504};
    Eigen::Vector3d next_position = {0.5131, 0.2607, 0.0252};
    Eigen::Matrix3d next_attitude_;
    // clang-format off
	next_attitude_ << 0.9027,   -0.4298,    0.0193,
    				  0.4292,    0.8962,   -0.1123,
    				  0.0309,    0.1097,    0.9935;
    // clang-format on
    Sophus::SO3d next_attitude(Eigen::Quaterniond(next_attitude_).normalized());
    constexpr double tol = 1e-4;

    BOOST_CHECK_LE(diff(s2.position(), next_position), tol);
    BOOST_CHECK_LE(diff(s2.velocity(), next_velocity), tol);
    BOOST_CHECK_LE(diff(s2.attitude(), next_attitude), tol);
}