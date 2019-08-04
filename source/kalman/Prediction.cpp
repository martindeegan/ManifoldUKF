#include <Constants.hpp>
#include <kalman/Prediction.hpp>
#include <yaml_matrix/yaml_matrix.hpp>

namespace manifold_ukf
{
namespace gnc
{
namespace kalman
{
UkfPrediction::UkfPrediction(YAML::Node config) : transformation(config["UT"])
{
    using std::placeholders::_1;
    std::function<State(const State&)> functor = std::bind(&UkfPrediction::predict, this, _1);
    transformation.set_transformation(functor);

    constexpr size_t IMU_DoF = 12;
    using ImpulseVector = Eigen::Matrix<double, IMU_DoF, 1>;
    ImpulseVector Q_i_vec = config["Q_i"].as<ImpulseVector>();
    Eigen::DiagonalMatrix<double, IMU_DoF> Q_i{Q_i_vec};

    Eigen::Matrix<double, State::DoF, IMU_DoF> F_i =
        Eigen::Matrix<double, State::DoF, IMU_DoF>::Zero();

    F_i.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    F_i.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();
    F_i.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
    F_i.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();

    Q = F_i * Q_i * F_i.transpose();
}

void UkfPrediction::operator()(const History::ConstIterator prev, const History::Iterator next)
{
    // Set internal references for use in the transition function
    _prev = prev;
    _next = next;

    // Propagate previous state
    _next->state = transformation(prev->state);

    // Add process noise
    _next->state.covariance() += Q;
}

State UkfPrediction::predict(const State& state)
{
    const State& prev_state = state;
    State next_state(_next->state.timeUSec());

    const measurements::ImuMeasurement& prev_imu = *(_prev->measurement.imu);
    const measurements::ImuMeasurement& next_imu = *(_next->measurement.imu);

    double dt = static_cast<double>(_next->get_time() - _prev->get_time()) * constants::USEC_TO_SEC;

    // Propagate unchanged biases
    next_state.gyroBias() = prev_state.gyroBias();
    next_state.accelBias() = prev_state.accelBias();
    next_state.gravity() = prev_state.gravity();
    next_state.magneticFieldVector() = prev_state.magneticFieldVector();

    const Eigen::Vector3d& w_prev = prev_imu.angular_rates - prev_state.gyroBias();
    const Eigen::Vector3d& w_next = next_imu.angular_rates - next_state.gyroBias();
    const Eigen::Vector3d w_mid = (w_next + w_prev) / 2.0;
    const Sophus::SO3d dR = Sophus::SO3d::exp(w_mid * dt);
    next_state.attitude() = prev_state.attitude() * dR;

    /**
     * Remove biases
     * Rotate to global frame
     * Remove gravity
     */
    const Eigen::Vector3d& a_prev =
        prev_state.attitude() * (prev_imu.acceleration - prev_state.accelBias()) -
        prev_state.gravity();
    const Eigen::Vector3d& a_next =
        next_state.attitude() * (next_imu.acceleration - next_state.accelBias()) -
        next_state.gravity();

    const Eigen::Vector3d a_mid = (a_next + a_prev) / 2.0;
    next_state.velocity() = prev_state.velocity() + (a_mid * dt);

    const Eigen::Vector3d& v_prev = prev_state.velocity();
    const Eigen::Vector3d& v_next = next_state.velocity();
    const Eigen::Vector3d v_mid = (v_next + v_prev) / 2.0;
    next_state.position() = prev_state.position() + (v_mid * dt);

    // Move corrected sensor readings
    next_state.angularVelocity() = w_next;
    next_state.acceleration() = a_next;

    return next_state;
}
}  // namespace kalman
}  // namespace gnc
}  // namespace manifold_ukf