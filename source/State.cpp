#include <Constants.hpp>
#include <State.hpp>

namespace maav
{
namespace gnc
{
State::State(uint64_t time_usec) : time_usec_(time_usec) {}
State State::zero(uint64_t time_usec)
{
    State new_state(time_usec);
    new_state.attitude() = Sophus::SO3d(Eigen::Quaterniond::Identity());
    new_state.position() = Eigen::Vector3d::Zero();
    new_state.velocity() = Eigen::Vector3d::Zero();
    new_state.angularVelocity() = Eigen::Vector3d::Zero();
    new_state.acceleration() = {0.0, 0.0, -constants::STANDARD_GRAVITY};
    new_state.gyroBias() = Eigen::Vector3d::Zero();
    new_state.accelBias() = Eigen::Vector3d::Zero();
    // The standard gravity vector
    new_state.gravity() = {0.0, 0.0, -constants::STANDARD_GRAVITY};
    // Ann Arbor's Magnetic field in NED in microtesla
    new_state.magneticFieldVector() = constants::ANN_ARBOR_MAGNETIC_FIELD;
    // Constant start
    new_state.covariance() = 0.00001 * CovarianceMatrix::Identity();
    return new_state;
}

State State::mean(const std::array<State, N>& sigma_points, const std::array<double, N>& weights)
{
    const State& first = sigma_points[0];
    uint64_t time = first.timeUSec();
    State mean_state = State::zero(time);

    // Use only the previous mean's transformed point because this is linear with respect to the
    // attitude
    mean_state.attitude() = sigma_points[0].attitude();
    for (size_t i = 0; i < sigma_points.size(); i++)
    {
        mean_state.position() += sigma_points[i].position() * weights[i];
        mean_state.velocity() += sigma_points[i].velocity() * weights[i];

        mean_state.acceleration() += sigma_points[i].acceleration() * weights[i];
        mean_state.angularVelocity() += sigma_points[i].angularVelocity() * weights[i];

        // TODO: Determine if other values are necessary to average.
        // TODO: Use other estimated parameters
        // Propagate bias
        mean_state.gyroBias() += sigma_points[i].gyroBias() * weights[i];
        mean_state.accelBias() += sigma_points[i].accelBias() * weights[i];
    }
    return mean_state;
}

State::CovarianceMatrix State::cov(const State& mean, const std::array<State, N>& sigma_points,
    const std::array<double, N>& weights)
{
    std::array<ErrorStateVector, N> points;
    for (size_t i = 0; i < N; i++)
    {
        const State& state = sigma_points[i];
        ErrorStateVector& e_state = points[i];
        Eigen::Vector3d q_err = (mean.attitude().inverse() * state.attitude()).log();
        e_state.segment<3>(0) = q_err;
        e_state.segment<3>(3) = state.position();
        e_state.segment<3>(6) = state.velocity();
        e_state.segment<3>(9) = state.gyroBias();
        e_state.segment<3>(12) = state.accelBias();

        // TODO: add more states to estimate
    }

    ErrorStateVector mean_e_state;
    Eigen::Vector3d q_err = Eigen::Vector3d::Zero();
    mean_e_state.segment<3>(0) = q_err;
    mean_e_state.segment<3>(3) = mean.position();
    mean_e_state.segment<3>(6) = mean.velocity();
    mean_e_state.segment<3>(9) = mean.gyroBias();
    mean_e_state.segment<3>(12) = mean.accelBias();

    CovarianceMatrix new_covariance = CovarianceMatrix::Zero();
    for (size_t i = 0; i < sigma_points.size(); i++)
    {
        ErrorStateVector residual = points[i] - mean_e_state;
        new_covariance += weights[i] * residual * residual.transpose();
    }

    return new_covariance;
}

State State::compute_gaussian(const std::array<State, N>& points,
    const std::array<double, N>& m_weights, const std::array<double, N>& c_weights)
{
    State mu = State::mean(points, m_weights);
    mu.covariance() = State::cov(mu, points, c_weights);
    return mu;
}

State& State::operator+=(const State::ErrorStateVector& e_state)
{
    const Sophus::SO3d attitude_err = Sophus::SO3d::exp(e_state.head(3));
    const Eigen::Vector3d position_err = e_state.segment<3>(3);
    const Eigen::Vector3d velocity_err = e_state.segment<3>(6);
    const Eigen::Vector3d gyro_bias_err = e_state.segment<3>(9);
    const Eigen::Vector3d accel_bias_err = e_state.segment<3>(12);

    position() += attitude() * position_err;
    velocity() += attitude() * velocity_err;
    attitude() *= attitude_err;
    gyroBias() += gyro_bias_err;
    accelBias() += accel_bias_err;

    return *this;
}

State::ErrorStateVector State::operator-(const State& other) const
{
    ErrorStateVector difference;
    difference.segment<3>(0) = (other.attitude().inverse() * attitude()).log();
    difference.segment<3>(3) = other.attitude().inverse() * (position() - other.position());
    difference.segment<3>(6) = other.attitude().inverse() * (velocity() - other.velocity());
    difference.segment<3>(9) = gyroBias() - other.gyroBias();
    difference.segment<3>(12) = accelBias() - other.accelBias();

    return difference;
}

Sophus::SE3d State::getPose() const
{
    Sophus::SE3d pose;
    pose.so3() = attitude();
    pose.translation() = position();
    return pose;
}

void State::setPose(const Sophus::SE3d& pose)
{
    attitude() = pose.so3();
    position() = pose.translation();
}

const Sophus::SO3d& State::attitude() const { return attitude_; }
Sophus::SO3d& State::attitude() { return attitude_; }
const Eigen::Vector3d& State::angularVelocity() const { return angular_velocity_; }
Eigen::Vector3d& State::angularVelocity() { return angular_velocity_; }
const Eigen::Vector3d& State::position() const { return position_; }
Eigen::Vector3d& State::position() { return position_; }
const Eigen::Vector3d& State::velocity() const { return velocity_; }
Eigen::Vector3d& State::velocity() { return velocity_; }
const Eigen::Vector3d& State::acceleration() const { return acceleration_; }
Eigen::Vector3d& State::acceleration() { return acceleration_; }
void State::setTime(uint64_t usec) { time_usec_ = usec; }
uint64_t State::timeUSec() const { return time_usec_; }
double State::timeSec() const { return static_cast<double>(time_usec_) * constants::USEC_TO_SEC; }
const Eigen::Vector3d& State::gyroBias() const { return gyro_bias_; }
Eigen::Vector3d& State::gyroBias() { return gyro_bias_; }
const Eigen::Vector3d& State::accelBias() const { return accel_bias_; }
Eigen::Vector3d& State::accelBias() { return accel_bias_; }
const Eigen::Vector3d& State::gravity() const { return gravity_; }
Eigen::Vector3d& State::gravity() { return gravity_; }
const Eigen::Vector3d& State::magneticFieldVector() const { return magnetic_field_; }
Eigen::Vector3d& State::magneticFieldVector() { return magnetic_field_; }
const State::CovarianceMatrix& State::covariance() const { return covar_; }
State::CovarianceMatrix& State::covariance() { return covar_; }
std::ostream& operator<<(std::ostream& os, const State& state)
{
    std::cout << "State - Attitude: " << state.attitude().unit_quaternion().w() << ' '
              << state.attitude().unit_quaternion().x() << ' '
              << state.attitude().unit_quaternion().y() << ' '
              << state.attitude().unit_quaternion().z() << '\n';
    std::cout << "State - Position: " << state.position().x() << ' ' << state.position().y() << ' '
              << state.position().z() << '\n';
    std::cout << "State - Velocity: " << state.velocity().x() << ' ' << state.velocity().y() << ' '
              << state.velocity().z() << '\n';
    std::cout << "State - Angular Velocity: " << state.angularVelocity().x() << ' '
              << state.angularVelocity().y() << ' ' << state.angularVelocity().z() << '\n';
    std::cout << "State - Acceleration: " << state.acceleration().x() << ' '
              << state.acceleration().y() << ' ' << state.acceleration().z() << '\n';
    std::cout << "State - gyroBias: " << state.gyroBias().x() << ' ' << state.gyroBias().y() << ' '
              << state.gyroBias().z() << '\n';
    std::cout << "State - accelBias: " << state.accelBias().x() << ' ' << state.accelBias().y()
              << ' ' << state.accelBias().z() << '\n';
    std::cout << "State - Variance: " << state.covariance().diagonal().transpose() << std::endl;
    return os;
}

}  // namespace gnc
}  // namespace maav
