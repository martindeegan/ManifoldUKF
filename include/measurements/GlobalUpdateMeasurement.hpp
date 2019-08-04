#ifndef GLOBAL_UPDATE_MEASUREMENT
#define GLOBAL_UPDATE_MEASUREMENT

#include <ostream>

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <State.hpp>

namespace manifold_ukf
{
namespace gnc
{
namespace measurements
{
/*
 * Stores the necessary info we need from a global update (SLAM) measurement
 */
class GlobalUpdateMeasurement
{
public:
    /**
     * Necessary definitions
     */
    constexpr static size_t DoF = 6;
    using CovarianceMatrix = Eigen::Matrix<double, DoF, DoF>;
    using ErrorStateVector = Eigen::Matrix<double, DoF, 1>;

    /**
     * @brief Computes the error state between two SensorMeasurements
     */
    ErrorStateVector operator-(const GlobalUpdateMeasurement& other) const;

    /**
     * @brief adds the error state into a GlobalUpdateMeasurement
     */
    GlobalUpdateMeasurement& operator+=(const ErrorStateVector& other);

    /**
     * @return Const reference to the covariance matrix
     */
    const CovarianceMatrix& covariance() const;

    /**
     * @return Mutable reference to the covariance matrix
     */
    CovarianceMatrix& covariance();

    /**
     * @return Const reference to the measured value
     */
    const Sophus::SE3d& pose() const;

    /**
     * @return Mutable reference to the measured value
     */
    Sophus::SE3d& pose();

    /**
     * @brief Computes the mean and covariance of a set of SensorMeasurements
     * @param points Sigma points from an UnscentedTransform
     * @param m_weights Mean weights from an UnscentedTransform
     * @param c_weights Covariance weights from an UnscentedTransform
     */
    static GlobalUpdateMeasurement compute_gaussian(
        const std::array<GlobalUpdateMeasurement, State::N>& points,
        const std::array<double, State::N>& m_weights,
        const std::array<double, State::N>& c_weights)
    {
        GlobalUpdateMeasurement gaussian;
        gaussian.pose().so3() = points[0].pose().so3();
        for (size_t i = 0; i < State::N; i++)
        {
            gaussian.pose().translation() += m_weights[i] * points[i].pose().translation();
        }

        gaussian.covariance() = CovarianceMatrix::Zero();
        for (size_t i = 0; i < State::N; i++)
        {
            gaussian.covariance() +=
                c_weights[i] * (points[i] - gaussian) * (points[i] - gaussian).transpose();
        }

        return gaussian;
    }

    uint64_t timeUSec() const;
    void setTime(uint64_t time_usec);

private:
    Sophus::SE3d pose_;
    uint64_t time_usec_;
    CovarianceMatrix covariance_;
};

std::ostream& operator<<(std::ostream& os, const GlobalUpdateMeasurement& meas);

}  // namespace measurements
}  // namespace gnc
}  // namespace manifold_ukf

#endif
