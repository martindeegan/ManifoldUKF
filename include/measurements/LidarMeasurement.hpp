#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <cstdint>

#include <Eigen/Dense>

#include <State.hpp>

namespace maav
{
namespace gnc
{
namespace measurements
{
/*
 * Stores the necessary info we need from a lidar message.
 */
class LidarMeasurement
{
public:
    /**
     * Necessary definitions
     */
    constexpr static size_t DoF = 1;
    using CovarianceMatrix = Eigen::Matrix<double, DoF, DoF>;
    using ErrorStateVector = Eigen::Matrix<double, DoF, 1>;
    using SensorVector = Eigen::Matrix<double, 1, 1>;

    /**
     * @brief Computes the error state between two SensorMeasurements
     */
    ErrorStateVector operator-(const LidarMeasurement& other) const;

    /**
     * @brief adds the error state into a LidarMeasurement
     */
    LidarMeasurement& operator+=(const ErrorStateVector& other);

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
    const SensorVector& distance() const;

    /**
     * @return Mutable reference to the measured value
     */
    SensorVector& distance();

    /**
     * @brief Computes the mean and covariance of a set of SensorMeasurements
     * @param points Sigma points from an UnscentedTransform
     * @param m_weights Mean weights from an UnscentedTransform
     * @param c_weights Covariance weights from an UnscentedTransform
     */
    static LidarMeasurement compute_gaussian(const std::array<LidarMeasurement, State::N>& points,
        const std::array<double, State::N>& m_weights,
        const std::array<double, State::N>& c_weights)
    {
        LidarMeasurement gaussian;
        gaussian.distance() = SensorVector::Zero();
        for (size_t i = 0; i < State::N; i++)
        {
            gaussian += m_weights[i] * points[i].distance();
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
    uint64_t time_usec_;
    SensorVector distance_;
    CovarianceMatrix covariance_;
};

std::ostream& operator<<(std::ostream& os, const LidarMeasurement& meas);
}  // namespace measurements
}  // namespace gnc
}  // namespace maav

#endif
