#ifndef PLANEFIT_UPDATE_HPP
#define PLANEFIT_UPDATE_HPP

#include <yaml-cpp/yaml.h>

#include <kalman/BaseUpdate.hpp>
#include <measurements/PlaneFitMeasurement.hpp>

namespace manifold_ukf
{
namespace gnc
{
namespace kalman
{
/**
 * TODO: Combine this with manifold_ukf::gnc::measurements::PlaneFit
 * Represents the TargetSpace of the lidar update
 */
class PFSensorMeasurement
{
public:
    /**
     * Necessary definitions
     */
    constexpr static size_t DoF = 4;
    using CovarianceMatrix = Eigen::Matrix<double, DoF, DoF>;
    using ErrorStateVector = Eigen::Matrix<double, DoF, 1>;
    using SensorVector = Eigen::Matrix<double, DoF, 1>;

    /**
     * @brief Computes the error state between two SensorMeasurements
     */
    ErrorStateVector operator-(const PFSensorMeasurement& other) const;

    /**
     * @brief adds the error state into a PFSensorMeasurement
     */
    PFSensorMeasurement& operator+=(const ErrorStateVector& other);

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
    const SensorVector& readings() const;

    /**
     * @return Mutable reference to the measured value
     */
    SensorVector& readings();

    /**
     * @brief Computes the mean and covariance of a set of SensorMeasurements
     * @param points Sigma points from an UnscentedTransform
     * @param m_weights Mean weights from an UnscentedTransform
     * @param c_weights Covariance weights from an UnscentedTransform
     */
    static PFSensorMeasurement compute_gaussian(
        const std::array<PFSensorMeasurement, State::N>& points,
        const std::array<double, State::N>& m_weights,
        const std::array<double, State::N>& c_weights)
    {
        PFSensorMeasurement gaussian;
        gaussian.readings() = SensorVector::Zero();
        for (size_t i = 0; i < State::N; i++)
        {
            gaussian += m_weights[i] * points[i].readings();
        }

        gaussian.covariance() = CovarianceMatrix::Zero();
        for (size_t i = 0; i < State::N; i++)
        {
            gaussian.covariance() +=
                c_weights[i] * (points[i] - gaussian) * (points[i] - gaussian).transpose();
        }

        return gaussian;
    }

private:
    SensorVector readings_;
    CovarianceMatrix covariance_;
};

class PlaneFitUpdate : public BaseUpdate<PFSensorMeasurement>
{
public:
    /**
     *  @param config This yaml node must have a "planefit" key with unscented transform parameters
     * and
     * a sensor covariance matrix, R
     */
    PlaneFitUpdate(YAML::Node config);

    /**
     * @breif Computes the observation model (h(x)) for a state provided by an UnscentedTransform
     * @param state A sigma point proved by an UnscentedTransform
     * @return The predicted lidar observation
     */
    PFSensorMeasurement predicted(const State& state);

    /**
     * @param meas
     * @return The relevant lidar measurement from the list of measurements
     */
    PFSensorMeasurement measured(const measurements::Measurement& meas);

    /**
     * @brief Performs the correction step for a lidar
     * @param snapshot A mutable reference to a point in time. The state will be updated according
     * to the sensor model
     */
    void operator()(History::Snapshot& snapshot);

private:
    using BaseUpdate<PFSensorMeasurement>::correct;
};
}  // namespace kalman
}  // namespace gnc
}  // namespace manifold_ukf

#endif