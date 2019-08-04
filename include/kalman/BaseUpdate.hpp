#pragma once

#include <yaml-cpp/node/detail/bool_type.h>
#include <yaml-cpp/yaml.h>

#include <yaml_matrix/yaml_matrix.hpp>
#include <kalman/Extrinsics.hpp>
#include <kalman/History.hpp>
#include <kalman/UnscentedTransform.hpp>
#include <measurements/Measurement.hpp>

using std::placeholders::_1;

namespace maav
{
namespace gnc
{
namespace kalman
{
/**
 * TargetSpace is some space where we compare our prediction ot our sensor measurements.
 * Note that TargetSpace doesn't have to be a sensor measured.
 */
template <class TargetSpace>
class BaseUpdate
{
private:
    using UT = UnscentedTransform<TargetSpace>;
    constexpr static size_t TargetDoF = TargetSpace::DoF;
    using CovarianceMatrix = typename TargetSpace::CovarianceMatrix;
    using ErrorStateVector = typename TargetSpace::ErrorStateVector;
    using CrossCovarianceMatrix = Eigen::Matrix<double, State::DoF, TargetDoF>;
    using KalmanGainMatrix = CrossCovarianceMatrix;

public:
    BaseUpdate(YAML::Node config)
        : enabled_(config["enabled"].as<bool>()),
          enable_outliers_(config["enable_outliers"].as<bool>()),
          unscented_transform_(config["UT"]),
          extrinsics_(config["extrinsics"])
    {
        Eigen::Matrix<double, TargetDoF, 1> R_diag =
            config["R"].as<Eigen::Matrix<double, TargetDoF, 1>>();
        R_ = Eigen::DiagonalMatrix<double, TargetDoF>(R_diag);
        unscented_transform_.set_transformation(std::bind(&BaseUpdate::predicted, this, _1));
    }

    bool enabled() { return enabled_; }

protected:
    /**
     * Nonlinear h function.
     * Maps the current state to some element of the target space
     */
    virtual TargetSpace predicted(const State& state) = 0;

    /**
     * Maps the actual sensor measurements to some element of the target space.
     */
    virtual TargetSpace measured(const measurements::Measurement& meas) = 0;

    // Outlier protection. Bad data association causes the filter to diverge quickly.
    // Discard any measurement more than 3 standard deviations away from the estimate.
    bool rejectOutlier(const typename TargetSpace::ErrorStateVector& residual,
        const typename TargetSpace::CovarianceMatrix& covariance)
    {
        double mahl_dist = std::sqrt((residual.transpose() * covariance.inverse() * residual)(0));
        return mahl_dist > 5;
    }

    /**
     * Updates the state based on the measurements in a snapshot.
     */
    void correct(History::Snapshot& snapshot)
    {
        if (!enabled_) return;

        State& state = snapshot.state;

        const TargetSpace predicted_meas = unscented_transform_(extrinsics_(state));
        const TargetSpace measured_mes = measured(snapshot.measurement);
        ErrorStateVector residual = measured_mes - predicted_meas;
        const CovarianceMatrix S = predicted_meas.covariance() + R_;

        // Reject outliers
        if (rejectOutlier(residual, S) && enable_outliers_)
        {
            std::cout << "WARNING: Outlier rejected." << std::endl;
            return;
        }

        // Extract necessary variables from the UT
        const typename UT::Weights& c_weights = unscented_transform_.c_weights();
        const typename UT::SigmaPoints& sigma_points = unscented_transform_.last_sigma_points();
        const typename UT::TransformedPoints& transformed_points =
            unscented_transform_.last_transformed_points();

        CrossCovarianceMatrix Sigma_x_z = CrossCovarianceMatrix::Zero();
        for (size_t i = 0; i < UnscentedTransform<TargetSpace>::N; i++)
        {
            Sigma_x_z += c_weights[i] * (sigma_points[i] - state) *
                         (transformed_points[i] - predicted_meas).transpose();
        }
        const KalmanGainMatrix K = Sigma_x_z * S.inverse();
        // Update the state gaussian in place
        state += K * residual;
        state.covariance() -= K * S * K.transpose();
    }

private:
    bool enabled_;
    bool enable_outliers_;
    UT unscented_transform_;
    CovarianceMatrix R_;

protected:
    Extrinsics extrinsics_;
};
}  // namespace kalman
}  // namespace gnc
}  // namespace maav
