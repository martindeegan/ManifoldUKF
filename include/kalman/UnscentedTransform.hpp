#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <Eigen/Cholesky>

#include <State.hpp>

namespace manifold_ukf
{
namespace gnc
{
namespace kalman
{
template <class TargetSpace>
class UnscentedTransform
{
public:
    constexpr static size_t N = 1 + 2 * State::DoF;

    using Transform = std::function<TargetSpace(const State&)>;
    using SigmaPoints = std::array<State, N>;
    using TransformedPoints = std::array<TargetSpace, N>;
    using Weights = std::array<double, N>;

    UnscentedTransform() = default;

    UnscentedTransform(Transform transform, double alpha, double beta, double kappa)
        : _transformation(transform)
    {
        set_parameters(alpha, beta, kappa);
    }

    UnscentedTransform(YAML::Node config)
    {
        double alpha = config["alpha"].as<double>();
        double beta = config["beta"].as<double>();
        double kappa = config["kappa"].as<double>();

        set_parameters(alpha, beta, kappa);
    }
    /**
     * Transforms the gaussian from the state space to a gaussian in some target
     * space
     */
    TargetSpace operator()(const State& state);

    void set_parameters(double alpha, double beta, double kappa);

    void set_transformation(Transform transform);

private:
    TransformedPoints _transformed_points;
    SigmaPoints _sigma_points;

    void generate_sigma_points(const State& state);

private:
    Transform _transformation;

    double _lambda;
    double _alpha;
    double _beta;
    double _kappa;

    Weights _m_weights;
    Weights _c_weights;

public:
    const Weights& m_weights() const { return _m_weights; }
    const Weights& c_weights() const { return _c_weights; }
    const SigmaPoints& last_sigma_points() const { return _sigma_points; }
    const TransformedPoints& last_transformed_points() const { return _transformed_points; }
};

template <class TargetSpace>
TargetSpace UnscentedTransform<TargetSpace>::operator()(const State& state)
{
    generate_sigma_points(state);
    // Pass all sigma points through the transform
    std::transform(
        _sigma_points.begin(), _sigma_points.end(), _transformed_points.begin(), _transformation);

    // Recompute the mean and covariance
    TargetSpace result = TargetSpace::compute_gaussian(_transformed_points, _m_weights, _c_weights);
    return result;
}

template <class TargetSpace>
void UnscentedTransform<TargetSpace>::generate_sigma_points(const State& state)
{
    Eigen::LLT<State::CovarianceMatrix> decomp(
        (static_cast<double>(State::DoF) + _lambda) * state.covariance());
    const State::CovarianceMatrix L = decomp.matrixL();

    // Set the starting point to the current mean
    _sigma_points[0] = state;
    for (size_t i = 0; i < State::DoF; i++)
    {
        const State::ErrorStateVector& sigma_point_offset = L.col(i);

        State additive_point = state;
        State subtractive_point = state;

        additive_point += sigma_point_offset;
        subtractive_point += (-1) * sigma_point_offset;

        _sigma_points[2 * i + 1] = additive_point;
        _sigma_points[2 * i + 2] = subtractive_point;
    }
}

template <class TargetSpace>
void UnscentedTransform<TargetSpace>::set_parameters(double alpha, double beta, double kappa)
{
    _alpha = alpha;
    _beta = beta;
    _kappa = kappa;

    constexpr auto n_d = static_cast<double>(State::DoF);

    _lambda = _alpha * _alpha * (n_d + _kappa) - n_d;
    const double w_m_0 = _lambda / (n_d + _lambda);
    const double w_c_0 = w_m_0 + (1 - (_alpha * _alpha) + _beta);
    const double w = 1 / (2 * (n_d + _lambda));

    _m_weights.fill(w);
    _c_weights.fill(w);
    _m_weights[0] = w_m_0;
    _c_weights[0] = w_c_0;
}

template <class TargetSpace>
void UnscentedTransform<TargetSpace>::set_transformation(Transform transform)
{
    _transformation = transform;
}

}  // namespace kalman
}  // namespace gnc
}  // namespace manifold_ukf
