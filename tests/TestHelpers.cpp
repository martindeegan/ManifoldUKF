
#include <cmath>

#include "TestHelpers.hpp"

double diff(const Sophus::SO3d& r1, const Sophus::SO3d& r2)
{
    const Sophus::SO3d err = r1.inverse() * r2;
    const Eigen::Quaterniond& q = err.unit_quaternion();
    return std::acos(std::abs(q.w()));
}

double diff(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2)
{
    return diff(Sophus::SO3d(q1), Sophus::SO3d(q2));
}

double diff(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
    const Eigen::Vector3d err = v1 - v2;
    return err.norm();
}
