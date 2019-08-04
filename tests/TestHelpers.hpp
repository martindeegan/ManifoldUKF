#pragma once

#include <Eigen/Eigen>
#include <sophus/so3.hpp>

/**
 * Returns the error in radians between two rotations
 */
double diff(const Sophus::SO3d& r1, const Sophus::SO3d& r2);

/**
 * Returns the error in radians between the two quaternions
 */
double diff(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);

/**
 * Returns the error (norm) between two vectors
 */
double diff(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
