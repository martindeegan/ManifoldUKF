#pragma once

#include <memory>

#include <measurements/GlobalUpdateMeasurement.hpp>
#include <measurements/ImuMeasurement.hpp>
#include <measurements/LidarMeasurement.hpp>
#include <measurements/PlaneFitMeasurement.hpp>
#include <measurements/VisualOdometry.hpp>

namespace maav
{
namespace gnc
{
namespace measurements
{
/**
 * The Measurement class holds a group of measurements that are meant to be
 * together (so, they all happened at the same time).
 */
struct Measurement
{
    std::shared_ptr<const ImuMeasurement> imu;
    std::shared_ptr<const LidarMeasurement> lidar;
    std::shared_ptr<const PlaneFitMeasurement> plane_fit;
    std::shared_ptr<const VisualOdometryMeasurement> visual_odometry;
    std::shared_ptr<const GlobalUpdateMeasurement> global_update;
};

/*
 * The MeasurementSet class is the same as the Measurement class.
 * The reason we give it a different name is that it is used for a different
 * purpose. Where the Measurement class is meant to represent all the
 * measurements that we have from a particular moment in time, the
 * MeasurementSet class is used to hold a collection of measurements that may
 * not have happened at the same moment, mostly for convenience so we don't
 * have to split up measurements into many arguments.
 */
using MeasurementSet = Measurement;

}  // namespace measurements
}  // namespace gnc
}  // namespace maav
