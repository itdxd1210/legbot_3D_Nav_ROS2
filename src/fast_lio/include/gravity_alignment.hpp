#pragma once
#include <Eigen/Geometry>
#include <stdexcept>

namespace fastlio_init {
// Stationary accelerometer measures specific force opposite to gravity.
// Return IMU -> level-odom rotation. Yaw is an arbitrary local convention,
// NOT geographic heading or map localization. No sensor extrinsics change.
inline Eigen::Quaterniond gravityAlignedRotation(const Eigen::Vector3d &mean_acc)
{
    if (!mean_acc.allFinite() || mean_acc.norm() < 1e-6)
        throw std::invalid_argument("Invalid gravity initialization acceleration");
    return Eigen::Quaterniond::FromTwoVectors(
        mean_acc.normalized(), Eigen::Vector3d::UnitZ()).normalized();
}
}  // namespace fastlio_init
