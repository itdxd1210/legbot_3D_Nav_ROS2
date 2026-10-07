#pragma once

#include <string>
#include <Eigen/Eigenvalues>
#include "lidar_localization/alignment_attempt_policy.hpp"

namespace lidar_localization
{

struct TranslationConstraintMetrics
{
  bool valid{false};
  std::string status{"disabled"};
  Eigen::Matrix<double, 6, 6> score_information{
    Eigen::Matrix<double, 6, 6>::Constant(NAN)};
  // NDT additive translation parameters are target/map axes, not body axes.
  Eigen::Matrix3d information{Eigen::Matrix3d::Constant(NAN)};
  Eigen::Vector3d eigenvalues{Eigen::Vector3d::Constant(NAN)};
  Eigen::Matrix3d directions{Eigen::Matrix3d::Constant(NAN)};
};

inline TranslationConstraintMetrics analyzeTranslationConstraint(
  const Eigen::Matrix<double, 6, 6> & score_hessian, std::size_t correspondences)
{
  TranslationConstraintMetrics result;
  if (!correspondences || !score_hessian.allFinite()) {
    result.status = "invalid_hessian_or_empty_correspondences";
    return result;
  }
  const Eigen::Matrix<double, 6, 6> h =
    -0.5 * (score_hessian + score_hessian.transpose()) / static_cast<double>(correspondences);
  result.score_information = h;
  const Eigen::Matrix3d rr = h.block<3, 3>(3, 3);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rotation_solver(rr);
  if (rotation_solver.info() != Eigen::Success || !rotation_solver.eigenvalues().allFinite() ||
    rotation_solver.eigenvalues().maxCoeff() <= 0 ||
    rotation_solver.eigenvalues().minCoeff() <= rotation_solver.eigenvalues().maxCoeff() * 1e-9)
  {
    result.status = "angular_block_not_positive_or_singular";
    return result;
  }
  const Eigen::Matrix3d inverse_rr = rotation_solver.eigenvectors() *
    rotation_solver.eigenvalues().cwiseInverse().asDiagonal() * rotation_solver.eigenvectors().transpose();
  // Schur complement eliminates angular nuisance variables. Comparing this
  // 3x3 spectrum does not mix translation metres with Euler-angle radians.
  const Eigen::Matrix3d tr = h.block<3, 3>(0, 3);
  const Eigen::Matrix3d marginal = h.block<3, 3>(0, 0) - tr * inverse_rr * tr.transpose();
  result.information = 0.5 * (marginal + marginal.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(result.information);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
    solver.eigenvalues().maxCoeff() <= 0 ||
    solver.eigenvalues().minCoeff() < -solver.eigenvalues().maxCoeff() * 1e-8)
  {
    result.status = "translation_information_not_positive_semidefinite";
    return result;
  }
  result.eigenvalues = solver.eigenvalues().cwiseMax(0.0);
  result.directions = solver.eigenvectors();
  result.valid = true;
  result.status = "ok";
  return result;
}

struct DirectionalTranslationPreview
{
  bool valid{false};
  int weak_directions{0};
  double min_eigen_ratio{NAN};
  Eigen::Vector3d gains{Eigen::Vector3d::Constant(NAN)};
  Eigen::Vector3d original_delta{Eigen::Vector3d::Constant(NAN)};
  Eigen::Vector3d retained_delta{Eigen::Vector3d::Constant(NAN)};
  Eigen::Vector3d filtered_position{Eigen::Vector3d::Constant(NAN)};
};

inline DirectionalTranslationPreview previewDirectionalTranslation(
  const TranslationConstraintMetrics & metrics, const Eigen::Matrix4f & prediction,
  const Eigen::Matrix4f & candidate, double min_eigen_ratio)
{
  DirectionalTranslationPreview result;
  result.min_eigen_ratio = min_eigen_ratio;
  if (!metrics.valid || !std::isfinite(min_eigen_ratio) || min_eigen_ratio <= 0 ||
    min_eigen_ratio >= 1 || !metrics.eigenvalues.allFinite() ||
    metrics.eigenvalues.maxCoeff() <= 0 || metrics.eigenvalues.minCoeff() < 0 ||
    !metrics.directions.allFinite() ||
    !(metrics.directions.transpose() * metrics.directions).isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
    !isFiniteRigidPose(prediction) || !isFiniteRigidPose(candidate))
  {
    return result;
  }
  const Eigen::Vector3d ratios = metrics.eigenvalues / metrics.eigenvalues.maxCoeff();
  result.original_delta = (candidate.block<3, 1>(0, 3) - prediction.block<3, 1>(0, 3)).cast<double>();
  for (int i = 0; i < 3; ++i) {
    result.gains(i) = ratios(i) < min_eigen_ratio ? 0.0 : 1.0;
    result.weak_directions += result.gains(i) == 0;
  }
  result.retained_delta = metrics.directions * result.gains.asDiagonal() *
    metrics.directions.transpose() * result.original_delta;
  result.filtered_position = prediction.block<3, 1>(0, 3).cast<double>() + result.retained_delta;
  result.valid = result.filtered_position.allFinite();
  return result;
}

}  // namespace lidar_localization
