#include <cassert>
#include <limits>
#include "lidar_localization/directional_translation_policy.hpp"

using namespace lidar_localization;

int main()
{
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
  information.diagonal() << 0.01, 0.02, 10.0, 3.0, 4.0, 5.0;
  auto metrics = analyzeTranslationConstraint(-100.0 * information, 100);
  assert(metrics.valid);
  auto prediction = Eigen::Matrix4f::Identity().eval();
  auto candidate = prediction;
  candidate.block<3, 1>(0, 3) << 0.4f, -0.3f, 0.2f;
  candidate.block<3, 3>(0, 0) = Eigen::AngleAxisf(0.04f, Eigen::Vector3f::UnitY()).toRotationMatrix();
  const auto result = previewDirectionalTranslation(metrics, prediction, candidate, 0.05);
  assert(result.valid && result.weak_directions == 2);
  assert(result.filtered_position.isApprox(Eigen::Vector3d(0, 0, 0.2), 1e-6));
  assert((candidate.block<3, 1>(0, 3) - Eigen::Vector3f(0.4f, -0.3f, 0.2f)).norm() < 1e-7);

  // Arbitrarily rotated map axes: filtering rotates with geometry, not X/Y labels.
  Eigen::Matrix3d rotation = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  Eigen::Matrix<double, 6, 6> axes = Eigen::Matrix<double, 6, 6>::Identity();
  axes.block<3, 3>(0, 0) = rotation;
  auto rotated_metrics = analyzeTranslationConstraint(-100.0 * axes * information * axes.transpose(), 100);
  auto rotated_prediction = prediction;
  rotated_prediction.block<3, 1>(0, 3) << 20, -4, 2;
  auto rotated_candidate = rotated_prediction;
  rotated_candidate.block<3, 1>(0, 3) += (rotation * candidate.block<3, 1>(0, 3).cast<double>()).cast<float>();
  const auto rotated = previewDirectionalTranslation(rotated_metrics, rotated_prediction, rotated_candidate, 0.05);
  assert(rotated.valid && rotated.weak_directions == 2);
  assert(rotated.filtered_position.isApprox(
    rotated_prediction.block<3, 1>(0, 3).cast<double>() + rotation * result.filtered_position, 1e-5));

  // Translation/rotation coupling weakens translation even when Htt looks strong.
  information.setIdentity();
  information(0, 0) = 10;
  information(3, 3) = 10;
  information(0, 3) = information(3, 0) = 9.99;
  metrics = analyzeTranslationConstraint(-information, 1);
  assert(metrics.valid && metrics.eigenvalues(0) < 0.03);
  assert(metrics.information(0, 0) < 0.03);
  auto filtered = previewDirectionalTranslation(metrics, prediction, candidate, 0.05);
  assert(filtered.valid && filtered.weak_directions == 1);
  assert(std::abs(filtered.filtered_position.x()) < 1e-8);

  // Angular units do not decide translation weakness after marginalization.
  axes.setIdentity(); axes.block<3, 3>(3, 3) *= 180.0 / std::acos(-1.0);
  const auto scaled = analyzeTranslationConstraint(-axes * information * axes.transpose(), 1);
  assert(scaled.valid && scaled.information.isApprox(metrics.information, 1e-9));

  metrics = analyzeTranslationConstraint(-Eigen::Matrix<double, 6, 6>::Identity(), 1);
  filtered = previewDirectionalTranslation(metrics, prediction, candidate, 0.05);
  assert(filtered.valid && filtered.weak_directions == 0);
  assert(filtered.filtered_position.isApprox(candidate.block<3, 1>(0, 3).cast<double>()));
  assert(!previewDirectionalTranslation(metrics, prediction, candidate, 0).valid);
  assert(!previewDirectionalTranslation(metrics, prediction, candidate, 1).valid);
  auto invalid_metrics = metrics;
  invalid_metrics.directions(0, 0) = 2;
  assert(!previewDirectionalTranslation(invalid_metrics, prediction, candidate, 0.05).valid);
  invalid_metrics = metrics;
  invalid_metrics.eigenvalues(0) = std::numeric_limits<double>::quiet_NaN();
  assert(!previewDirectionalTranslation(invalid_metrics, prediction, candidate, 0.05).valid);
  candidate(0, 3) = std::numeric_limits<float>::quiet_NaN();
  assert(!previewDirectionalTranslation(metrics, prediction, candidate, 0.05).valid);
  assert(!analyzeTranslationConstraint(-information, 0).valid);
  information(3, 3) = 0;
  assert(!analyzeTranslationConstraint(-information, 1).valid);
  information.setIdentity(); information(0, 0) = -1;
  assert(!analyzeTranslationConstraint(-information, 1).valid);
  information.setZero();
  assert(!analyzeTranslationConstraint(-information, 1).valid);
  information.setIdentity(); information(0, 0) = std::numeric_limits<double>::quiet_NaN();
  assert(!analyzeTranslationConstraint(-information, 1).valid);
}
