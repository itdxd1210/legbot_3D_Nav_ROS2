#include "lidar_localization/alignment_attempt_policy.hpp"
#include "lidar_localization/localization_update_policy.hpp"
#include "lidar_localization/prediction_state_policy.hpp"
#include "lidar_localization/alignment_failure_taxonomy.hpp"

#include <cassert>
#include <limits>

namespace ll = lidar_localization;

int main()
{
  const float radians = 3.14159265358979323846f / 180.0f;
  Eigen::Matrix4f prediction = Eigen::Matrix4f::Identity();
  // Real stair pitch and a rotated corridor must not trip a correction gate.
  prediction.block<3, 3>(0, 0) =
    (Eigen::AngleAxisf(0.8f, Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(25.0f * radians, Eigen::Vector3f::UnitY())).toRotationMatrix();
  prediction(0, 3) = 12.0f;
  prediction(2, 3) = 3.0f;
  Eigen::Matrix4f extra = Eigen::Matrix4f::Identity();
  extra.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(7.5f * radians, Eigen::Vector3f::UnitY()).toRotationMatrix();
  const auto bad = ll::computeAlignmentCorrectionMetrics(prediction, prediction * extra);
  assert(bad.pose_valid);
  assert(std::abs(bad.rotation_deg - 7.5) < 1e-3);
  assert(std::abs(bad.pitch_deg - 7.5) < 1e-3);
  assert(bad.yaw_deg < 1e-3);

  ll::MeasurementGateParams params;
  params.enable_odom_tf_prediction_correction_guard = true;
  params.odom_tf_prediction_correction_guard_rotation_deg = 5.0;
  params.enable_rejected_seed_update = true;
  params.enable_consistency_recovery_gate = true;
  params.enable_odom_tf_prediction_recovery_correction_guard = true;
  auto input = ll::makeMeasurementGateInput(0.02, 0.1, 0.02, bad.translation_m,
    bad.yaw_deg, 100, true, bad.rotation_deg, bad.pose_valid);
  const auto gate = ll::evaluateMeasurementGate(params, input);
  assert(gate.reject_measurement);
  assert(!gate.rejected_seed_update_applied);
  assert(gate.status_message == "odom_tf_prediction_rotation_guard_rejected");
  ll::AlignmentFailureTaxonomyInput diagnostic;
  diagnostic.map_received = diagnostic.initialpose_received = diagnostic.has_converged = true;
  diagnostic.filtered_point_count = 1000;
  diagnostic.status_message = gate.status_message;
  diagnostic.fitness_score = 0.02;
  diagnostic.effective_score_threshold = 2.0;
  assert(ll::classifyAlignmentFailure({}, diagnostic).bad_match_active);
  const auto decision = ll::decideLocalizationUpdate(gate);
  assert(decision.advance_prediction_without_measurement);
  assert(!decision.update_current_pose);
  assert(!decision.update_prediction_from_rejected_measurement);

  const auto trusted = ll::resetPredictionState(prediction, 10.0);
  const auto held = ll::advancePredictionWithoutMeasurement(
    trusted, 10.1, ll::PredictionAdvanceMode::kNone);
  assert(held.last_accepted_pose_matrix.isApprox(trusted.last_accepted_pose_matrix));
  assert(held.last_accepted_pose_time_sec == 10.0);
  assert(held.predicted_pose_matrix.isApprox(trusted.predicted_pose_matrix));
  // Holding the anchor does not hold the LIO pose: the next odom delta moves it.
  Eigen::Matrix4f odom_delta = Eigen::Matrix4f::Identity();
  odom_delta(0, 3) = 0.2f;
  assert(!(held.last_accepted_pose_matrix * odom_delta).isApprox(prediction));

  const auto good = ll::computeAlignmentCorrectionMetrics(prediction, prediction);
  input.correction_rotation_deg = good.rotation_deg;
  assert(!ll::evaluateMeasurementGate(params, input).reject_measurement);
  input.correction_rotation_deg = 4.3;
  assert(!ll::evaluateMeasurementGate(params, input).reject_measurement);
  input.pose_metrics_valid = false;
  assert(ll::evaluateMeasurementGate(params, input).reject_measurement);
  input.pose_metrics_valid = true;
  input.fitness_score = std::numeric_limits<double>::quiet_NaN();
  assert(ll::evaluateMeasurementGate(params, input).reject_measurement);
  Eigen::Matrix4f malformed = prediction;
  malformed(0, 0) = std::numeric_limits<float>::quiet_NaN();
  assert(!ll::computeAlignmentCorrectionMetrics(prediction, malformed).pose_valid);
  malformed = prediction;
  malformed.block<3, 3>(0, 0) *= 2.0f;
  assert(!ll::computeAlignmentCorrectionMetrics(prediction, malformed).pose_valid);
  return 0;
}
