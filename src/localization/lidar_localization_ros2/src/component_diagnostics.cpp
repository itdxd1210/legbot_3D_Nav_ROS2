#include "component_internal.hpp"
#include <iomanip>
#include <sstream>
void PCLLocalization::publishAlignmentStatusForAttempt(
  const builtin_interfaces::msg::Time & stamp,
  uint8_t level,
  const std::string & message,
  const lidar_localization::AlignmentAttempt & attempt,
  std::size_t filtered_point_count,
  bool imu_prediction_active,
  const std::string & registration_seed_source)
{
  publishAlignmentStatus(
    stamp,
    level,
    message,
    attempt.has_converged,
    attempt.fitness_score,
    attempt.alignment_time_sec,
    filtered_point_count,
    attempt.correction_translation_m,
    attempt.correction_yaw_deg,
    attempt.seed_translation_since_accept_m,
    attempt.seed_yaw_since_accept_deg,
    attempt.accepted_gap_sec,
    imu_prediction_active,
    registration_seed_source,
    attempt.registration_localizability,
    &attempt);
}

void PCLLocalization::publishAlignmentStatus(
  const builtin_interfaces::msg::Time & stamp,
  uint8_t level,
  const std::string & message,
  bool has_converged,
  double fitness_score,
  double alignment_time_sec,
  std::size_t filtered_point_count,
  double correction_translation_m,
  double correction_yaw_deg,
  double seed_translation_since_accept_m,
  double seed_yaw_since_accept_deg,
  double accepted_gap_sec,
  bool imu_prediction_active,
  const std::string & registration_seed_source,
  const lidar_localization::RegistrationLocalizabilityMetrics & registration_localizability,
  const lidar_localization::AlignmentAttempt * attempt)
{
  if (!status_pub_) {return;}

  const AlignmentStatusPublishInput publish_input{
    stamp,
    level,
    message,
    has_converged,
    fitness_score,
    alignment_time_sec,
    filtered_point_count,
    correction_translation_m,
    correction_yaw_deg,
    seed_translation_since_accept_m,
    seed_yaw_since_accept_deg,
    accepted_gap_sec,
    imu_prediction_active,
    registration_seed_source,
    registration_localizability};

  const auto evaluation = evaluateAlignmentStatus(stamp, publish_input);
  auto status = makeAlignmentDiagnosticStatus(evaluation.status_input);

  appendAlignmentDiagnosticValues(
    status,
    prepareAlignmentDiagnosticValuesInput(
      evaluation.status_input,
      evaluation.status_preparation,
      evaluation.reinitialization_request));
  if (attempt) {
    auto append = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(item);
      };
    auto scalar = [&append](const std::string & key, double value) {
        append(key, std::to_string(value));
      };
    append("state_phase", "before_backend_commit");
    append("pose_metrics_valid", lidar_localization::boolString(attempt->pose_metrics_valid));
    append("correction_xyz_frame", "prediction_base_axes");
    scalar("correction_rotation_deg", attempt->correction_rotation_deg);
    scalar("correction_roll_deg", attempt->correction_roll_deg);
    scalar("correction_pitch_deg", attempt->correction_pitch_deg);
    scalar("rotation_guard_limit_deg",
      measurement_gate_config_.odom_tf_prediction_correction_guard_rotation_deg);
    scalar("trusted_pose_stamp_before", last_accepted_pose_time_sec_);
    scalar("scan_stamp_sec", attempt->scan_stamp_sec);
    append("direction_shadow_enabled", lidar_localization::boolString(enable_directional_constraint_shadow_));
    if (enable_directional_constraint_shadow_) {
      const auto & constraint = attempt->translation_constraint;
      const auto & preview = attempt->directional_translation_preview;
      append("direction_shadow_frame", global_frame_id_);
      append("direction_shadow_status", constraint.status);
      append("direction_shadow_valid", lidar_localization::boolString(preview.valid));
      append("direction_shadow_control_applied", "false");
      append("direction_shadow_baseline_accepted", lidar_localization::boolString(message == "ok"));
      scalar("direction_shadow_min_eigen_ratio", directional_constraint_min_eigen_ratio_);
      scalar("direction_shadow_weak_directions", preview.weak_directions);
      scalar("direction_shadow_removed_translation_m", preview.valid ?
        (preview.original_delta - preview.retained_delta).norm() : NAN);
      static constexpr const char * translation_axes[] = {"x", "y", "z"};
      for (int i = 0; i < 3; ++i) {
        scalar("direction_shadow_eigenvalue_" + std::to_string(i), constraint.eigenvalues(i));
        scalar("direction_shadow_gain_" + std::to_string(i), preview.gains(i));
        scalar(std::string("direction_shadow_position_") + translation_axes[i], preview.filtered_position(i));
        scalar(std::string("direction_shadow_retained_") + translation_axes[i], preview.retained_delta(i));
        for (int j = 0; j < 3; ++j) {
          scalar("direction_shadow_basis_" + std::to_string(i) + "_" + std::to_string(j),
            constraint.directions(i, j));
        }
      }
      // All curvature entries permit offline threshold sweeps and checks of
      // angular coupling; six-digit rounding is avoided for small curvatures.
      for (int i = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
          std::ostringstream value;
          value << std::setprecision(12) << constraint.score_information(i, j);
          append("direction_shadow_information_" + std::to_string(i) + "_" + std::to_string(j), value.str());
        }
      }
    }
    append("odom_recovery_review_enabled", lidar_localization::boolString(odom_recovery_review_config_.enabled));
    append("odom_recovery_review_candidate", lidar_localization::boolString(odom_recovery_review_decision_.candidate));
    append("odom_recovery_review_confirmed", lidar_localization::boolString(odom_recovery_review_decision_.confirmed));
    scalar("odom_recovery_review_frames", odom_recovery_review_decision_.frames);
    scalar("odom_recovery_review_position_spread_m", odom_recovery_review_decision_.position_spread_m);
    scalar("odom_recovery_review_rotation_spread_deg", odom_recovery_review_decision_.rotation_spread_deg);
    append("odom_candidate_review_enabled", lidar_localization::boolString(odom_candidate_review_config_.enabled));
    append("odom_candidate_review_candidate", lidar_localization::boolString(odom_candidate_review_decision_.candidate));
    append("odom_candidate_review_confirmed", lidar_localization::boolString(odom_candidate_review_decision_.confirmed));
    scalar("odom_candidate_review_frames", odom_candidate_review_decision_.frames);
    scalar("odom_candidate_review_span_sec", odom_candidate_review_decision_.observation_span_sec);
    scalar("odom_candidate_review_required_frames", odom_candidate_review_config_.frames);
    scalar("odom_candidate_review_required_span_sec", odom_candidate_review_config_.min_observation_span_sec);
    scalar("odom_candidate_review_position_spread_m", odom_candidate_review_decision_.position_spread_m);
    scalar("odom_candidate_review_rotation_spread_deg", odom_candidate_review_decision_.rotation_spread_deg);
    const auto & anchor = last_good_map_to_odom_.transform;
    scalar("anchor_before_x", anchor.translation.x);
    scalar("anchor_before_y", anchor.translation.y);
    scalar("anchor_before_z", anchor.translation.z);
    scalar("anchor_before_qx", anchor.rotation.x);
    scalar("anchor_before_qy", anchor.rotation.y);
    scalar("anchor_before_qz", anchor.rotation.z);
    scalar("anchor_before_qw", anchor.rotation.w);
    static constexpr const char * axes[] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
      scalar(std::string("correction_") + axes[i] + "_m", attempt->correction_xyz_m(i));
      scalar(std::string("prediction_") + axes[i], attempt->prediction_reference(i, 3));
      scalar(std::string("candidate_") + axes[i],
        attempt->has_converged ? attempt->final_transformation(i, 3) : NAN);
      scalar(std::string("trusted_before_") + axes[i], last_accepted_pose_matrix_(i, 3));
    }
    const Eigen::Quaternionf predicted_q(attempt->prediction_reference.block<3, 3>(0, 0));
    const Eigen::Quaternionf candidate_q(attempt->final_transformation.block<3, 3>(0, 0));
    static constexpr const char * quaternion_axes[] = {"qx", "qy", "qz", "qw"};
    for (int i = 0; i < 4; ++i) {
      scalar(std::string("prediction_") + quaternion_axes[i], predicted_q.coeffs()(i));
      scalar(std::string("candidate_") + quaternion_axes[i],
        attempt->has_converged ? candidate_q.coeffs()(i) : NAN);
    }
  }
  publishAlignmentDiagnosticStatus(stamp, status);
  publishReinitializationRequest(stamp, evaluation.reinitialization_request);
}

lidar_localization::AlignmentStatusInput PCLLocalization::makeAlignmentStatusInput(
  const AlignmentStatusPublishInput & input) const
{
  const double stamp_sec = stamp_to_sec(input.stamp);
  return lidar_localization::makeAlignmentStatusInput(
    lidar_localization::AlignmentStatusObservation{
      input.level,
      input.message,
      input.has_converged,
      input.fitness_score,
      input.alignment_time_sec,
      input.filtered_point_count,
      input.correction_translation_m,
      input.correction_yaw_deg,
      input.seed_translation_since_accept_m,
      input.seed_yaw_since_accept_deg,
      input.accepted_gap_sec,
      input.imu_prediction_active,
      input.registration_seed_source,
      input.registration_localizability},
    makeAlignmentStatusRuntimeContext(stamp_sec));
}

lidar_localization::AlignmentStatusRuntimeContext
PCLLocalization::makeAlignmentStatusRuntimeContext(double stamp_sec) const
{
  const auto fallback_seed_metrics = lidar_localization::computeAlignmentSeedMetrics(
    have_last_accepted_pose_,
    last_accepted_pose_matrix_,
    predicted_pose_matrix_,
    stamp_sec,
    last_accepted_pose_time_sec_);

  return lidar_localization::AlignmentStatusRuntimeContext{
    registration_method_,
    consecutive_rejected_updates_,
    have_last_accepted_pose_,
    stamp_sec,
    last_accepted_pose_time_sec_,
    fallback_seed_metrics.translation_since_accept_m,
    map_recieved_,
    initialpose_recieved_,
    measurementGateParams(),
    reinitializationTriggerParams(),
    failure_taxonomy_params_};
}

diagnostic_msgs::msg::DiagnosticStatus PCLLocalization::makeAlignmentDiagnosticStatus(
  const lidar_localization::AlignmentStatusInput & status_input) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = status_input.level;
  status.name = "lidar_localization_ros2/alignment";
  status.message = status_input.message;
  status.hardware_id = status_input.registration_method;
  return status;
}

PCLLocalization::AlignmentStatusEvaluation PCLLocalization::evaluateAlignmentStatus(
  const builtin_interfaces::msg::Time & stamp,
  const AlignmentStatusPublishInput & publish_input)
{
  AlignmentStatusEvaluation evaluation;
  evaluation.status_input = makeAlignmentStatusInput(publish_input);
  evaluation.status_preparation =
    lidar_localization::prepareAlignmentStatus(evaluation.status_input);
  evaluation.reinitialization_request =
    applyReinitializationRequestLatch(
      stamp,
      evaluation.status_preparation.reinitialization_request,
      evaluation.status_input.level == diagnostic_msgs::msg::DiagnosticStatus::OK &&
      evaluation.status_input.message == "ok" &&
      std::isfinite(evaluation.status_input.fitness_score) &&
      evaluation.status_input.fitness_score <= reinitialization_request_clear_max_fitness_ &&
      std::isfinite(evaluation.status_input.correction_translation_m) &&
      evaluation.status_input.correction_translation_m <=
      reinitialization_request_clear_max_correction_translation_m_);

  const auto recovery_evaluation =
    lidar_localization::evaluateAlignmentStatusRecovery(
      evaluation.status_input,
      evaluation.reinitialization_request);
  updateRecoverySupervisorState(
    stamp,
    recovery_evaluation.state,
    recovery_evaluation.action);
  return evaluation;
}

lidar_localization::AlignmentDiagnosticValuesInput
PCLLocalization::prepareAlignmentDiagnosticValuesInput(
  const lidar_localization::AlignmentStatusInput & status_input,
  const lidar_localization::AlignmentStatusPreparation & status_preparation,
  const ReinitializationRequestDecision & reinitialization_request) const
{
  auto diagnostic_input = lidar_localization::makeAlignmentStatusDiagnosticValuesInput(
    lidar_localization::makeAlignmentStatusDiagnosticInput(
      status_input,
      status_preparation,
      reinitialization_request,
      lidar_localization::AlignmentStatusDiagnosticRuntimeContext{
        lidar_localization::recoverySupervisorStateName(recovery_supervisor_state_),
        recovery_supervisor_action_,
        recovery_supervisor_state_entered_stamp_sec_,
        recovery_supervisor_transition_count_,
        reinitialization_request_latched_,
        reinitialization_request_latch_stamp_sec_}));
  const auto imu_diagnostics = lidar_localization::makeImuPreintegrationDiagnostics(
    makeImuPreintegrationDiagnosticsInput(status_input));
  diagnostic_input.imu_preintegration_enabled = imu_diagnostics.enabled;
  diagnostic_input.imu_preintegration_status =
    lidar_localization::imuPreintegrationDiagnosticStatusMessage(imu_diagnostics.status);
  diagnostic_input.imu_preintegration_fallback_mode = imu_diagnostics.fallback_mode;
  diagnostic_input.imu_smoother_initialized = imu_diagnostics.smoother_initialized;
  diagnostic_input.imu_has_new_samples = imu_diagnostics.has_new_samples;
  diagnostic_input.imu_received_sample_count = imu_diagnostics.received_sample_count;
  diagnostic_input.imu_integrated_sample_count = imu_diagnostics.integrated_sample_count;
  diagnostic_input.imu_skipped_sample_count = imu_diagnostics.skipped_sample_count;
  diagnostic_input.imu_transform_failure_count = imu_diagnostics.transform_failure_count;
  diagnostic_input.imu_non_finite_sample_count = imu_diagnostics.non_finite_sample_count;
  diagnostic_input.imu_invalid_dt_count = imu_diagnostics.invalid_dt_count;
  diagnostic_input.imu_last_dt_sec = imu_diagnostics.last_dt_sec;
  diagnostic_input.imu_last_sample_age_sec = imu_diagnostics.last_sample_age_sec;
  diagnostic_input.imu_integration_window_sec = imu_diagnostics.integration_window_sec;
  diagnostic_input.imu_seed_consistency_gate_enabled =
    imu_diagnostics.seed_consistency_gate_enabled;
  diagnostic_input.imu_seed_consistency_seed_allowed =
    imu_diagnostics.seed_consistency_seed_allowed;
  diagnostic_input.imu_seed_consistency_valid_comparison_count =
    imu_diagnostics.seed_consistency_valid_comparison_count;
  diagnostic_input.imu_seed_consistency_consecutive_pass_count =
    imu_diagnostics.seed_consistency_consecutive_pass_count;
  diagnostic_input.imu_seed_consistency_translation_error_m =
    imu_diagnostics.seed_consistency_translation_error_m;
  diagnostic_input.imu_seed_consistency_rotation_error_deg =
    imu_diagnostics.seed_consistency_rotation_error_deg;
  diagnostic_input.imu_seed_consistency_sample_passed =
    imu_diagnostics.seed_consistency_sample_passed;
  diagnostic_input.scan_time_status =
    lidar_localization::scanTimeRangeStatusMessage(latest_scan_time_status_);
  diagnostic_input.scan_time_field = latest_scan_time_field_;
  diagnostic_input.scan_time_duration_sec = latest_scan_time_duration_sec_;
  diagnostic_input.scan_time_valid_point_count = latest_scan_time_valid_point_count_;
  diagnostic_input.scan_time_invalid_point_count = latest_scan_time_invalid_point_count_;
  const auto deskew_readiness = lidar_localization::makeDeskewReadinessDiagnostics(
    lidar_localization::DeskewReadinessInput{
      latest_scan_time_status_,
      imu_diagnostics.status});
  diagnostic_input.deskew_ready = deskew_readiness.ready;
  diagnostic_input.deskew_readiness_status =
    lidar_localization::deskewReadinessStatusMessage(deskew_readiness.status);
  diagnostic_input.continuous_time_deskew_enabled = use_continuous_time_deskew_;
  diagnostic_input.continuous_time_deskew_applied = latest_continuous_time_deskew_applied_;
  diagnostic_input.continuous_time_deskew_status = latest_continuous_time_deskew_status_;
  diagnostic_input.continuous_time_deskew_point_count =
    latest_continuous_time_deskew_point_count_;
  diagnostic_input.continuous_time_deskew_skipped_invalid_time_count =
    latest_continuous_time_deskew_skipped_invalid_time_count_;
  diagnostic_input.continuous_time_deskew_clamped_time_count =
    latest_continuous_time_deskew_clamped_time_count_;
  diagnostic_input.continuous_time_deskew_mode = continuous_time_deskew_mode_;
  diagnostic_input.continuous_time_deskew_pose_history_coverage_ratio =
    latest_continuous_time_deskew_pose_history_coverage_ratio_;
  diagnostic_input.localizability_guard_enabled = enable_localizability_guard_;
  diagnostic_input.localizability_valid = latest_horizontal_localizability_.valid;
  diagnostic_input.horizontal_localizability_eigenvalue_ratio =
    latest_horizontal_localizability_.eigenvalue_ratio;
  diagnostic_input.localizability_guard_active = latest_localizability_guard_active_;
  return diagnostic_input;
}

lidar_localization::ImuPreintegrationDiagnosticsInput
PCLLocalization::makeImuPreintegrationDiagnosticsInput(
  const lidar_localization::AlignmentStatusInput & status_input) const
{
  std::lock_guard<std::mutex> lock(imu_preintegration_mutex_);
  return lidar_localization::ImuPreintegrationDiagnosticsInput{
    use_imu_preintegration_,
    imu_preintegration_fallback_mode_,
    imu_smoother_.isInitialized(),
    last_imu_stamp_ > 0.0,
    latest_imu_seed_has_new_samples_,
    status_input.imu_prediction_active,
    latest_imu_seed_prediction_finite_,
    latest_imu_seed_received_sample_count_,
    latest_imu_seed_integrated_sample_count_,
    latest_imu_seed_skipped_sample_count_,
    latest_imu_seed_transform_failure_count_,
    latest_imu_seed_non_finite_sample_count_,
    latest_imu_seed_invalid_dt_count_,
    latest_imu_seed_last_dt_sec_,
    latest_imu_seed_last_sample_age_sec_,
    latest_imu_seed_integration_window_sec_,
    lidar_localization::imuStaleSampleAgeThresholdSec(scan_period_),
    lidar_localization::imuMaximumIntegrationWindowSec(scan_period_),
    imu_seed_consistency_gate_enabled_,
    imu_seed_consistency_state_.seed_allowed,
    imu_seed_consistency_state_.valid_comparison_count,
    imu_seed_consistency_state_.consecutive_pass_count,
    latest_imu_seed_consistency_translation_error_m_,
    latest_imu_seed_consistency_rotation_error_deg_,
    latest_imu_seed_consistency_sample_passed_};
}

void PCLLocalization::appendAlignmentDiagnosticValues(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const lidar_localization::AlignmentDiagnosticValuesInput & diagnostic_values_input) const
{
  for (const auto & value :
    lidar_localization::makeRosAlignmentDiagnosticKeyValues(diagnostic_values_input))
  {
    status.values.push_back(value);
  }
}

diagnostic_msgs::msg::DiagnosticArray PCLLocalization::makeAlignmentDiagnosticArray(
  const builtin_interfaces::msg::Time & stamp,
  const diagnostic_msgs::msg::DiagnosticStatus & status) const
{
  diagnostic_msgs::msg::DiagnosticArray status_array;
  status_array.header.stamp = stamp;
  status_array.header.frame_id = base_frame_id_;
  status_array.status.push_back(status);
  return status_array;
}

void PCLLocalization::publishAlignmentDiagnosticStatus(
  const builtin_interfaces::msg::Time & stamp,
  const diagnostic_msgs::msg::DiagnosticStatus & status)
{
  status_pub_->publish(makeAlignmentDiagnosticArray(stamp, status));
}
