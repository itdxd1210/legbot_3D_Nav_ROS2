from pathlib import Path
import resource
import subprocess


def test_recovery_review_policy(tmp_path):
    root = Path(__file__).resolve().parents[1]
    binary = tmp_path / 'recovery_review'
    subprocess.run(['c++', '-std=c++17', '-O0', '-I/usr/include/eigen3',
                    '-I' + str(root / 'include'),
                    str(root / 'test/test_odom_recovery_review.cpp'),
                    '-o', str(binary)], check=True)
    def no_core_dump():
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    subprocess.run([str(binary)], check=True, preexec_fn=no_core_dump)


def test_review_occurs_once_before_commit_and_after_generation_check():
    root = Path(__file__).resolve().parents[1]
    subscriber = (root / 'src/component_subscribers.cpp').read_text()
    assert subscriber.count('reviewOdomRecoveryCandidate(pipeline_result, selected_seed)') == 1
    check = subscriber.index('initialPoseGenerationMatches(seed_generation)')
    review = subscriber.index('reviewOdomRecoveryCandidate(pipeline_result, selected_seed)')
    commit = subscriber.index('applyAcceptedAlignmentPipelineResult(', review)
    assert check < review < commit
    alignment = (root / 'src/component_alignment.cpp').read_text()
    assert 'odom_recovery_review_.observe' not in alignment.split(
        'void PCLLocalization::reviewOdomRecoveryCandidate(', 1)[0]
    assert 'odom_recovery_review_.reset()' in (root / 'src/component_prediction_state.cpp').read_text()


def test_review_preserves_rotation_validity_and_timestamp_gates():
    root = Path(__file__).resolve().parents[1]
    source = (root / 'src/component_alignment.cpp').read_text()
    review = source.split('void PCLLocalization::reviewOdomRecoveryCandidate(', 1)[1].split(
        'void PCLLocalization::logAlignmentPipelineRecovery(', 1)[0]
    for required in ['attempt.target_ready && attempt.has_converged && attempt.pose_metrics_valid',
                     'attempt.scan_stamp_sec > last_accepted_pose_time_sec_',
                     'attempt.correction_rotation_deg <= limits.odom_tf_prediction_correction_guard_rotation_deg',
                     'attempt.correction_yaw_deg <= limits.odom_tf_prediction_correction_guard_yaw_deg',
                     '!pipeline_result.recovered_by_retry_from_last_pose',
                     'status_message == "odom_tf_prediction_correction_guard_rejected"']:
        assert required in review
    assert 'if (!odom_recovery_review_decision_.confirmed) {return;}' in review
    assert 'last_good_map_to_odom_ =' not in review
    assert 'last_accepted_pose_matrix_ =' not in review


def test_tracking_review_holds_before_backend_without_advancing_failed_prediction():
    root = Path(__file__).resolve().parents[1]
    subscriber = (root / 'src/component_subscribers.cpp').read_text()
    assert subscriber.count('reviewOdomTrackingCandidate(pipeline_result, selected_seed)') == 1
    check = subscriber.index('initialPoseGenerationMatches(seed_generation)')
    review = subscriber.index('reviewOdomTrackingCandidate(pipeline_result, selected_seed)')
    pending = subscriber.index('if (tracking_candidate_pending)', review)
    commit = subscriber.index('applyAcceptedAlignmentPipelineResult(', pending)
    assert check < review < pending < commit
    branch = subscriber[pending:subscriber.index('if (handleTerminalAlignmentPipelineResult(', pending)]
    assert 'publishAlignmentStatusForAttempt' in branch
    assert 'publishBridgePoseAsRejectedOutput' in branch
    assert 'audit_commit(false)' in branch and 'return;' in branch
    assert 'advancePredictionWithoutMeasurement' not in branch
    assert 'applyAcceptedAlignmentPipelineResult' not in branch
    alignment = (root / 'src/component_alignment.cpp').read_text()
    policy = alignment.split('bool PCLLocalization::reviewOdomTrackingCandidate(', 1)[1].split(
        'void PCLLocalization::logAlignmentPipelineRecovery(', 1)[0]
    for required in ['pipeline_result.gate_result.reject_measurement',
                     '!attempt.target_ready || !attempt.has_converged || !attempt.pose_metrics_valid',
                     'attempt.scan_stamp_sec <= last_accepted_pose_time_sec_',
                     'odom_recovery_review_decision_.confirmed',
                     'anchor.inverse() * seed.init_guess',
                     '"odom_candidate_review_pending"']:
        assert required in policy
    for forbidden in ['last_good_map_to_odom_ =', 'last_accepted_pose_matrix_ =',
                      'attempt.final_transformation =', 'advancePredictionWithoutMeasurement']:
        assert forbidden not in policy
    assert 'odom_candidate_review_.reset()' in (root / 'src/component_prediction_state.cpp').read_text()


def test_tracking_review_defaults_and_bringup_opt_in():
    root = Path(__file__).resolve().parents[1]
    source = (root / 'src/component_lifecycle.cpp').read_text()
    assert 'declare_parameter("enable_odom_candidate_review", false)' in source
    assert 'declare_parameter("odom_candidate_review_frames", 6)' in source
    assert 'declare_parameter("odom_candidate_review_min_span_sec", 0.50)' in source
    preset = (root / 'param/localization.yaml').read_text()
    assert 'enable_odom_candidate_review: false' in preset
    assert 'odom_candidate_review_frames: 6' in preset
    bringup = root.parents[1] / 'legbot_bringup/launch'
    launch = (bringup / 'map_localization.launch.py').read_text()
    assert "LaunchConfiguration('candidate_review')" in launch
    assert "DeclareLaunchArgument('candidate_review', default_value='true'" in launch
    main = (bringup / 'pct_cross_floor_demo.launch.py').read_text()
    assert "'candidate_review': LaunchConfiguration('localization_candidate_review')" in main
