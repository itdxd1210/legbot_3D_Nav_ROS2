from pathlib import Path
import subprocess
import resource
import pytest


@pytest.mark.parametrize('policy', [
    'full_pose_guard', 'alignment_attempt_policy', 'measurement_gate_policy',
    'prediction_state_policy', 'localization_update_policy',
    'alignment_pipeline_policy', 'alignment_failure_taxonomy'])
def test_full_pose_guard_and_prediction_hold(tmp_path, policy):
    root = Path(__file__).resolve().parents[1]
    binary = tmp_path / 'full_pose_guard'
    subprocess.run(['c++', '-std=c++17', '-O0', '-I/usr/include/eigen3',
                    '-I' + str(root / 'include'),
                    str(root / ('test/test_' + policy + '.cpp')),
                    '-o', str(binary)], check=True)
    # Assertion failures should not create disk-heavy apport/core artifacts.
    def no_core_dump():
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    subprocess.run([str(binary)], check=True, preexec_fn=no_core_dump)


def test_external_odom_rejection_precedes_backend_and_seed_writes():
    root = Path(__file__).resolve().parents[1]
    source = (root / 'src/component_alignment.cpp').read_text()
    function = source.split('bool PCLLocalization::applyAcceptedAlignmentPipelineResult(', 1)[1]
    guard = function.index('pipeline_result.gate_result.reject_measurement')
    early_return = function.index('return false;', guard)
    assert early_return < function.index('makeRegistrationObservation(')
    assert early_return < function.index('applyRegistrationPoseBackend(')
    assert 'updatePredictionFromRejectedMeasurement' not in function[:early_return]


def test_odom_guard_keeps_original_prediction_across_refinement_and_retry():
    root = Path(__file__).resolve().parents[1]
    source = (root / 'src/component_alignment.cpp').read_text()
    assert 'use_odom_tf_prediction_ ? trusted_prediction : init_guess' in source
    assert 'use_odom_tf_prediction_ ? trusted_prediction : last_accepted_pose_matrix_' in source
    subscriber = (root / 'src/component_subscribers.cpp').read_text()
    assert 'init_guess,\n    selected_seed.init_guess,' in subscriber
    assert 'REJECTED_MATCH_MODIFIED_TRUSTED_STATE' in subscriber
