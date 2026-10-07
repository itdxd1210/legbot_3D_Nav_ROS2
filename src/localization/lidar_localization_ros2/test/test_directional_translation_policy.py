from pathlib import Path
import resource
import subprocess


def test_directional_translation_policy(tmp_path):
    root = Path(__file__).resolve().parents[1]
    binary = tmp_path / 'direction_policy'
    subprocess.run(['c++', '-std=c++17', '-O0', '-I/usr/include/eigen3',
                    '-I' + str(root / 'include'),
                    str(root / 'test/test_directional_translation_policy.cpp'),
                    '-o', str(binary)], check=True)
    def no_core_dump():
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    subprocess.run([str(binary)], check=True, preexec_fn=no_core_dump)


def test_direction_preview_is_observation_only():
    root = Path(__file__).resolve().parents[1]
    source = (root / 'src/component_alignment.cpp').read_text()
    preview = source.split('// Directional preview is observation-only.', 1)[1].split(
        '\n  return attempt;', 1)[0]
    assert 'attempt.directional_translation_preview =' in preview
    for write in ['attempt.final_transformation =', 'last_good_map_to_odom_ =',
                  'last_accepted_pose_matrix_ =', 'gate_result.']:
        assert write not in preview
    subscriber = (root / 'src/component_subscribers.cpp').read_text()
    assert 'directional_translation_preview' not in subscriber
