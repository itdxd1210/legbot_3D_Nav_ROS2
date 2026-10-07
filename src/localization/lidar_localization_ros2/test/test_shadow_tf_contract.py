"""Every localization TF send must honor the shadow-mode output gate."""
from pathlib import Path


def test_all_transform_outputs_are_gated():
    root = Path(__file__).resolve().parents[1]
    outputs = []
    for source in (root / 'src').glob('*.cpp'):
        for line in source.read_text().splitlines():
            if 'broadcaster_.sendTransform(' in line:
                outputs.append(line)
                assert 'get_parameter("publish_tf").as_bool()' in line
    assert outputs
