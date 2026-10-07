from pathlib import Path
import subprocess


def test_required_odom_seed(tmp_path):
    root = Path(__file__).resolve().parents[1]
    source = tmp_path / 'test.cpp'
    source.write_text('''
#include <lidar_localization/registration_seed_policy.hpp>
#include <cassert>
using namespace lidar_localization;
int main() {
  for (auto fallback : {RegistrationSeedSource::kPreviousDelta,
                        RegistrationSeedSource::kCurrentPose}) {
    assert(shouldSkipUnavailableOdomSeed(true, true, fallback));
    assert(!shouldSkipUnavailableOdomSeed(false, true, fallback));
    assert(!shouldSkipUnavailableOdomSeed(true, false, fallback));
  }
  assert(!shouldSkipUnavailableOdomSeed(true, true, RegistrationSeedSource::kOdomTfPrediction));
}
''')
    binary = tmp_path / 'test'
    subprocess.run(['c++', '-std=c++17', '-I'+str(root/'include'), str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
