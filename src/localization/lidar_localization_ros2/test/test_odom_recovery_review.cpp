#include <cassert>
#include <limits>
#include <iostream>
#include <string>
#include "lidar_localization/odom_recovery_review.hpp"

using namespace lidar_localization;

Eigen::Matrix4f pose(float x, float y = 0.0f, float yaw = 0.0f)
{
  auto p = Eigen::Matrix4f::Identity().eval();
  p.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  p(0, 3) = x;
  p(1, 3) = y;
  return p;
}

int main(int argc, char ** argv)
{
  OdomRecoveryReviewParams params;
  params.enabled = true;
  OdomRecoveryReview review;
  if (argc > 1) {
    if (std::string(argv[1]) == "tracking") {
      params.frames = 6;
      params.min_observation_span_sec = 0.5;
      params.max_position_spread_m = 0.03;
      params.max_rotation_spread_deg = 0.5;
      params.max_fitness = std::numeric_limits<double>::max();
    }
    // Offline replay: timestamp, eligibility, fitness, candidate[16], odom[16].
    double stamp, score;
    bool eligible;
    Eigen::Matrix4f candidate, odom;
    while (std::cin >> stamp >> eligible >> score) {
      for (int i = 0; i < 16; ++i) {std::cin >> candidate(i / 4, i % 4);}
      for (int i = 0; i < 16; ++i) {std::cin >> odom(i / 4, i % 4);}
      if (!std::cin) {return 2;}
      const auto d = review.observe(params, stamp, candidate, odom, score, eligible);
      std::cout << stamp << ' ' << d.frames << ' ' << d.confirmed << ' '
                << d.position_spread_m << ' ' << d.rotation_spread_deg << '\n';
    }
    return 0;
  }
  const auto anchor = pose(0.6f, 0.3f, 0.4f);
  auto feed = [&](double t, float noise = 0.0f, bool eligible = true, double score = 0.003) {
      const auto odom = pose(static_cast<float>(t), static_cast<float>(t * 0.5),
        static_cast<float>(t * 0.1));
      Eigen::Matrix4f candidate = anchor * odom;
      candidate(0, 3) += noise;
      return review.observe(params, t, candidate, odom, score, eligible);
    };
  // Robot moves and rotates; compare candidates at the same odometry timestamp.
  assert(!feed(1.0).confirmed);
  assert(!feed(1.1, 0.01f).confirmed);
  auto decision = feed(1.2, 0.02f);
  assert(decision.confirmed && decision.frames == 3);
  assert(decision.position_spread_m < 0.03);
  assert(feed(1.3).frames == 1);  // A confirmation consumes the window.

  review.reset();
  feed(2.0); feed(2.1);
  assert(!feed(2.2, 0.2f).confirmed);
  assert(!feed(2.3).confirmed);
  assert(!feed(2.4).confirmed);
  assert(feed(2.5).confirmed);

  review.reset();
  feed(3.0); feed(3.1);
  assert(feed(3.2, 0, false).frames == 0);
  assert(feed(3.3).frames == 1);
  assert(feed(3.7).frames == 1);  // Missing scans invalidate continuity.
  assert(feed(3.7).frames == 0);  // Duplicate stamps cannot count as frames.
  assert(feed(3.6).frames == 0);  // Nor can out-of-order scans.
  assert(feed(3.8).frames == 1);
  assert(feed(3.9, 0, true, 0.006).frames == 0);
  assert(feed(4.0).frames == 1);
  auto invalid = pose(0);
  invalid(0, 0) = -1;  // Reflection is finite, but not a rigid rotation.
  assert(review.observe(params, 4.1, invalid, pose(0), 0.001, true).frames == 0);
  invalid = pose(0);
  invalid(0, 3) = std::numeric_limits<float>::quiet_NaN();
  assert(review.observe(params, 4.2, invalid, pose(0), 0.001, true).frames == 0);

  review.reset();
  for (int i = 0; i < 3; ++i) {
    auto candidate = pose(0.6f, 0, i == 2 ? 0.04f : 0.0f);
    assert(!review.observe(params, 5 + i * 0.1, candidate, pose(0), 0.001, true).confirmed);
  }
  params.enabled = false;
  assert(feed(6.0).frames == 0);
  params.enabled = true;
  params.frames = 1;
  assert(!params.valid());
  assert(feed(6.1).frames == 0);
  params.frames = 3;
  params.max_scan_gap_sec = std::numeric_limits<double>::quiet_NaN();
  assert(!params.valid());
  assert(feed(6.2).frames == 0);

  // The same reviewer also audits ordinary tracking updates. No axis is
  // privileged and candidate collection must not change the caller's anchor.
  params = OdomRecoveryReviewParams{};
  params.enabled = true;
  params.frames = 6;
  params.min_observation_span_sec = 0.5;
  params.max_position_spread_m = 0.03;
  params.max_rotation_spread_deg = 0.5;
  review.reset();
  const auto trusted = pose(0);
  const auto trusted_copy = trusted;
  auto observe_tracking = [&](double t, const Eigen::Matrix4f & update) {
      const auto odom = pose(static_cast<float>(t), static_cast<float>(t * 0.4),
        static_cast<float>(t * 0.3));
      return review.observe(params, t, update * odom, odom, 0.001, true);
    };
  // A short 28-cm alias never earns a trusted commit, even when stable.
  for (int i = 0; i < 5; ++i) {
    assert(!observe_tracking(10.0 + i * 0.1, pose(0, 0.28f)).confirmed);
  }
  assert(!observe_tracking(10.5, pose(0)).confirmed);
  for (int i = 1; i < 5; ++i) {
    assert(!observe_tracking(10.5 + i * 0.1, pose(0)).confirmed);
  }
  decision = observe_tracking(11.0, pose(0));
  assert(decision.confirmed && decision.frames == 6);
  assert(decision.observation_span_sec >= 0.5 - 1e-6);
  assert(trusted.isApprox(trusted_copy));

  // Correct recovery is not projected back toward a corrupted prediction.
  review.reset();
  const auto correction = pose(0.22f, -0.16f, 0.04f);
  for (int i = 0; i < 6; ++i) {
    decision = observe_tracking(12.0 + i * 0.1, correction);
    assert(decision.confirmed == (i == 5));
  }
  // Six callback bursts are not six independent observations over 0.5 sec.
  review.reset();
  for (int i = 0; i < 6; ++i) {
    assert(!observe_tracking(13.0 + i * 0.01, correction).confirmed);
  }
  review.reset();
  for (int i = 0; i <= 10; ++i) {
    decision = observe_tracking(14.0 + i * 0.05, correction);
    assert(decision.confirmed == (i == 10));  // 20-Hz scans still confirm.
  }
  params.min_observation_span_sec = -1;
  assert(!params.valid());
  params.min_observation_span_sec = 2;  // Impossible with six gaps <= 0.3.
  assert(!params.valid());
}
