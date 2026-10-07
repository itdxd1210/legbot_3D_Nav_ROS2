#pragma once

#include <deque>
#include "lidar_localization/alignment_attempt_policy.hpp"

namespace lidar_localization
{

struct OdomRecoveryReviewParams
{
  bool enabled{false};
  int frames{3};
  double max_position_spread_m{0.10};
  double max_rotation_spread_deg{1.0};
  double max_fitness{0.005};
  double max_scan_gap_sec{0.30};
  double min_observation_span_sec{0.0};

  bool valid() const
  {
    return frames >= 2 && frames <= 32 &&
           std::isfinite(max_position_spread_m) && max_position_spread_m > 0 &&
           std::isfinite(max_rotation_spread_deg) && max_rotation_spread_deg > 0 &&
           std::isfinite(max_fitness) && max_fitness > 0 &&
           std::isfinite(max_scan_gap_sec) && max_scan_gap_sec > 0 &&
           std::isfinite(min_observation_span_sec) && min_observation_span_sec >= 0 &&
           min_observation_span_sec <= (frames - 1) * max_scan_gap_sec;
  }
};

struct OdomRecoveryReviewDecision
{
  bool candidate{false};
  bool confirmed{false};
  int frames{0};
  double position_spread_m{0.0};
  double rotation_spread_deg{0.0};
  double observation_span_sec{0.0};
};

// Shared temporal review for tracking candidates and large-innovation recovery.
// This class collects evidence only; the caller owns every trusted commit.
class OdomRecoveryReview
{
public:
  void reset()
  {
    samples_.clear();
    last_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  }

  OdomRecoveryReviewDecision observe(
    const OdomRecoveryReviewParams & params, double stamp_sec,
    const Eigen::Matrix4f & candidate_map_base, const Eigen::Matrix4f & odom_base,
    double fitness, bool eligible_candidate)
  {
    OdomRecoveryReviewDecision result;
    if (!std::isfinite(stamp_sec) ||
      (std::isfinite(last_stamp_sec_) && stamp_sec <= last_stamp_sec_))
    {
      samples_.clear();
      return result;
    }
    if (std::isfinite(last_stamp_sec_) && stamp_sec - last_stamp_sec_ > params.max_scan_gap_sec) {
      samples_.clear();
    }
    last_stamp_sec_ = stamp_sec;
    if (!params.enabled || !params.valid() || !eligible_candidate ||
      !std::isfinite(fitness) || fitness < 0 || fitness > params.max_fitness ||
      !isFiniteRigidPose(candidate_map_base) || !isFiniteRigidPose(odom_base))
    {
      samples_.clear();
      return result;
    }
    result.candidate = true;
    samples_.push_back({stamp_sec, candidate_map_base * odom_base.inverse()});
    // Keep enough time as well as enough frames at higher scan rates. The
    // bounded window prevents a stationary/high-rate stream growing memory.
    const std::size_t capacity = params.min_observation_span_sec > 0.0 ?
      32u : static_cast<std::size_t>(params.frames);
    while (samples_.size() > capacity) {samples_.pop_front();}
    while (samples_.size() > static_cast<std::size_t>(params.frames) &&
      stamp_sec - samples_[1].stamp_sec + 1e-6 >= params.min_observation_span_sec)
    {
      samples_.pop_front();
    }
    result.frames = static_cast<int>(samples_.size());
    result.observation_span_sec = stamp_sec - samples_.front().stamp_sec;
    // C_i * inverse(O_i) * O_now: carry every candidate to the current scan.
    // The full rigid motion is used, independent of corridor or spawn heading.
    for (std::size_t i = 0; i < samples_.size(); ++i) {
      for (std::size_t j = i + 1; j < samples_.size(); ++j) {
        const Eigen::Matrix4f a = samples_[i].map_to_odom * odom_base;
        const Eigen::Matrix4f b = samples_[j].map_to_odom * odom_base;
        result.position_spread_m = std::max(result.position_spread_m,
          static_cast<double>((a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm()));
        const Eigen::Matrix3f relative = a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0);
        result.rotation_spread_deg = std::max(result.rotation_spread_deg,
          static_cast<double>(Eigen::AngleAxisf(relative).angle()) * 180.0 / std::acos(-1.0));
      }
    }
    result.confirmed = result.frames >= params.frames &&
      result.observation_span_sec + 1e-6 >= params.min_observation_span_sec &&
      result.position_spread_m <= params.max_position_spread_m &&
      result.rotation_spread_deg <= params.max_rotation_spread_deg;
    if (result.confirmed) {samples_.clear();}
    return result;
  }

private:
  struct Sample
  {
    double stamp_sec;
    Eigen::Matrix4f map_to_odom;
  };
  std::deque<Sample> samples_;
  double last_stamp_sec_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace lidar_localization
