# Required odometry prediction after initialization

When `use_odom_tf_prediction` is enabled and a map/odom anchor exists,
missing scan-time odometry TF now skips registration for that scan. It no
longer falls back to previous registration delta or current-pose seeds.
The accepted anchor and registration failure counters are unchanged.
Diagnostics report `odom_prediction_unavailable_scan_skipped` at the scan stamp.
The next scan retries normally. No latest-time transform is substituted.

Bootstrap before an anchor exists and modes with odometry prediction disabled
remain unchanged. This can reduce matching update rate if TF is persistently
late; it does not repair timestamp problems or guarantee registration accuracy.

## Full-pose innovation guard and state audit

Go2 bringup enables a provisional 5-degree full rotation innovation limit,
alongside the existing 0.5-m translation guard. The generic preset leaves
`odom_tf_prediction_correction_guard_rotation_deg` at zero (disabled).
The demo override is `localization_rotation_guard_deg:=5.0` (no rebuild needed).
The innovation compares the candidate with the same-time trusted map/odom
anchor composed with raw LIO, not with a horizontal body attitude. Thus real
stair pitch or the fixed 45-degree sensor mounting is not itself rejected.
Initializer refinement and registration retries do not change that reference.

The 5-degree setting is an experimental starting value: successful-run
same-time pose pairs estimated a maximum 4.30-degree innovation; the failed
run showed a 7.52-degree step at simulation time 177.2 s. This log estimate
is not an offline registration replay or proof that every alias is detected.
No Gazebo truth is read by the online acceptance guard.

Nonfinite/non-rigid poses and non-increasing measurement timestamps are
rejected. In established external-odom mode every rejected candidate exits
before any pose backend writes, even if legacy rejected-seed update is enabled.
Trusted pose/time and map/odom anchors remain unchanged; live LIO continues
to propagate navigation with the held transform. Recovery score relaxation
cannot override the full-rotation guard. Slowly accumulating plausible aliases
remain a limitation; Hessian diagnostics are not used as a directional filter.

With registration diagnostics enabled, `/alignment_status` contains prediction
and candidate XYZ/quaternion, signed local XYZ innovation, roll/pitch/rotation
innovation, threshold, and pre-commit trusted state. `MATCH_COMMIT` records
actual post-processing state changes and the resulting anchor. A rejected
candidate changing trusted pose/time or anchor raises
`REJECTED_MATCH_MODIFIED_TRUSTED_STATE`. The audit is per scan, bounded by
the scan rate; no point clouds or additional bags are saved.
