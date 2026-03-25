# Quaternion Debug Sanity Check

This note documents the runtime sanity check added for the matrix orthogonality issue.

## Goal

Validate, at runtime, that we handle problematic rotation matrices safely:

1. Try quaternion conversion on the **raw (untouched)** rotation matrix.
2. If needed, project to nearest valid SO(3) matrix.
3. Try quaternion conversion again on projected matrix.
4. Log whether raw conversion failed and projected conversion succeeded.

## What was added

### 1) IMU-path sanity check in RobotModel

File: `h12_ros2_controller/core/robot_model.py`

Inside `full_q(...)` debug instrumentation:

- Build raw rotation from incoming IMU quaternion (`imu_quat_raw`).
- Compute metrics:
  - `|norm-1|`
  - `ortho_fro = ||R^T R - I||_F`
  - `det(R)`
  - angular distance to projected orientation
- Try quaternion conversion on raw matrix (`raw_quat_ok`).
- Project matrix to SO(3), then try conversion again (`proj_quat_ok`).
- Emit debug logs with both flags and error text when conversion fails.

### 2) `matrix_to_pose(...)` sanity check

File: `h12_ros2_controller/ros2/utility.py`

- Try `Quaternion(matrix=rotation_matrix)` before projection.
- Log `raw_quat_ok` and exception details if raw conversion fails.
- Project invalid matrix to SO(3).
- Try conversion on projected matrix.
- If projected conversion still fails (unexpected), log and fall back to identity quaternion.

## How to read logs

### Healthy case (small numeric noise)

You should see lines like:

- `raw_quat_ok=True proj_quat_ok=True`
- very small `ortho_fro` (near 1e-6 or lower)
- `det` close to 1

This means projection is mostly a safety net.

### Problematic/outlier case (the issue we are guarding)

Expected signature:

- `raw_quat_ok=False`
- `proj_quat_ok=True`
- Warning/debug line with `raw_matrix_quat_error=...`

This directly confirms the fix path is doing useful work: raw matrix would have failed strict quaternion conversion, projected matrix succeeds.

## Why this confirms the issue

The original crash came from strict quaternion construction from a near-invalid matrix. This check demonstrates, in the same runtime path:

- whether raw conversion would throw,
- whether projection resolves it,
- and whether downstream pose conversion remains robust.

So we now have direct evidence in logs instead of only inferring from matrix metrics.

## Notes

- Diagnostics are intentionally throttled to avoid log spam.
- These logs are intended for debugging and can be kept in a debug-only commit if desired.
