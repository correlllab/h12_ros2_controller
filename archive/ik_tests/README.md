# IK accuracy / repeatability test suite

Drives `FrameController` through a configurable matrix of
`(case x sweep-point x trial)` frame-task goals and records per-joint
commanded vs. actual position, velocity, and torque, with URDF and
software (clip) limits overlaid as dashed lines. Targets the
`test/ik-debug` branch.

> **Run from the repo source root.** URDF/asset paths in the configs are
> relative (`assets/h1_2/...`) to match the rest of the repo's examples.
> If `python -m ...` resolves to the colcon install tree and fails to
> find yaml configs, run `colcon build --packages-select h12_ros2_controller`
> once so the updated `package_data` (yaml configs) get installed — or
> run straight from the source tree without the overlay sourced.

## Safety layer

The runner publishes commands via `LowCmdHandler`, which reads
`topics.low_cmd` from the controller config you select.

| controller config | publish topic | routed through safety? |
|---|---|---|
| `safety_split.yaml` (default) | `rt/safety/lowcmd_upper_in` | **yes** — upper-body relay |
| `safety_full.yaml` | `rt/safety/lowcmd_in` | **yes** — full-body relay |
| `debug.yaml` | `rt/lowcmd` | **no** — direct to robot |

The controller config and the running safety node **must match
suffixes** — this is the same rule enforced by
[robot_safety_launch.py](../../launch/robot_safety_launch.py). If the
safety node was launched with `_full`, select `safety_full.yaml`; if
`_split`, select `safety_split.yaml`. Mismatch means no subscriber on
the publish topic and tracking error will look catastrophic for the
wrong reason.

Pick `debug.yaml` **only** for pure sim runs (bypasses safety entirely).

External E-stop is not observed by the runner itself — the safety node
handles it. If e-stop trips mid-run the safety relay drops commands;
the runner will record the ensuing tracking error (high `q_cmd` vs
flat `q_actual`). Abort with Ctrl-C and the runner will call
`controller.shutdown()` cleanly.

## ROS transport environment (real mode)

The real-mode evaluator runs through ROS safety topics. Before
`python -m ... --mode real`, use the same shell environment as the
safety node: same ROS2 overlay, same middleware settings, and same
domain id when applicable.

```bash
source install/setup.bash          # your ROS2 overlay
# optional but recommended for explicitness:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0             # or your robot's configured domain
```

At startup the runner prints `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID`,
and `CYCLONEDDS_URI`, then warns (with a 2s pause) when important
variables are missing.

## Layout

```
h12_ros2_controller/tests/
  ik_eval_server.py     preferred runner — delegates control to a running
                        frame_task_server, records via rt/lowstate + ROS topics
  ik_eval.py            standalone runner (FrameController driver, sim mode only)
  ik_feasibility.py     offline target vetting — run before editing sweep.yaml
  plot_ik_eval.py       per-trial + aggregate plots (works with both runners)
  configs/
    minimal.yaml        single case / single sweep point / 3 trials
    sweep.yaml          full case list + weights/damping sweep scaffold
```

Each run writes to `runs/<timestamp>_<label>_<mode>/`:

```
runs/<ts>_minimal_sim/
  meta.yaml                resolved test + controller config snapshot
  summary.csv              one row per trial (steady-state error, convergence)
  trials/<case>__<sweep>__t<N>.npz   per-trial timeseries
  fk_roundtrip.npz         offline IK-purity diagnostic (if enabled)
  plots/                   produced by plot_ik_eval.py
```

## Run it

```bash
# ── preferred: server-based runner (requires frame_task_server already running) ──
# Records via rt/lowstate; all motion controlled by the server (safe, proven)
python -m h12_ros2_controller.tests.ik_eval_server --config sweep.yaml --yes
python -m h12_ros2_controller.tests.ik_eval_server --config sweep_tilt.yaml --yes
ros2 topic echo /frame_task/_action/feedback

# ── offline target vetting before editing sweep.yaml ─────────────────────────
python -m h12_ros2_controller.tests.ik_feasibility

# ── standalone runner, sim mode only (no hardware) ───────────────────────────
python -m h12_ros2_controller.tests.ik_eval --config minimal.yaml --mode sim

# ── plot the most recent run (works for both runners) ─────────────────────────
python -m h12_ros2_controller.tests.plot_ik_eval
# or a specific dir
python -m h12_ros2_controller.tests.plot_ik_eval runs/20260420_143000_minimal_real
```

## What's in a trial

For every control step the runner captures:

- `q_actual`, `dq_actual`, `tau_actual` from the low-state subscriber
- `q_cmd` (= `ik_solver.q` after the step)
- `tau_cmd` (= gravity-compensation feedforward)
- `frame_err` (6-vector) from the active frame task
- `ee_actual` (6D pose) via FK on the reduced model
- `twist` at the commanded frame

Plus a once-per-trial reference `q_ik_optimal` produced by pink's iterative
`solve_ik_reduced` (zero-start) to distinguish "IK failure" from
"tracking failure."

## What to look at

- **`plots/<trial>/<joint>.png`** — q_actual vs q_cmd, dq, tau_actual vs
  tau_cmd, with URDF (red dashed) and clip (orange dotted) limits.
  A joint pinned against a clip line that is tighter than the URDF
  line is the smoking gun for bad `position_offset` calibration.
- **`plots/<trial>/cartesian.png`** — lin/ang error vs time.
- **`plots/repeatability/<case>__<sweep>.png`** — end-effector scatter
  across trials for a given target. Width of the cluster is your
  repeatability number; offset from the red X is your accuracy.
- **`plots/sweep_summary.png`** — mean steady-state error per sweep
  point; how the weights/damping trade off against each other.
- **`plots/fk_roundtrip.png`** — histogram of FK->IK residuals from
  sampled reduced-config poses. If this is clean and the on-robot
  plots are not, the IK solver is innocent.

## Configuring new cases

Edit `tests/configs/<your>.yaml`:

```yaml
cases:
  - name: descriptive_name
    frame: left_wrist_yaw_link       # any frame in the reduced model
    target_pose: [x, y, z, r, p, y]  # meters + rad (or deg if
                                      # orientation_units: deg)
```

The `sweeps:` block is a Cartesian product over the lists given; a
single-element list pins that dimension. Empty `{}` pins everything
to pink's defaults.

## Additional diagnostics that are already wired in

1. **FK -> IK round-trip** (`fk_roundtrip.enabled: true`) — run before the
   on-robot trials to isolate IK-solver issues from calibration.
2. **`q_ik_optimal` reference** — recorded per trial when
   `trial.record_ik_optimal: true`; overlaid as a green horizontal
   dashed line on the per-joint position panel.
3. **Home repeatability** — implicit: every `return_home_between`
   transition drives to `home_q`; inspect the tail of each trial
   at the home_q step in the per-joint plot.
4. **Software- vs URDF-limit clamping** — dashed lines on every joint
   panel highlight which limit (if any) a joint is pinned against.

## Suggested follow-ups (easy to add)

- **Symmetry test**: add mirrored left/right cases and compare
  `summary.csv` rows pairwise - a single miscalibrated joint shows as
  asymmetric error.
- **Start-config sensitivity**: seed `home_q` with N alternative values
  (add a `homes:` list) and run each case from each home; divergent
  final q means the IK solver is falling into different local minima.
- **Velocity-saturation logging**: detect time steps where
  `upper_controller.limit_joint_vel` scales below 1.0 (currently only
  the resulting twist is recorded).
- **Posture-task error**: record `posture_task.compute_error` to see
  whether posture regularization is fighting the frame task.
