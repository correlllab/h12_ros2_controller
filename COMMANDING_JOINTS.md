# Commanding the H1-2 Arms — Student Guide

This guide is for students who want to drive the H1-2 arms from their own
computer. It covers three things:

1. Building only the packages you need (`custom_ros_messages`,
   `h12_ros2_model`, `h12_ros2_controller`).
2. Sending one-off **joint commands** (target joint angles) to the arms — not
   just end-effector frame targets.
3. **Streaming setpoints** at a fixed rate — joint targets (over ROS *or*
   straight Unitree DDS) or end-effector frame targets (over ROS) — for piping
   the output of your own IK / collision-avoidance policy into our control
   pipeline ([jump to section 5](#5-streaming-setpoints-joint-or-frame)).

> ⚠️ **Safety first.** These commands move a real robot. Always have the
> physical e-stop within reach, keep clear of the arms, and start with small
> moves and long durations. Read [Safety Notes](#safety-notes) before running
> anything on hardware.

---

## 1. What you need

The controller stack is three ROS 2 packages that live under your workspace
`src/`:

| Package | Why you need it |
| --- | --- |
| `custom_ros_messages` | Defines the `NamedConfig` / `FrameTask` actions and `MotorCmd*` / `MotorState*` messages used to talk to the controller. |
| `h12_ros2_model` | Robot description (URDF/SRDF) the controller loads. |
| `h12_ros2_controller` | The controller nodes themselves (`frame_task_server`, etc.). |

**Prerequisites**

- ROS 2 (Humble) installed and sourced (`source /opt/ros/humble/setup.bash`).
- `colcon` build tools (`sudo apt install python3-colcon-common-extensions`).
- The Python dependencies this package needs (Pinocchio, numpy, the Unitree
  SDK, etc.). The easiest way to get them is to follow the `uv`/`pip`
  instructions in the main [README](README.md#installation). On a machine that
  only *commands* the robot (and isn't the one running on the robot's network)
  you still need these because the server imports them.

---

## 2. Build the packages

Create a workspace and clone the three packages under `src/`:

```bash
mkdir -p ~/ws_h12/src
cd ~/ws_h12/src
git clone https://github.com/correlllab/custom_ros_messages.git
git clone https://github.com/correlllab/h12_ros2_model.git
git clone https://github.com/correlllab/h12_ros2_controller.git

# h12_ros2_controller pulls in the Unitree SDK as a submodule
cd h12_ros2_controller
git submodule update --init --recursive
```

Build from the workspace root and source the overlay:

```bash
cd ~/ws_h12
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_ros_messages h12_ros2_model h12_ros2_controller
source install/setup.bash
```

> Re-run `source install/setup.bash` in **every new terminal** you use to talk
> to the robot. After editing any Python in these packages, re-run
> `colcon build` (or `colcon build --symlink-install` so Python edits are
> picked up without rebuilding).

Confirm the action interfaces built:

```bash
ros2 interface show custom_ros_messages/action/NamedConfig
```

---

## 3. Start the controller

The controller server is what actually commands the motors. The
`named_config` action that you use for joint commands is served by the
**`frame_task_server`** node.

The simplest way to bring everything up is the provided launch file, which
starts `robot_state_publisher`, `joint_state_publisher`, and
`frame_task_server`:

```bash
ros2 launch h12_ros2_controller robot_launch.py config:=debug.yaml
```

Pick the config that matches your situation:

| Config | Use |
| --- | --- |
| `debug.yaml` | Default. Use with the [Mujoco sim](https://github.com/HIRO-group/h1_mujoco) or for dry runs. |
| `sport.yaml` | Sport mode on hardware. |
| `safety_full.yaml` / `safety_split.yaml` | Run with the safety layer (see below). |

### Running with the safety layer

If you want the safety layer in the loop, use `robot_safety_launch.py`, which
additionally starts the `estop` node and `h12_safety_layer/safety_node`. The
controller config and safety config suffixes must match (`_full` with `_full`,
`_split` with `_split`):

```bash
ros2 launch h12_ros2_controller robot_safety_launch.py \
    config:=safety_full safety_config:=default_safety_full
```

This requires the `estop` and `h12_safety_layer` packages to be built in your
workspace as well.

> Start in the **sim / debug** path first and confirm your joint commands look
> right before moving to hardware or the safety-layer launch.

---

## 4. Sending joint commands

The arms are commanded in joint space through the **`NamedConfig` action**
(`/named_config`). A "named config" is just a vector of target joint angles
that the controller drives to using `goto_reduced_configuration`.

> 💡 **One-off vs. streaming.** `NamedConfig` is an *action*: each goal blocks
> until the arms converge, so it's meant for discrete, occasional moves. If you
> want to send a continuous stream of setpoints (e.g. the output of your own IK
> controller at ~10 Hz), use the
> [stream server](#5-streaming-setpoints-joint-or-frame) instead — do **not**
> spam `NamedConfig`/`FrameTask` goals.

### The 14 controllable arm joints

Joint commands address the **enabled arm joints** only — the 7 left-arm and
7 right-arm joints, in this exact order
(`h12_ros2_controller/utility/joint_definition.py → ENABLED_JOINTS`):

| Index | Joint |
| --- | --- |
| 0 | `left_shoulder_pitch_joint` |
| 1 | `left_shoulder_roll_joint` |
| 2 | `left_shoulder_yaw_joint` |
| 3 | `left_elbow_joint` |
| 4 | `left_wrist_roll_joint` |
| 5 | `left_wrist_pitch_joint` |
| 6 | `left_wrist_yaw_joint` |
| 7 | `right_shoulder_pitch_joint` |
| 8 | `right_shoulder_roll_joint` |
| 9 | `right_shoulder_yaw_joint` |
| 10 | `right_elbow_joint` |
| 11 | `right_wrist_roll_joint` |
| 12 | `right_wrist_pitch_joint` |
| 13 | `right_wrist_yaw_joint` |

Angles are in **radians**. A vector of all zeros (`home`) is the arms-down
neutral pose.

### Step 1 — Define a named config

Add your target joint vector to `NAMED_CONFIGS` in
[`h12_ros2_controller/utility/named_config.py`](h12_ros2_controller/utility/named_config.py):

```python
import numpy as np

from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS

NAMED_CONFIGS = {
    'home': np.zeros(len(ENABLED_JOINTS)),

    # Example: lift both elbows ~45 degrees, everything else neutral.
    'elbows_up': np.array([
        0.0,  0.0,  0.0,  0.78,  0.0,  0.0,  0.0,   # left arm
        0.0,  0.0,  0.0,  0.78,  0.0,  0.0,  0.0,   # right arm
    ]),
}
```

Rebuild so the server picks up the new config, then restart the launch:

```bash
cd ~/ws_h12 && colcon build --packages-select h12_ros2_controller
source install/setup.bash
# restart robot_launch.py
```

(If you built with `--symlink-install`, you only need to restart the server.)

### Step 2 — Send the config

**Option A — interactive client.** The `frame_task_client` prompts you for a
config name:

```bash
ros2 run h12_ros2_controller frame_task_client
# Available config names: ['home', 'elbows_up']
# Enter config name (or press Enter for manual frame task): elbows_up
```

Press `BACKSPACE` to cancel an in-progress goal.

**Option B — one-off from the command line.** Send the action goal directly
with `ros2 action send_goal`. The `duration` is how long the controller has to
reach the target (give it a few seconds; longer = slower, gentler motion):

```bash
ros2 action send_goal /named_config custom_ros_messages/action/NamedConfig \
    "{config_name: 'elbows_up', duration: {sec: 5, nanosec: 0}}" --feedback
```

You'll see `joint_error` feedback shrink toward zero as the arms converge, then
a `success: true` result.

### Reading the current joint angles

To build a new config, it helps to read where the arms are now. The
`joint_state_publisher` publishes `sensor_msgs/JointState` on `/joint_states`
(this covers all body joints; pick out the arm joints by name from the list
above):

```bash
ros2 topic echo /joint_states --once
```

---

## 5. Streaming setpoints (joint or frame)

This is the interface to use when you have your **own controller producing
targets** — for example a QP-based IK / collision-avoidance policy — and want
to feed its output into our pipeline at a steady rate. It streams either
**joint** configurations or **end-effector frame** poses.

### How it works

`stream_server` runs **one persistent control loop** at the controller rate
(`ctrl_hz`, 50 Hz by default). It always tracks the *latest* setpoint you sent,
in whichever mode matches your most recent message:

```
                          ┌─ JOINT mode (goto_reduced_configuration) ─┐
your policy (~10 Hz) ──► setpoint ──► 50 Hz loop ──► same PINK QP ──► robot
                          ├─ ROS:  Float64MultiArray on "joint_stream"
                          ├─ DDS:  MotorCmds_       on "rt/joint_stream"
                          └─ FRAME mode (control_step_reduced)
                             ROS:  PoseStamped      on "frame_stream"
```

Key properties:

- **Two modes, one loop.** The mode flips automatically to match the last
  setpoint received: a joint message → JOINT mode, a frame message → FRAME mode.
  Switching to JOINT mode clears any active frame tasks. Pick one mode per
  streaming session; don't interleave the two.
- **Rate decoupling.** You can publish slower than the control loop (e.g.
  10 Hz). The 50 Hz loop keeps tracking the most recent setpoint, so motion
  stays smooth between your updates. Newer setpoints simply overwrite older
  ones — there is no queue to fall behind.
- **Collision avoidance stays on.** *Both* modes solve through the same PINK
  reduced-model QP, with the **self-collision barrier and joint/velocity limits
  enabled** (JOINT mode is a config task; FRAME mode is frame tasks + a posture
  task). If your own collision avoidance is also a QP-IK, the two compose
  cleanly — yours shapes the target, ours guarantees self-collision and limit
  safety on the executed motion. (Your policy should still own
  *environment/obstacle* avoidance; the server only knows about the robot's own
  geometry.)
- **Holds on startup.** Starts in JOINT mode with the setpoint initialized to
  the current configuration, so the arms stay put until your first message.
- **Safety relay still applies.** Launch with `--config safety_full.yaml` (and
  the safety node, see [section 3](#running-with-the-safety-layer)) and the
  stream is clipped/e-stopped by the safety layer exactly like every other
  command path.

> Run `stream_server` **instead of** `frame_task_server` / `dual_arm_server` —
> they all command the same motors, so only one controller server should run at
> a time.

### Start the server

```bash
ros2 run h12_ros2_controller stream_server --config debug.yaml
```

### Joint mode — stream over ROS

The server subscribes to `std_msgs/Float64MultiArray` on **`joint_stream`**: the
same **14 arm joints in `ENABLED_JOINTS` order** as in
[section 4](#the-14-controllable-arm-joints), in radians. Quick smoke test:

```bash
ros2 topic pub -r 10 /joint_stream std_msgs/Float64MultiArray \
    "{data: [0,0,0,0.78,0,0,0, 0,0,0,0.78,0,0,0]}"
```

From your own Python node, publish each target your policy produces:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class PolicyStreamer(Node):
    def __init__(self):
        super().__init__('policy_streamer')
        self.pub = self.create_publisher(Float64MultiArray, 'joint_stream', 10)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

    def tick(self):
        q_target = my_ik_policy_step()      # -> list/array of 14 floats (rad)
        self.pub.publish(Float64MultiArray(data=list(q_target)))
```

### Joint mode — stream over straight DDS (no ROS)

If your code doesn't run inside ROS, publish a `unitree_go/MotorCmds_` message
on the Unitree DDS channel **`rt/joint_stream`**. The server reads the `.q`
field of the i-th entry as the target for `ENABLED_JOINTS[i]`:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_ as MotorCmd_default

ChannelFactoryInitialize(0)                 # match the controller's domain_id
pub = ChannelPublisher('rt/joint_stream', MotorCmds_)
pub.Init()

while running:
    q_target = my_ik_policy_step()          # 14 floats (rad), ENABLED_JOINTS order
    msg = MotorCmds_(cmds=[MotorCmd_default() for _ in range(len(q_target))])
    for cmd, q in zip(msg.cmds, q_target):
        cmd.q = float(q)
    pub.Write(msg)
    # sleep to your control rate, e.g. 0.1 s for 10 Hz
```

> Make sure the DDS **domain id** (and network interface) match the controller's
> `network:` config block, or the messages won't be delivered.

### Frame mode — stream over ROS

If your policy outputs **end-effector poses** instead of joint configurations,
publish `geometry_msgs/PoseStamped` on **`frame_stream`**. The
**`header.frame_id`** names the frame to drive (e.g. `left_ee`, `right_ee`, or
any frame in the URDF); the `pose` is the target in the `pelvis` frame. The
server adds/updates an IK frame task for that frame and tracks it.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class FrameStreamer(Node):
    def __init__(self):
        super().__init__('frame_streamer')
        self.pub = self.create_publisher(PoseStamped, 'frame_stream', 10)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

    def tick(self):
        msg = PoseStamped()
        msg.header.frame_id = 'left_ee'     # which frame to drive
        msg.pose = my_pose_policy_step()    # geometry_msgs/Pose target (in pelvis)
        self.pub.publish(msg)
```

To track **both arms** at once, stream `left_ee` and `right_ee` messages
(alternating is fine) — each frame keeps its own task until you switch to JOINT
mode.

> **Frame streaming is ROS-only.** Unitree's DDS IDL has no SE(3)/pose message
> type, so there is no clean straight-DDS carrier for poses. Joint targets map
> onto `MotorCmds_`; poses do not. If you need pose streaming without ROS, talk
> to the maintainers — it would require a custom DDS IDL.

### Tuning notes

- Keep per-update steps **small**. The loop limits joint/Cartesian velocity, so
  large jumps are rate-limited rather than tracked instantly — fine for safety,
  but your policy should already produce nearby successive targets.
- If you need a faster executed motion, the velocity ceilings live in the
  controller config under `controller: {v_lim, w_lim, dq_lim}`.
- There is currently **no feedback channel** on this interface. Read joint
  state from `/joint_states` (ROS) or `rt/lowstate` (DDS) to close your own
  loop; for frame mode, current frame poses are available via TF (run a launch
  file that starts `robot_state_publisher`).

---

## 6. End-effector (frame) commands — for reference

If instead of joints you want to command a wrist **pose** in SE(3), the same
server exposes the `FrameTask` action (`/frame_task`). The interactive
`frame_task_client` lets you enter a frame name and target pose. See the
[ROS Interface section of the README](README.md#ros-interface) for details.

---

## Safety Notes

- **Keep the physical e-stop in hand.** Software is not a substitute.
- **Validate in sim/debug first** (`config:=debug.yaml`) before hardware.
- **Use generous durations** (5 s or more) for your first moves so the arms
  travel slowly.
- **Sanity-check your joint vector** — wrong sign or wrong index can swing an
  arm unexpectedly. Compare against `/joint_states` before sending.
- **Stay clear of the workspace** while a goal is executing.
- Prefer the **safety-layer launch** (`robot_safety_launch.py`) on hardware
  once you're comfortable with the basics.

---

## Quick reference

```bash
# Build (from workspace root)
colcon build --packages-select custom_ros_messages h12_ros2_model h12_ros2_controller
source install/setup.bash

# Start the controller (serves /named_config and /frame_task)
ros2 launch h12_ros2_controller robot_launch.py config:=debug.yaml

# Send a one-off joint-space goal by name
ros2 action send_goal /named_config custom_ros_messages/action/NamedConfig \
    "{config_name: 'home', duration: {sec: 5, nanosec: 0}}" --feedback

# --- OR: stream setpoints (run instead of the action server) ---
ros2 run h12_ros2_controller stream_server --config debug.yaml
# JOINT setpoints over ROS (14 floats, ENABLED_JOINTS order):
ros2 topic pub -r 10 /joint_stream std_msgs/Float64MultiArray \
    "{data: [0,0,0,0.78,0,0,0, 0,0,0,0.78,0,0,0]}"
# (or publish unitree_go/MotorCmds_ on the DDS channel rt/joint_stream)
# FRAME setpoints over ROS (header.frame_id names the frame):
ros2 topic pub -r 10 /frame_stream geometry_msgs/PoseStamped \
    "{header: {frame_id: 'left_ee'}, pose: {position: {x: 0.3, y: 0.2, z: 0.1}}}"

# Read current joint angles
ros2 topic echo /joint_states --once
```
