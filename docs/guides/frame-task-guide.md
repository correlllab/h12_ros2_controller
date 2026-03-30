# Frame Task Architecture Guide

## Overview

The **frame task** system is a generalized inverse kinematics (IK) based control framework that allows you to specify desired poses for **any named frame on the robot**, not just end-effectors. This provides a flexible way to:

- Command end-effectors to target poses
- Control intermediate links or body frames
- Combine multiple frame tasks simultaneously
- Apply hierarchical control constraints

Unlike the simpler `dual_arm_server` (which controls only left and right end-effectors), `frame_task_server` accepts arbitrary frame names and can handle multiple frames in parallel.

## Core Concepts

### What are Frame Tasks?

A **frame task** is a desired pose (position + orientation) specification for a named rigid body frame on the robot. The system uses inverse kinematics to compute joint velocities that move that frame toward the target pose.

**Key properties:**
- Each task has a unique `task_name` (e.g., `"left_wrist"_task`, `"torso_task"`)
- References a robot **frame name** from the URDF (e.g., `"left_wrist_yaw_link"`, `"torso_link"`)
- Specifies a target pose as either:
  - A 6D pose vector: `[x, y, z, roll, pitch, yaw]`
  - A 4×4 transformation matrix

### IK Solver Integration

The `IKSolver` maintains a set of active frame tasks and solves for joint velocities that:

1. **Minimize task errors** - drives each frame toward its target pose
2. **Respect constraints** - joint limits, velocity limits, collision avoidance
3. **Maintain posture** - penalizes deviation from neutral joint positions

The underlying solver uses **Pink** (a QP-based IK library) with costs for:
- **Position error:** 50.0 cost
- **Orientation error:** 30.0 cost
- **Posture error:** 1e-3 cost (low priority)

### Architecture Layers

```
┌─────────────────────────────────────────┐
│  ROS2 Layer (frame_task_client/server)  │  Action interface, topic publishing
├─────────────────────────────────────────┤
│  FrameController                         │  High-level frame task management
├─────────────────────────────────────────┤
│  IKSolver                               │  Task-based IK computation
│  - frame_tasks (dict)                   │  - Validates constraints
│  - ik_step_reduced()                    │  - Returns joint velocities
├─────────────────────────────────────────┤
│  RobotModel                             │  Kinematics, dynamics, transforms
│  - get_frame_transformation()           │  - Collision checking
│  - get_frame_position()                 │
├─────────────────────────────────────────┤
│  LowCmdHandler → ChannelInterface       │  Safety checks, hardware publishing
├─────────────────────────────────────────┤
│  Hardware (Unitree SDK)                 │  Motor commands and state feedback
└─────────────────────────────────────────┘
```

## Usage: Server/Client Pattern

### Frame Task Server

The `frame_task_server` ROS2 node runs the control loop:

```bash
ros2 run h12_ros2_controller frame_task_server
```

**Key responsibilities:**
- Accepts `frame_task` action goals with frame names and target poses
- Runs IK solver at ~33 Hz (dt=0.03s)
- Publishes feedback with linear/angular errors for each frame
- Publishes convenience topics for monitoring

**Publishes (100 Hz):**
- `/left_ee_pose` - current left end-effector pose
- `/right_ee_pose` - current right end-effector pose
- `/left_ee_target` - target left end-effector pose (if set)
- `/right_ee_target` - target right end-effector pose (if set)
- `/frame_names` - active frame names (StringArray)
- `/frame_targets` - target poses (PoseArray)
- `/frame_poses` - current frame poses (PoseArray)

### Frame Task Client

Program your robot via `frame_task_client`:

```bash
ros2 run h12_ros2_controller frame_task_client
```

**Interactive mode:**
1. Enter named configuration name (e.g., `"home"`) or proceed to manual task
2. For manual frame tasks:
   - Enter frame name (e.g., `"left_wrist_yaw_link"`)
   - Enter target pose: `x y z roll pitch yaw`
   - Repeat for additional frames, or finish
3. Robot moves to target(s)
4. Press BACKSPACE to cancel

**Available named configs:**
See [Named Configurations](#named-configurations) section.

### Programmatic Usage

```python
from h12_ros2_controller.ros2.frame_task_client import FrameTaskClient

client = FrameTaskClient()

# Move two frames simultaneously
frame_names = ['left_wrist_yaw_link', 'right_wrist_yaw_link']
frame_targets = [
    Pose(position=Point(0.3, 0.2, 0.1),
         orientation=Quaternion(0, 0, 0, 1)),
    Pose(position=Point(0.3, -0.2, 0.1),
         orientation=Quaternion(0, 0, 0, 1))
]
client.send_frame_task_goal(frame_names, frame_targets)
```

## Frame Reference

Robot frame names you can control:

### End-Effectors (Primary Use)
- `left_wrist_yaw_link` - left hand pose
- `right_wrist_yaw_link` - right hand pose

### Intermediate Links
- `left_shoulder_roll_link`
- `left_shoulder_pitch_link`
- `left_shoulder_roll_link`
- `left_elbow_pitch_link`
- `right_*` (symmetric with left)

### Body Link
- `torso_link` - robot torso/trunk

### Pelvis (Base)
- `pelvis` - floating base frame

**All frame names must exist in the URDF.** Check your robot URDF for available frames:

```bash
grep '<link name=' assets/h1_2/h1_2.urdf
```

## How It Works: Detailed Flow

### 1. Goal Reception

Client sends action goal:
```python
goal_msg.frame_names = ['left_wrist_yaw_link', 'right_wrist_yaw_link']
goal_msg.frame_targets = [left_pose, right_pose]
```

### 2. Server Initialization

Server clears old tasks and adds new frame tasks:
```python
for frame_name, frame_target in zip(goal.frame_names, goal.frame_targets):
    task_name = f'{frame_name}_task'
    self.controller.add_frame_task(task_name, frame_name, pose_to_matrix(frame_target))
```

The `IKSolver` creates a `pink.tasks.FrameTask` for each frame:
```python
frame_task = pink.tasks.FrameTask(
    frame_name,
    position_cost=50.0,      # Position tracking priority
    orientation_cost=30.0,    # Orientation tracking priority
    lm_damping=3.0            # IK numerical damping
)
```

### 3. Control Loop (Every dt=0.03s)

For each iteration:

**a. Compute IK:**
```python
# Solve: minimize ||error||² subject to constraints
vel = self.controller.ik_step_reduced()  # Returns joint velocities
```

The IK solver handles:
- All frame task errors simultaneously
- Joint/velocity/acceleration limits
- Self-collision avoidance
- Configuration (posture) objectives

**b. Apply Command:**
```python
self.controller.control_step_reduced()  # Sends velocity to hardware
```

**c. Publish State:**
```python
# Publish current frame poses, target poses, errors
error = self.controller.get_frame_task_error(task_name)
errors_linear.append(np.linalg.norm(error[:3]))     # Position error magnitude
errors_angular.append(np.linalg.norm(error[3:]))    # Orientation error magnitude
```

**d. Check Convergence:**
```python
if max(errors_linear) < threshold_linear and max(errors_angular) < threshold_angular:
    goal_handle.succeed()
    break
```

### 4. Goal Result

Server publishes success/failure and sends final result message.

## Advanced Features

### Multi-Frame Coordination

Specify multiple frames to move them **in parallel** with coordinated IK:

```python
# Both arms move toward symmetric targets
client.send_frame_task_goal(
    ['left_wrist_yaw_link', 'right_wrist_yaw_link'],
    [left_target, right_target]  # Solved together, not sequentially
)
```

The IK solver computes a **single velocity command** that moves all frames toward their targets while respecting constraints. This enables:
- Coordinated arm motions
- Collision avoidance between arms
- Smooth, collision-free paths

### Hierarchical Control

Frame tasks have priorities based on cost:

| Cost        | Task                  | Priority |
|-------------|----------------------|----------|
| 50.0        | Position error        | Highest  |
| 30.0        | Orientation error     | High     |
| 30.0        | CoM stability (optional) | High   |
| 1e-3        | Posture (joint bias)  | Lowest   |

Higher-cost tasks are prioritized by the QP solver. This means:
- End-effector poses are achieved first
- Body stability is secondary
- Natural joint positions are fallback objective

### Dynamic Task Addition/Removal

Clear and rebuild tasks at runtime:

```python
self.controller.clear_frame_tasks()  # Remove all active tasks
self.controller.add_frame_task('new_task', 'frame_name')  # Add new task
```

## Named Configurations

Predefined joint configurations for common poses:

```python
NAMED_CONFIGS = {
    'home': [0.0, 0.5, ...],        # Home position
    'reach_high': [0.2, 1.2, ...],  # Arms reaching up
    'relax': [-0.1, 0.0, ...],      # Relaxed posture
    # ... more configs
}
```

Use via action:
```python
client.send_named_config_goal('home')
```

Or directly:
```python
from h12_ros2_controller.utility.named_config import NAMED_CONFIGS
q_target = NAMED_CONFIGS['home']
controller.goto_reduced_configuration(q_target)
```

## Comparison: Frame Task vs. Dual Arm Server

| Feature | Frame Task Server | Dual Arm Server |
|---------|-------------------|-----------------|
| **Frames controllable** | Any named frame | Only left/right EE |
| **Multi-frame** | ✓ Simultaneous | ✗ Sequential |
| **End-effector control** | ✓ Yes | ✓ Yes |
| **Intermediate link control** | ✓ Yes | ✗ No |
| **Use case** | General manipulation, multi-link reaching | Simple bilateral reaching |
| **Status** | Active (recommended) | Legacy (commented in robot_launch) |

## Real-Time Topics Reference

When `frame_task_server` is running:

```bash
# Monitor end-effector poses
ros2 topic echo /left_ee_pose
ros2 topic echo /right_ee_pose

# Monitor active frame tasks
ros2 topic echo /frame_names
ros2 topic echo /frame_poses
ros2 topic echo /frame_targets

# Monitor EE state (always published)
rviz2  # Shows TF tree and pose markers
```

## Troubleshooting

### Frames not moving
- Check frame name spelling against URDF
- Verify action server is running: `ros2 action list`
- Monitor errors: `ros2 action send_goal /frame_task ... --feedback`

### Collision detected
- Add collision avoidance margin in `d_min` parameter
- Reduce control speed or increase timeout
- Verify URDF collision geometry

### Jerky/unstable motion
- Reduce control gains in IK solver (position_cost, orientation_cost)
- Increase damping (lm_damping)
- Reduce velocity limits

## See Also

- [ROS2 Interfaces Guide](ros2-interfaces.md) - Full topic/action reference
- [System Overview](../architecture/system-overview.md) - Architecture layers
- [Safety Reference](../reference/safety.md) - Constraint enforcement
- [Core Controllers API](../api/core-controllers.md) - FrameController class reference
