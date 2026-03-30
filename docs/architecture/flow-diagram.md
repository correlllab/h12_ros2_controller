# End-to-End Flow Diagram

A full topic/service/action and runtime flow from user command to robot actuation.

```mermaid
flowchart TD
    A[Operator / App] --> B{Entry Path}

    B -->|ROS2 launch| L1[launch/robot_launch.py]
    B -->|ROS2 launch| L2[launch/full_launch.py]
    B -->|Direct script| S1[example/frame_controller_goto.py]
    B -->|GUI| G1[ros2/hand_cmd_gui.py]

    L1 --> N1[robot_state_publisher]
    L1 --> N2[joint_state_publisher]
    L1 --> N5[frame_task_server]
    L1 --> N4[hand_controller_node]

    L2 --> N1
    L2 --> N2
    L2 --> N3[dual_arm_server]
    L2 --> R1[rviz2]

    C2[frame_task_client] -->|action goal| A3[/frame_task action/]
    C2 -->|action goal| A2[/named_config action/]
    C1[dual_arm_client] -->|action goal| A1[/dual_arm action/]
    C1 -->|action goal| A2

    A1 --> N3
    A2 --> N3
    A2 --> N5
    A3 --> N5

    N3 --> P1[/left_ee_pose topic/]
    N3 --> P2[/right_ee_pose topic/]
    N3 --> P3[/left_ee_target topic/]
    N3 --> P4[/right_ee_target topic/]

    N5 --> P1
    N5 --> P2
    N5 --> P3
    N5 --> P4
    N5 --> P5[/frame_names topic/]
    N5 --> P6[/frame_targets topic/]
    N5 --> P7[/frame_poses topic/]

    G1 -->|Float64MultiArray| T1[/left_hand_cmd topic/]
    G1 -->|Float64MultiArray| T2[/right_hand_cmd topic/]
    T1 --> N4
    T2 --> N4
    N4 --> T3[/left_hand_state topic/]
    N4 --> T4[/right_hand_state topic/]

    N2 --> J1[/joint_states topic/]
    N1 --> TF1[TF tree]
    J1 --> R1
    TF1 --> R1
    P1 --> R1
    P2 --> R1

    subgraph CoreControl[Core Control Stack]
        U1[UpperController]
        U2[ArmController]
        U3[FrameController]
        U4[HandController]
        K1[IKSolver]
        M1[RobotModel]
        H1[LowCmdHandler]
        D1[ChannelInterface]
    end

    N3 --> U2
    N5 --> U3
    N4 --> U4
    U2 --> K1
    U3 --> K1
    K1 --> M1
    U1 --> H1
    U2 --> H1
    U3 --> H1
    U4 --> D1
    H1 --> D1
    M1 --> D1

    subgraph DDS[Unitree DDS / Hardware]
        D2[rt/lowcmd]
        D3[rt/arm_sdk]
        D4[rt/lowstate]
        D5[rt/inspire/cmd]
        D6[rt/inspire/state]
        HW[Robot Motors + Hands]
    end

    D1 --> D2
    D1 --> D3
    D1 --> D5
    D4 --> D1
    D6 --> D1
    D2 --> HW
    D3 --> HW
    D5 --> HW
    HW --> D4
    HW --> D6

    subgraph Safety[Safety and Guard Rails]
        S2[Joint position limits]
        S3[Joint velocity limits]
        S4[Joint torque limits]
        E1[Emergency stop]
    end

    H1 --> S2
    H1 --> S3
    H1 --> S4
    S2 --> E1
    S3 --> E1
    S4 --> E1
    E1 --> D1
```

## Operational interpretation

- ROS2 clients are command front-ends; control authority is in the server/controller stack.
- **Recommended:** Use `frame_task_server` for general manipulation; `dual_arm_server` is legacy.
- `frame_task_server` publishes EE pose/target topics for monitoring and rviz visualization, whether or not frame_task goals are active.
- Safety checks are continuous and independent of action success.
- RViz receives state/target streams for observability; it does not command hardware.
- Direct scripts bypass ROS2 actions but still pass through the same core safety and low-command path.
