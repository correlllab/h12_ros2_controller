#!/usr/bin/env python3
"""arm_plan A/B client (UNTESTED ON REAL — plan h12/arm_plan_merge_plan_2026-07-09.md P4).

Sends the SAME arm motion to both sides so the legs-only MPC's mode-2 preview
(numeric arm_aware_plan=2 in the stabilize model) matches what the arms
actually do:
  (a) a NamedConfig action goal to the FrameTask stack (which owns the real
      arms today), and
  (b) the identical joint goal + duration to the MPC node's "Arm Plan *" gRPC
      task parameters (the min-jerk segment every rollout then replays).

FrameTask executes its own diff-IK profile, not an exact min-jerk — preview
fidelity is approximate until the native upper-body controller authors plans.

Usage (robot up, h12_lower_body_controller running with arm_aware_plan=2):
  ros2 run h12_ros2_controller arm_plan_client --config arms_front_45 \
      --duration 1.5 [--grpc localhost:10000] [--no-frametask | --no-plan]

This file is a pure ADDITION — no FrameTask code is modified.
"""
import argparse
import sys
import time

# 14 arm joints, motor idx 13..26: L sh_p sh_r sh_y elbow wr_r wr_p wr_y, R same.
NAMED = {
    "home":          [0.0] * 14,
    "arms_front_45": [-0.75, 0, 0, 0, 0, 0, 0, -0.75, 0, 0, 0, 0, 0, 0],
    "arms_front":    [-1.45, 0, 0, 0, 0, 0, 0, -1.45, 0, 0, 0, 0, 0, 0],
}


def send_plan_grpc(addr, joints, duration):
    """Arm the MPC-side min-jerk segment via agent gRPC SetTaskParameters."""
    import grpc
    try:
        from mujoco_mpc.proto import agent_pb2, agent_pb2_grpc
    except ImportError:
        # deploy-tree fallback (same proto the twin bench uses)
        import os
        sys.path.insert(0, os.path.expanduser(
            "~/Desktop/h12/mujoco_mpc/mujoco_mpc/build/mjpc/agent_service"))
        import agent_pb2, agent_pb2_grpc
    channel = grpc.insecure_channel(addr)
    stub = agent_pb2_grpc.AgentStub(channel)
    params = {"Arm Plan Sec": float(duration)}
    for j, q in enumerate(joints):
        params[f"Arm Goal J{j}"] = float(q)
    req = agent_pb2.SetTaskParametersRequest()
    for k, v in params.items():
        req.parameters[k].numeric = v
    stub.SetTaskParameters(req, timeout=2.0)
    # rising edge AFTER the goal params are in place
    req2 = agent_pb2.SetTaskParametersRequest()
    req2.parameters["Arm Plan Active"].numeric = 1.0
    stub.SetTaskParameters(req2, timeout=2.0)


def clear_plan_grpc(addr):
    import grpc
    try:
        from mujoco_mpc.proto import agent_pb2, agent_pb2_grpc
    except ImportError:
        import os
        sys.path.insert(0, os.path.expanduser(
            "~/Desktop/h12/mujoco_mpc/mujoco_mpc/build/mjpc/agent_service"))
        import agent_pb2, agent_pb2_grpc
    stub = agent_pb2_grpc.AgentStub(grpc.insecure_channel(addr))
    req = agent_pb2.SetTaskParametersRequest()
    req.parameters["Arm Plan Active"].numeric = 0.0
    stub.SetTaskParameters(req, timeout=2.0)


def send_named_config(config, duration):
    """Fire the existing NamedConfig action (FrameTask stack, untouched)."""
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from custom_ros_messages.action import NamedConfig
    from builtin_interfaces.msg import Duration as DurationMsg

    rclpy.init()
    node = Node("arm_plan_client")
    client = ActionClient(node, NamedConfig, "named_config")
    if not client.wait_for_server(timeout_sec=3.0):
        node.get_logger().error("named_config action server not up")
        rclpy.shutdown()
        return False
    goal = NamedConfig.Goal()
    goal.config_name = config
    goal.duration = DurationMsg(sec=int(duration),
                                nanosec=int((duration % 1.0) * 1e9))
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    gh = fut.result()
    ok = bool(gh and gh.accepted)
    node.get_logger().info(f"NamedConfig '{config}' accepted={ok}")
    rclpy.shutdown()
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="arms_front_45",
                    help=f"named config ({'/'.join(NAMED)}) or use --joints")
    ap.add_argument("--joints", type=float, nargs=14, default=None,
                    help="explicit 14 joint targets (overrides --config)")
    ap.add_argument("--duration", type=float, default=1.5)
    ap.add_argument("--grpc", default="localhost:10000")
    ap.add_argument("--no-frametask", action="store_true",
                    help="only arm the MPC plan (twin/bench use)")
    ap.add_argument("--no-plan", action="store_true",
                    help="only send the FrameTask goal (condition A)")
    ap.add_argument("--clear", action="store_true",
                    help="drop Arm Plan Active and exit")
    a = ap.parse_args()

    if a.clear:
        clear_plan_grpc(a.grpc)
        print("plan cleared")
        return
    joints = a.joints if a.joints is not None else NAMED[a.config]

    if not a.no_plan:
        send_plan_grpc(a.grpc, joints, a.duration)
        print(f"plan armed: {a.config if a.joints is None else 'custom'} "
              f"T={a.duration}s -> {a.grpc}")
    if not a.no_frametask:
        # plan first, FrameTask second: the segment t0 latches on the MPC's
        # next plan cycle (~ms), so preview leads execution slightly — the
        # correct order for an APA.
        send_named_config(a.config, a.duration)
    if not a.no_plan:
        time.sleep(a.duration + 0.5)
        clear_plan_grpc(a.grpc)
        print("plan auto-cleared (ready for re-trigger)")


if __name__ == "__main__":
    main()
