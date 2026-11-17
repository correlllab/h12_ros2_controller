import time

import rclpy

from h12_ros2_controller.core.control_timing import ControlTiming


def node_control_timing(node, timeout):
    '''Use ROS time in simulation, including before the server executor starts'''
    if not node.get_parameter('use_sim_time').value:
        return None

    def now():
        if not rclpy.ok(context=node.context):
            raise RuntimeError('ROS context shut down; holding last command')
        # executor references may outlive membership after a temporary spin
        executor = node.executor
        if executor is None or node not in executor.get_nodes():
            rclpy.spin_once(node, timeout_sec=0.0)
        return node.get_clock().now().nanoseconds * 1e-9

    deadline = time.monotonic() + timeout
    while now() == 0.0:
        if time.monotonic() >= deadline:
            raise TimeoutError('No simulation clock received before controller startup')
        time.sleep(0.01)
    return ControlTiming(now)
