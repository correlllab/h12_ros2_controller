import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from h12_ros2_controller.core.controller.gravity_comp_controller import GravityCompController
from h12_ros2_controller.utility.dds_init import init_channel_factory_from_env
from h12_ros2_controller.utility.path_definition_ros import URDF_MAGPIE_PIN_PATH, URDF_MAGPIE_SPHERE_PATH, SRDF_MAGPIE_SPHERE_PATH


class GravityCompServer(Node):
    def __init__(self, dt=0.03):
        super().__init__('gravity_comp_server')
        init_channel_factory_from_env()

        self.controller = GravityCompController(
            URDF_MAGPIE_PIN_PATH,
            URDF_MAGPIE_SPHERE_PATH,
            SRDF_MAGPIE_SPHERE_PATH,
            dt=dt,
            visualize=False,
            sport_mode=False,
        )

        self._loop_thread = None
        self._loop_running = threading.Event()
        self._lock = threading.Lock()

        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.mode_publisher = self.create_publisher(Bool, '/h12/gravity_comp_active', mode_qos)
        self.mode_service = self.create_service(SetBool, '/h12/set_gravity_comp', self._set_gravity_comp)

        self._publish_mode(active=False)
        self.get_logger().info('Gravity Comp Server initialized')

    def _publish_mode(self, active: bool):
        msg = Bool()
        msg.data = active
        self.mode_publisher.publish(msg)

    def _loop(self):
        self.get_logger().info('Gravity compensation loop started')
        while self._loop_running.is_set() and rclpy.ok():
            start_time = time.time()
            self.controller.gravity_compensation_step()
            time.sleep(max(0.0, self.controller.dt - (time.time() - start_time)))
        self.get_logger().info('Gravity compensation loop stopped')

    def _start_loop(self):
        if self._loop_thread is not None and self._loop_thread.is_alive():
            return False
        self._loop_running.set()
        self._loop_thread = threading.Thread(target=self._loop, daemon=True)
        self._loop_thread.start()
        self._publish_mode(active=True)
        return True

    def _stop_loop(self):
        if self._loop_thread is None:
            self._publish_mode(active=False)
            return False

        self._loop_running.clear()
        self._loop_thread.join(timeout=2.0)
        self._loop_thread = None
        self._publish_mode(active=False)
        return True

    def _set_gravity_comp(self, request, response):
        with self._lock:
            if request.data:
                started = self._start_loop()
                response.success = True
                response.message = 'Gravity compensation enabled' if started else 'Gravity compensation already enabled'
            else:
                stopped = self._stop_loop()
                response.success = True
                response.message = 'Gravity compensation disabled' if stopped else 'Gravity compensation already disabled'
        return response

    def shutdown(self):
        with self._lock:
            self._stop_loop()
            self.controller.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GravityCompServer(dt=0.03)
    try:
        rclpy.spin(node, executor=rclpy.executors.MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
