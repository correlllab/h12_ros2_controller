import time
import threading
import numpy as np
from typing import Optional, List
from h12_ros2_controller.utility.robot_setting import setup_gains, JOINT_POSITION_LIMITS, JOINT_VELOCITY_LIMITS, JOINT_TORQUE_LIMITS
from h12_ros2_controller.core.channel_interface import LowCmdPublisher, ArmSDKPublisher

class LowCmdController:
    def __init__(self, robot_model, dt=0.005, sport_mode: bool=False):
        self.dt = dt
        self.robot_model = robot_model
        # intialize command publisher
        if sport_mode:
            self.command_publisher = ArmSDKPublisher(dt)
        else:
            self.command_publisher = LowCmdPublisher(dt)
        setup_gains(self.command_publisher)

        # background safety monitoring thread
        self._safety_thread = threading.Thread(
            target=self._safety_checker,
            name='safety_thread',
            daemon=True
        )

    def set_joint_commands(self,
                           q: Optional[np.ndarray]=None,
                           dq: Optional[np.ndarray]=None,
                           tau: Optional[np.ndarray]=None,
                           joint_ids: Optional[List[int]]=None):
        '''Set multiple joint commands atomically'''
        with self.command_publisher._data_lock:
            if joint_ids is None:
                if q is not None:
                    self.command_publisher.q[:] = q
                if dq is not None:
                    self.command_publisher.dq[:] = dq
                if tau is not None:
                    self.command_publisher.tau[:] = tau
            else:
                if q is not None:
                    self.command_publisher.q[joint_ids] = q
                if dq is not None:
                    self.command_publisher.dq[joint_ids] = dq
                if tau is not None:
                    self.command_publisher.tau[joint_ids] = tau

    def enable_motors(self, motor_ids, init_q=None):
        if init_q is None:
            init_q = self.robot_model.state['q'][motor_ids]
        self.command_publisher.enable_motors(motor_ids, init_q)

    def start(self):
        self.command_publisher.start()

    def shutdown(self):
        self.command_publisher.shutdown()

    def estop(self):
        self.command_publisher.estop()

    def _safety_checker(self):
        '''Background thread for safety monitoring'''
        while True:
            state = self.robot_model.state
            for i in range(len(state['q'])):
                # check position limits
                if (state['q'][i] < JOINT_POSITION_LIMITS[i]['low'] - 0.1 or
                    state['q'][i] > JOINT_POSITION_LIMITS[i]['high'] + 0.1):
                    print(f'Position limit exceeded on joint {i}: {state["q"][i]:.3f} rad')
                    self.estop()
                # check velocity limits
                if abs(state['dq'][i]) > JOINT_VELOCITY_LIMITS[i] + 0.1:
                    print(f'Velocity limit exceeded on joint {i}: {state["dq"][i]:.3f} rad/s')
                    self.estop()
                # check torque limits
                if abs(state['tau'][i]) > JOINT_TORQUE_LIMITS[i] + 0.1:
                    print(f'Torque limit exceeded on joint {i}: {state["tau"][i]:.3f} Nm')
                    self.estop()
