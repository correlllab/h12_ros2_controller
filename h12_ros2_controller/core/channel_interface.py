import time
import threading
import numpy as np
from abc import ABC, abstractmethod

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher

from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_, MotorCmds_, MotorCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_ as MotorCmd_default
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.utils.crc import CRC

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as LowState_default
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmd_default
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__IMUState_ as IMUState_default

from h12_ros2_controller.utility.controller_config import get_publisher_clip_limits
from h12_ros2_controller.utility.joint_definition import NUM_MOTOR, NUM_HAND_DOF

# TOPIC_LOWCMD = 'rt/lowcmd'
# TOPIC_LOWCMD = 'rt/safety/lowcmd_in'
TOPIC_LOWCMD = 'rt/lowcmd'
TOPIC_LOWSTATE = 'rt/lowstate'
TOPIC_HIGHSTATE = 'rt/sportmodestate'
TOPIC_HANDSTATE = 'rt/inspire/state'
TOPIC_HANDCMD = 'rt/inspire/cmd'
TOPIC_ARM_SDK = 'rt/arm_sdk'

INDEX_NOT_USED = NUM_MOTOR

class StateSubscriber:
    def __init__(self, low_state_topic=TOPIC_LOWSTATE):
        # variable tracking states
        self._time_stamp = 0.0
        self._imu_state = IMUState_default()
        self._mode = np.zeros(NUM_MOTOR, dtype=np.uint8)
        self._q = np.zeros(NUM_MOTOR, dtype=np.float32)
        self._dq = np.zeros(NUM_MOTOR, dtype=np.float32)
        self._ddq = np.zeros(NUM_MOTOR, dtype=np.float32)
        self._tau = np.zeros(NUM_MOTOR, dtype=np.float32)
        self._temperature = np.zeros((NUM_MOTOR, 2), dtype=np.int16)
        self._vol = np.zeros(NUM_MOTOR, dtype=np.float32)
        self._sensor = np.zeros((NUM_MOTOR, 2), dtype=np.uint32)
        self._motor_state = np.zeros(NUM_MOTOR, dtype=np.uint32)

        # subscribe low state
        self._subscriber_lock = threading.Lock()
        self._low_state_subscriber = ChannelSubscriber(low_state_topic, LowState_)
        self._low_state_subscriber.Init(self._subscribe_low_state, 10)

    def _subscribe_low_state(self, msg: LowState_):
        with self._subscriber_lock:
            self._time_stamp = time.time()
            self._imu_state = msg.imu_state
            for i in range(NUM_MOTOR):
                self._mode[i] = msg.motor_state[i].mode
                self._q[i] = msg.motor_state[i].q
                self._dq[i] = msg.motor_state[i].dq
                self._ddq[i] = msg.motor_state[i].ddq
                self._tau[i] = msg.motor_state[i].tau_est
                self._temperature[i] = msg.motor_state[i].temperature
                self._vol[i] = msg.motor_state[i].vol
                self._sensor[i] = msg.motor_state[i].sensor
                self._motor_state[i] = msg.motor_state[i].motorstate

    @property
    def state(self):
        with self._subscriber_lock:
            return {
                'time_stamp': self._time_stamp,
                'imu_state': self._imu_state,
                'mode': self._mode.copy(),
                'q': self._q.copy(),
                'dq': self._dq.copy(),
                'ddq': self._ddq.copy(),
                'tau': self._tau.copy(),
                'temperature': self._temperature.copy(),
                'vol': self._vol.copy(),
                'sensor': self._sensor.copy(),
                'motor_state': self._motor_state.copy(),
            }

    def shutdown(self):
        self._low_state_subscriber.Close()
        print('StateSubscriber shutdown', flush=True)

class CommandPublisher(ABC):
    '''Base class for command publishers with shared functionality.'''
    def __init__(self, dt=0.005, clip_limits=None):
        self._dt = dt
        # data fileds
        self.mode = np.zeros(NUM_MOTOR, dtype=np.int32)
        self.q = np.zeros(NUM_MOTOR, dtype=np.float32)
        self.dq = np.zeros(NUM_MOTOR, dtype=np.float32)
        self.tau = np.zeros(NUM_MOTOR, dtype=np.float32)
        self.kp = np.zeros(NUM_MOTOR, dtype=np.float32)
        self.kd = np.zeros(NUM_MOTOR, dtype=np.float32)

        # shared threading interface
        self.data_lock = threading.Lock()
        self._publisher = None
        self._publishing = False
        self._publishing_thread = None

        # shared low_cmd object
        self._crc = None
        self._low_cmd = None

        effective_clip_limits = get_publisher_clip_limits() if clip_limits is None else clip_limits
        self._position_clip_limits = effective_clip_limits['position_clip']
        self._velocity_clip_limits = effective_clip_limits['velocity_clip']
        self._torque_clip_limits = effective_clip_limits['torque_clip']

        # initialize publisher-specific components
        self._init_low_cmd()
        self._init_publisher()
        self._init_thread()

        print(f'{self.__class__.__name__} initialized.', flush=True)
        print('All joints are locked in the initial position.', flush=True)

    @abstractmethod
    def _init_low_cmd(self):
        '''Initialize low_cmd object'''
        pass

    @abstractmethod
    def _init_publisher(self):
        '''Initialize publisher with desired topic and message types'''
        pass

    @abstractmethod
    def _init_thread(self):
        '''Initialize publisher thread'''
        pass

    def _publish_command(self):
        '''Publishing loop for low commands'''
        while self._publishing:
            start_time = time.time()
            with self.data_lock:
                # assert data integrity before publishing
                self._check_data_integrity()
                # write to low command
                for i in range(NUM_MOTOR):
                    # check position, velocity, torque limits
                    self._enforce_limits(i)
                    self._low_cmd.motor_cmd[i].mode = self.mode[i]
                    self._low_cmd.motor_cmd[i].q = self.q[i]
                    self._low_cmd.motor_cmd[i].dq = self.dq[i]
                    self._low_cmd.motor_cmd[i].tau = self.tau[i]
                    self._low_cmd.motor_cmd[i].kp = self.kp[i]
                    self._low_cmd.motor_cmd[i].kd = self.kd[i]
            # set CRC
            self._low_cmd.crc = self._crc.Crc(self._low_cmd)
            # write to publisher
            self._publisher.Write(self._low_cmd)
            # sleep to maintain publishing rate
            time.sleep(max(0, self._dt - (time.time() - start_time)))
            # print(f'LowCmdPublisher publish time: {time.time() - start_time:.6f} seconds')

    def _check_data_integrity(self):
        '''Check data integrity'''
        assert np.all(np.logical_or(self.mode == 0, self.mode == 1)), 'Motor mode should be either 0 (disabled) or 1 (enabled).'
        assert not np.isnan(self.q).any(), 'Position command should not contain NaN.'
        assert not np.isinf(self.q).any(), 'Position command should not contain Inf.'
        assert not np.isnan(self.dq).any(), 'Velocity command should not contain NaN.'
        assert not np.isinf(self.dq).any(), 'Velocity command should not contain Inf.'
        assert not np.isnan(self.tau).any(), 'Torque command should not contain NaN.'
        assert not np.isinf(self.tau).any(), 'Torque command should not contain Inf.'
        assert not np.isnan(self.kp).any(), 'Kp command should not contain NaN.'
        assert not np.isinf(self.kp).any(), 'Kp command should not contain Inf.'
        assert not np.isnan(self.kd).any(), 'Kd command should not contain NaN.'
        assert not np.isinf(self.kd).any(), 'Kd command should not contain Inf.'

    def _enforce_limits(self, i: int):
        '''Enforce joint limits for a specific joint index'''
        # enforce position limit
        if self.q[i] < self._position_clip_limits[i]['low']:
            self.q[i] = self._position_clip_limits[i]['low']
        if self.q[i] > self._position_clip_limits[i]['high']:
            self.q[i] = self._position_clip_limits[i]['high']
        # enforce velocity limit
        if abs(self.dq[i]) > self._velocity_clip_limits[i]:
            self.dq[i] = np.sign(self.dq[i]) * self._velocity_clip_limits[i]
        # enforce torque limit
        if abs(self.tau[i]) > self._torque_clip_limits[i]:
            self.tau[i] = np.sign(self.tau[i]) * self._torque_clip_limits[i]

    def enable_motors(self, motor_ids, init_q):
        '''Enable motors with given IDs and initial positions'''
        motor_ids, init_q = np.array(motor_ids), np.array(init_q)
        assert len(motor_ids) == len(init_q), 'Motor IDs and initial positions must have the same length.'
        assert np.all(motor_ids < NUM_MOTOR) and np.all(motor_ids >= 0), f'Motor IDs must be within [0, {NUM_MOTOR}).'

    def start(self):
        '''Start the publisher thread'''
        self._publishing = True
        self._publishing_thread.start()

    def shutdown(self):
        '''Shutdown the publisher'''
        self.estop()
        time.sleep(self._dt)
        self._publishing = False
        self._publisher.Close()
        print(f'{self.__class__.__name__} shutdown.', flush=True)

    def estop(self):
        '''Emergency stop logic'''
        with self.data_lock:
            self.mode.fill(0)
            self.tau.fill(0.0)
            self.kp.fill(0.0)
            self.kd.fill(0.0)

class LowCmdPublisher(CommandPublisher):
    '''Wrapper class of publisher publishing to LowCmd topic'''
    def __init__(self, dt=0.005, low_cmd_topic=TOPIC_LOWCMD, clip_limits=None):
        self._low_cmd_topic = low_cmd_topic
        super().__init__(dt=dt, clip_limits=clip_limits)

    def _init_low_cmd(self):
        # initialize low command message
        self._crc = CRC()
        self._low_cmd = LowCmd_default()
        self._low_cmd.mode_pr = 0
        self._low_cmd.mode_machine = 6

    def _init_publisher(self):
        '''Initialize low command publisher'''
        self._publisher = ChannelPublisher(self._low_cmd_topic, LowCmd_)
        self._publisher.Init()

    def _init_thread(self):
        '''Initialize threading for low command publisher'''
        self._publishing_thread = threading.Thread(
            target=self._publish_command,
            name='low_cmd_thread',
            daemon=True
        )

    def enable_motors(self, motor_ids, init_q):
        super().enable_motors(motor_ids, init_q)
        with self.data_lock:
            self.mode[motor_ids] = 1
            self.q[motor_ids] = init_q

class ArmSDKPublisher(CommandPublisher):
    '''ARM SDK command publisher using RecurrentThread approach.'''
    def __init__(self, dt=0.005, arm_sdk_topic=TOPIC_ARM_SDK, clip_limits=None):
        self._arm_sdk_topic = arm_sdk_topic
        super().__init__(dt=dt, clip_limits=clip_limits)

    def _init_low_cmd(self):
        # initialize low command message
        self._crc = CRC()
        self._low_cmd = LowCmd_default()

    def _init_publisher(self):
        '''Initialize ARM SDK publisher'''
        self._publisher = ChannelPublisher(self._arm_sdk_topic, LowCmd_)
        self._publisher.Init()

    def _init_thread(self):
        '''Initialize threading for ARM SDK publisher'''
        self._publishing_thread = threading.Thread(
            target=self._publish_command,
            name='arm_sdk_thread',
            daemon=True
        )

    def enable_motors(self, motor_ids, init_q):
        '''Override to add ARM SDK specific enable logic'''
        super().enable_motors(motor_ids, init_q)
        with self.data_lock:
            self._low_cmd.motor_cmd[INDEX_NOT_USED].q = 1.0
            self.q[motor_ids] = init_q

class HandSubscriber:
    def __init__(self):
        # variables tracking hand states
        self._q = np.zeros(NUM_HAND_DOF, dtype=np.float32)

        # subscribe hand state
        self.hand_state_subscriber = ChannelSubscriber(TOPIC_HANDSTATE, MotorStates_)
        self.hand_state_subscriber.Init(self.subscribe_hand_state, 10)

    def subscribe_hand_state(self, msg: MotorStates_):
        for i in range(NUM_HAND_DOF):
            self._q[i] = msg.states[i].q

    @property
    def q(self):
        return np.copy(self._q)

    @property
    def q_right(self):
        return np.copy(self._q[0:6])

    @property
    def q_left(self):
        return np.copy(self._q[6:12])

class HandPublisher:
    def __init__(self, dt=0.005):
        self.dt = dt
        # variables saving hand states
        self.q = np.zeros(NUM_HAND_DOF, dtype=np.float32)

        # publish hand command
        self.hand_cmd_publisher = ChannelPublisher(TOPIC_HANDCMD, MotorCmds_)
        self.hand_cmd_publisher.Init()

        # initialize hand command
        self.hand_cmd = MotorCmds_()
        self.hand_cmd.cmds = [MotorCmd_default() for _ in range(NUM_HAND_DOF)]

        # start publisher thread
        self.hand_cmd_thread = RecurrentThread(
            interval=self.dt,
            target=self.publish_hand_cmd,
            name='hand_cmd_thread'
        )

        print('HandPublisher initialized.', flush=True)
        self.hand_cmd_thread.Start()

    def publish_hand_cmd(self):
        for i in range(NUM_HAND_DOF):
            self.hand_cmd.cmds[i].q = self.q[i]
        # write to publisher
        self.hand_cmd_publisher.Write(self.hand_cmd)
