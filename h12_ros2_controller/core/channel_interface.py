import time
import numpy as np

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher

from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_, MotorCmds_, MotorCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_ as MotorCmd_default
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.utils.crc import CRC

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as LowState_default
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmd_default

TOPIC_LOWCMD = 'rt/lowcmd'
TOPIC_LOWSTATE = 'rt/lowstate'
TOPIC_HIGHSTATE = 'rt/sportmodestate'
TOPIC_HANDSTATE = 'rt/inspire/state'
TOPIC_HANDCMD = 'rt/inspire/cmd'

NUM_MOTOR = 27
NUM_HAND_DOF = 12

class StateSubscriber:
    def __init__(self):
        # variable tracking states
        self._q = np.zeros(NUM_MOTOR)
        self._dq = np.zeros(NUM_MOTOR)
        self._tau = np.zeros(NUM_MOTOR)

        # subscribe low state
        self.low_state_subscriber = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self.low_state_subscriber.Init(self.subscribe_low_state, 10)

    def subscribe_low_state(self, msg: LowState_):
        self.last_time = time.time()
        for i in range(NUM_MOTOR):
            self._q[i] = msg.motor_state[i].q
            self._dq[i] = msg.motor_state[i].dq
            self._tau[i] = msg.motor_state[i].tau_est

    @property
    def q(self):
        return np.copy(self._q)

    @property
    def dq(self):
        return np.copy(self._dq)

    @property
    def tau(self):
        return np.copy(self._tau)

class CommandPublisher:
    def __init__(self):
        # variables saving states
        self.mode = np.zeros(NUM_MOTOR, dtype=np.int32)
        self.q = np.zeros(NUM_MOTOR)
        self.dq = np.zeros(NUM_MOTOR)
        self.tau = np.zeros(NUM_MOTOR)
        self.kp = np.zeros(NUM_MOTOR)
        self.kd = np.zeros(NUM_MOTOR)

        # publish low command
        self.low_cmd_publisher = ChannelPublisher(TOPIC_LOWCMD, LowCmd_)
        self.low_cmd_publisher.Init()

        # initialize low command
        self.crc = CRC()
        self.low_cmd = LowCmd_default()
        self.low_cmd.mode_pr = 0
        self.low_cmd.mode_machine = 6

        # start publisher thread
        self.low_cmd_thread = RecurrentThread(
            interval=0.005,
            target=self.publish_low_cmd,
            name='low_cmd_thread'
        )

        print('CommandPublisher initialized.')
        print('All joints are locked in the initial position.')

    def publish_low_cmd(self):
        # start_time = time.time()
        for i in range(NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = self.mode[i]
            self.low_cmd.motor_cmd[i].q = self.q[i]
            self.low_cmd.motor_cmd[i].dq = self.dq[i]
            self.low_cmd.motor_cmd[i].tau = self.tau[i]
            self.low_cmd.motor_cmd[i].kp = self.kp[i]
            self.low_cmd.motor_cmd[i].kd = self.kd[i]
        # set CRC
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        # write to publisher
        self.low_cmd_publisher.Write(self.low_cmd)
        # print(f'CommandPublisher publish time: {time.time() - start_time:.6f} seconds')

    def enable_motor(self, motor_ids, init_q):
        motor_ids, init_q = np.array(motor_ids), np.array(init_q)
        assert len(motor_ids) == len(init_q), 'Motor IDs and initial positions must have the same length.'
        assert np.all(motor_ids < NUM_MOTOR) and np.all(motor_ids >= 0), f'Motor IDs must be within [0, {NUM_MOTOR}).'

        self.mode[motor_ids] = 1
        self.q[motor_ids] = init_q

    def start_publisher(self):
        self.low_cmd_thread.Start()

    def estop(self):
        self.mode = np.zeros(NUM_MOTOR, dtype=np.int32)

class HandSubscriber:
    def __init__(self):
        # variables tracking hand states
        self._q = np.zeros(NUM_HAND_DOF)

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
    def q_left(self):
        return np.copy(self._q[6:12])

    @property
    def q_right(self):
        return np.copy(self._q[0:6])

class HandPublisher:
    def __init__(self, dt=0.005):
        self.dt = dt
        # variables saving hand states
        self.q = np.zeros(NUM_HAND_DOF)

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

        print('HandPublisher initialized.')
        self.hand_cmd_thread.Start()

    def publish_hand_cmd(self):
        for i in range(NUM_HAND_DOF):
            self.hand_cmd.cmds[i].q = self.q[i]
        # write to publisher
        self.hand_cmd_publisher.Write(self.hand_cmd)
