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

from h12_ros2_controller.utility.joint_definition import BODY_JOINTS
from h12_ros2_controller.utility.robot_setting import JOINT_POSITION_LIMITS, JOINT_VELOCITY_LIMITS, JOINT_TORQUE_LIMITS

TOPIC_LOWCMD = 'rt/lowcmd'
TOPIC_LOWSTATE = 'rt/lowstate'
TOPIC_HIGHSTATE = 'rt/sportmodestate'
TOPIC_HANDSTATE = 'rt/inspire/state'
TOPIC_HANDCMD = 'rt/inspire/cmd'
TOPIC_ARM_SDK = 'rt/arm_sdk'

NUM_MOTOR = 27
NUM_HAND_DOF = 12
INDEX_NOT_USED = NUM_MOTOR

class StateSubscriber:
    def __init__(self):
        # variable tracking states
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
        self._low_state_subscriber = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self._low_state_subscriber.Init(self._subscribe_low_state, 10)

    def _subscribe_low_state(self, msg: LowState_):
        self.last_time = time.time()
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
        return {
            'mode': np.copy(self._mode),
            'q': np.copy(self._q),
            'dq': np.copy(self._dq),
            'ddq': np.copy(self._ddq),
            'tau': np.copy(self._tau),
            'temperature': np.copy(self._temperature),
            'vol': np.copy(self._vol),
            'sensor': np.copy(self._sensor),
            'motor_state': np.copy(self._motor_state),
        }

    def shutdown(self):
        self._low_state_subscriber.Close()
        print('StateSubscriber shutdown')

class CommandPublisher:
    def __init__(self):
        # variables saving states
        self.mode = np.zeros(NUM_MOTOR, dtype=np.int32)
        self.q = np.zeros(NUM_MOTOR, dtype=np.float32)
        self.dq = np.zeros(NUM_MOTOR, dtype=np.float32)
        self.tau = np.zeros(NUM_MOTOR, dtype=np.float32)
        self.kp = np.zeros(NUM_MOTOR, dtype=np.float32)
        self.kd = np.zeros(NUM_MOTOR, dtype=np.float32)

        # publish low command
        self._low_cmd_publisher = ChannelPublisher(TOPIC_LOWCMD, LowCmd_)
        self._low_cmd_publisher.Init()

        # initialize low command
        self._crc = CRC()
        self._low_cmd = LowCmd_default()
        self._low_cmd.mode_pr = 0
        self._low_cmd.mode_machine = 6

        # start publisher thread
        self._low_cmd_thread = RecurrentThread(
            interval=0.005,
            target=self._publish_low_cmd,
            name='low_cmd_thread'
        )

        print('CommandPublisher initialized.')
        print('All joints are locked in the initial position.')

    def _publish_low_cmd(self):
        # start_time = time.time()
        for i in range(NUM_MOTOR):
            # check position, velocity, torque limits
            safe = self._check_limits(i)
            self._low_cmd.motor_cmd[i].mode = self.mode[i]
            if safe:
                self._low_cmd.motor_cmd[i].q = self.q[i]
                self._low_cmd.motor_cmd[i].dq = self.dq[i]
                self._low_cmd.motor_cmd[i].tau = self.tau[i]
                self._low_cmd.motor_cmd[i].kp = self.kp[i]
                self._low_cmd.motor_cmd[i].kd = self.kd[i]
        # set CRC
        self._low_cmd.crc = self._crc.Crc(self._low_cmd)
        # write to publisher
        self._low_cmd_publisher.Write(self._low_cmd)
        # print(f'CommandPublisher publish time: {time.time() - start_time:.6f} seconds')

    def _check_limits(self, i):
        if (
            self.q[i] < JOINT_POSITION_LIMITS[i]['low'] or
            self.q[i] > JOINT_POSITION_LIMITS[i]['high']
        ):
            print(f'Position limit of joint {i} {BODY_JOINTS[i]} exceeded. Estopping...')
            print(f'  Commanded Position: {self.q[i]:.4f} rad')
            print(f'  Lower Limit: {JOINT_POSITION_LIMITS[i]["low"]:.4f} rad')
            print(f'  Upper Limit: {JOINT_POSITION_LIMITS[i]["high"]:.4f} rad')
            self.estop()
            return False
        if abs(self.dq[i]) > JOINT_VELOCITY_LIMITS[i]:
            print(f'Velocity limit of joint {i} {BODY_JOINTS[i]} exceeded. Estopping...')
            print(f'  Commanded Velocity: {self.dq[i]:.4f} rad/s')
            print(f'  Velocity Limit: {JOINT_VELOCITY_LIMITS[i]:.4f} rad/s')
            self.estop()
            return False
        if abs(self.tau[i]) > JOINT_TORQUE_LIMITS[i]:
            print(f'Torque limit of joint {i} {BODY_JOINTS[i]} exceeded. Estopping...')
            print(f'  Commanded Torque: {self.tau[i]:.4f} Nm')
            print(f'  Torque Limit: {JOINT_TORQUE_LIMITS[i]:.4f} Nm')
            self.estop()
            return False

        return True

    def enable_motor(self, motor_ids, init_q):
        motor_ids, init_q = np.array(motor_ids), np.array(init_q)
        assert len(motor_ids) == len(init_q), 'Motor IDs and initial positions must have the same length.'
        assert np.all(motor_ids < NUM_MOTOR) and np.all(motor_ids >= 0), f'Motor IDs must be within [0, {NUM_MOTOR}).'

        self.mode[motor_ids] = 1
        self.q[motor_ids] = init_q

    def start_publisher(self):
        self._low_cmd_thread.Start()

    def estop(self):
        self.mode.fill(0)
        self.kp.fill(0.0)
        self.kd.fill(0.0)

    def shutdown(self):
        self.estop()
        self._low_cmd_thread.Wait(0)
        self._low_cmd_publisher.Close()
        print('CommandPublisher shutdown.')

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

        print('HandPublisher initialized.')
        self.hand_cmd_thread.Start()

    def publish_hand_cmd(self):
        for i in range(NUM_HAND_DOF):
            self.hand_cmd.cmds[i].q = self.q[i]
        # write to publisher
        self.hand_cmd_publisher.Write(self.hand_cmd)

class ArmSDKPublisher:
    def __init__(self):
        # variables saving states
        self.q = np.zeros(NUM_MOTOR, dtype=np.float32)
        self.dq = np.zeros(NUM_MOTOR, dtype=np.float32)
        self.tau = np.zeros(NUM_MOTOR, dtype=np.float32)
        self.kp = np.zeros(NUM_MOTOR, dtype=np.float32)
        self.kd = np.zeros(NUM_MOTOR, dtype=np.float32)

        # publish arm sdk command
        self.arm_cmd_publisher = ChannelPublisher(TOPIC_ARM_SDK, LowCmd_)
        self.arm_cmd_publisher.Init()

        # initialize arm sdk command
        self.crc = CRC()
        self.arm_sdk_cmd = LowCmd_default()

        # start publisher thread
        self.arm_sdk_thread = RecurrentThread(
            interval=0.005,
            target=self.publish_arm_sdk_cmd,
            name='arm_sdk_thread'
        )

        print('ArmSDKPublisher initialized.')
        print('All arm joints are locked in the initial position.')

    def publish_arm_sdk_cmd(self):
        for i in range(NUM_MOTOR):
            self.arm_sdk_cmd.motor_cmd[i].q = self.q[i]
            self.arm_sdk_cmd.motor_cmd[i].dq = self.dq[i]
            self.arm_sdk_cmd.motor_cmd[i].tau = self.tau[i]
            self.arm_sdk_cmd.motor_cmd[i].kp = self.kp[i]
            self.arm_sdk_cmd.motor_cmd[i].kd = self.kd[i]
        # set CRC
        self.arm_sdk_cmd.crc = self.crc.Crc(self.arm_sdk_cmd)
        # write to publisher
        self.arm_cmd_publisher.Write(self.arm_sdk_cmd)

    def enable_motor(self, motor_ids, init_q):
        motor_ids, init_q = np.array(motor_ids), np.array(init_q)
        assert len(motor_ids) == len(init_q), 'Motor IDs and initial positions must have the same length.'
        assert np.all(motor_ids < NUM_MOTOR) and np.all(motor_ids >= 0), f'Motor IDs must be within [0, {NUM_MOTOR}).'

        # enable arm sdk
        self.arm_sdk_cmd.motor_cmd[INDEX_NOT_USED].q = 1.0
        self.q[motor_ids] = init_q

    def start_publisher(self):
        self.arm_sdk_thread.Start()

    def estop(self):
        self.dq.fill(0.0)
        self.tau.fill(0.0)
        self.kp.fill(0.0)
        self.kd.fill(8.0)
        self.publish_arm_sdk_cmd()

    def shutdown(self):
        self.estop()
        self.arm_sdk_thread.Wait(0)
        self.arm_cmd_publisher.Close()
        print('ArmSDKPublisher shutdown')
