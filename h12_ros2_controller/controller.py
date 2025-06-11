import time
import numpy as np
import tkinter as tk

import pink
import qpsolvers
import pinocchio as pin

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.utils.thread import RecurrentThread

from h12_ros2_controller.utility.joint_definition import ENABLED_JOINTS
from h12_ros2_controller.robot_model import RobotModel
from h12_ros2_controller.channel_interface import CommandPublisher

class ArmController:
    def __init__(self, filename, dt=0.01, vlim=1.0, visualize=False):
        # initialize robot model
        self.robot_model = RobotModel(filename)
        self.dt = dt
        self.vlim = vlim
        self.visualize = visualize

        # initialize channel
        ChannelFactoryInitialize(id=0)

        # initialize subscriber in robot model
        self.robot_model.init_subscriber()
        time.sleep(0.5)
        self.robot_model.sync_subscriber()
        self.robot_model.update_kinematics()

        # define enabled ids and frozen ids
        motor_ids = np.array([i for i in range(13, 27)])
        self.robot_model.init_reduced_model(ENABLED_JOINTS)

        # initialize command publisher for upper body motors
        self.command_publisher = CommandPublisher()
        # gain for arms
        self.command_publisher.kp[13:16] = 180.0
        self.command_publisher.kd[13:16] = 3.0
        self.command_publisher.kp[20:23] = 180.0
        self.command_publisher.kd[20:23] = 3.0
        # gain for elbow
        self.command_publisher.kp[16:18] = 150.0
        self.command_publisher.kd[16:18] = 3.0
        self.command_publisher.kp[23:25] = 150.0
        self.command_publisher.kd[23:25] = 3.0
        # gain for wrist
        self.command_publisher.kp[18:20] = 50.0
        self.command_publisher.kd[18:20] = 2.0
        self.command_publisher.kp[25:27] = 50.0
        self.command_publisher.kd[25:27] = 2.0
        init_q = self.robot_model.q_reduced
        self.command_publisher.enable_motor(motor_ids, init_q)
        self.command_publisher.start_publisher()

        # initialize IK tasks
        # left arm end effector task
        self.left_ee_name = 'left_wrist_yaw_link'
        self.left_ee_task = pink.FrameTask(
            self.left_ee_name,
            position_cost=50.0,
            orientation_cost=30.0,
            lm_damping=1.0
        )
        # right arm end effector task
        self.right_ee_name = 'right_wrist_yaw_link'
        self.right_ee_task = pink.FrameTask(
            self.right_ee_name,
            position_cost=50.0,
            orientation_cost=30.0,
            lm_damping=1.0
        )
        # posture task as regularization
        self.posture_task = pink.PostureTask(
            cost=1e-3
        )
        # variable for easier access
        self._left_arm_action = np.zeros(7)
        self._right_arm_action = np.zeros(7)

        # joint level task for joint-level control (goto_configuration)
        self.joint_task = pink.PostureTask(
            cost=30.0
        )

        # root_path = os.path.dirname(os.path.join(os.path.dirname(__file__), ".."))
        # self.sphere_model, _, self.collision_model = pin.buildModelsFromUrdf('assets/h1_2/h1_2_sphere.urdf')
        # self.collision_data = pink.utils.process_collision_pairs(
        #     self.sphere_model,
        #     self.collision_model,
        #     f'{root_path}/assets/h1_2/h1_2_sphere_collision.srdf',
        # )
        # self.collision_model = self.robot_model.collision_model
        # self.collision_data = pink.utils.process_collision_pairs(
        #     self.robot_model.model,
        #     self.robot_model.collision_model,
        #     f'{root_path}/assets/h1_2/h1_2_collision.srdf',
        # )

        # configuration trakcing robot states
        self.configuration = pink.Configuration(
            self.robot_model.model,
            self.robot_model.data,
            self.robot_model.zero_q,
            # collision_model=self.collision_model,
            # collision_data=self.collision_data
        )
        self.reduced_configuration = pink.Configuration(
            self.robot_model.reduced_model,
            self.robot_model.reduced_data,
            self.robot_model.zero_q_reduced,
        )

        # # collision barriers
        # self.collision_barrier = pink.barriers.SelfCollisionBarrier(
        #     n_collision_pairs=len(self.collision_model.collisionPairs),
        #     gain=20.0,
        #     safe_displacement_gain=1.0,
        #     d_min=0.05,
        # )

        # # spherical collision barriers
        # self.ee_barrier = pink.barriers.BodySphericalBarrier(
        #     ('left_wrist_yaw_link', 'right_wrist_yaw_link'),
        #     d_min=0.2,
        #     gain = 100.0,
        #     safe_displacement_gain=1.0
        # )

        # set initial target for all tasks
        self.tasks = [self.left_ee_task, self.right_ee_task, self.posture_task]
        for task in self.tasks:
            task.set_target_from_configuration(self.configuration)
        # set collision barrier
        self.barriers = []
        # self.barriers = [self.ee_barrier]
        # self.barriers = [self.collision_barrier]
        # select solver
        self.solver = qpsolvers.available_solvers[0]
        if 'osqp' in qpsolvers.available_solvers:
            self.solver = 'osqp'

        if self.visualize:
            self.robot_model.init_visualizer()

    '''
    joint position for left and right arms
    '''
    @property
    def left_arm_q(self):
        return np.copy(self.robot_model.q[13:20])

    @property
    def right_arm_q(self):
        return np.copy(self.robot_model.q[32:39])

    '''
    joint action for left and right arms
    '''
    @property
    def left_arm_action(self):
        return np.copy(self._left_arm_action)

    @property
    def right_arm_action(self):
        return np.copy(self._right_arm_action)

    '''
    left end effector properties
    left_ee_transformation: transformation matrix of the left end effector
    left_ee_position: position of the left end effector
    left_ee_rotation: rotation matrix of the left end effector
    left_ee_rpy: roll, pitch, yaw of the left end effector
    left_ee_pose: pose of the left end effector (x, y, z, roll, pitch, yaw)
    '''
    @property
    def left_ee_transformation(self):
        return self.robot_model.get_frame_transformation(self.left_ee_name)

    @property
    def left_ee_position(self):
        return self.robot_model.get_frame_position(self.left_ee_name)

    @property
    def left_ee_rotation(self):
        return self.robot_model.get_frame_rotation(self.left_ee_name)

    @property
    def left_ee_rpy(self):
        return pin.rpy.matrixToRpy(self.left_ee_rotation)

    @property
    def left_ee_pose(self):
        return np.concatenate([self.left_ee_position,
                               self.left_ee_rpy])

    '''
    left end effector properties
    left_ee_transformation: transformation matrix of the left end effector
    left_ee_position: position of the left end effector
    left_ee_rotation: rotation matrix of the left end effector
    left_ee_rpy: roll, pitch, yaw of the left end effector
    left_ee_pose: pose of the left end effector (x, y, z, roll, pitch, yaw)
    '''
    @property
    def right_ee_transformation(self):
        return self.robot_model.get_frame_transformation(self.right_ee_name)

    @property
    def right_ee_position(self):
        return self.robot_model.get_frame_position(self.right_ee_name)

    @property
    def right_ee_rotation(self):
        return self.robot_model.get_frame_rotation(self.right_ee_name)

    @property
    def right_ee_rpy(self):
        return pin.rpy.matrixToRpy(self.right_ee_rotation)

    @property
    def right_ee_pose(self):
        return np.concatenate([self.right_ee_position,
                               self.right_ee_rpy])

    '''
    left end effector target properties
    left_ee_target_transformation: transformation matrix of the left end effector target
    left_ee_target_position: position of the left end effector target
    left_ee_target_rotation: rotation matrix of the left end effector target
    left_ee_target_rpy: roll, pitch, yaw of the left end effector target
    left_ee_target_pose: pose of the left end effector target (x, y, z, roll, pitch, yaw)
    left_ee_error: error between the current left end effector and the target
    '''
    @property
    def left_ee_target_transformation(self):
        return self.left_ee_task.transform_target_to_world.np

    @left_ee_target_transformation.setter
    def left_ee_target_transformation(self, transformation):
        assert(transformation.shape == (4, 4)), 'Transformation should be a 4x4 matrix.'
        self.left_ee_task.transform_target_to_world = pin.SE3(transformation)

    @property
    def left_ee_target_position(self):
        return self.left_ee_task.transform_target_to_world.translation

    @left_ee_target_position.setter
    def left_ee_target_position(self, position):
        assert(len(position) == 3), 'Position should be a list of 3 elements (x, y, z).'
        self.left_ee_task.transform_target_to_world.translation = np.array(position)

    @property
    def left_ee_target_rotation(self):
        return self.left_ee_task.transform_target_to_world.rotation

    @left_ee_target_rotation.setter
    def left_ee_target_rotation(self, rotation):
        assert(rotation.shape == (3, 3)), 'Rotation should be a 3x3 matrix.'
        self.left_ee_task.transform_target_to_world.rotation = rotation

    @property
    def left_ee_target_rpy(self):
        return pin.rpy.matrixToRpy(self.left_ee_target_rotation)

    @left_ee_target_rpy.setter
    def left_ee_target_rpy(self, rpy):
        assert(len(rpy) == 3), 'Rpy should be a list of 3 elements (roll, pitch, yaw).'
        self.left_ee_target_rotation = pin.rpy.rpyToMatrix(np.array(rpy))

    @property
    def left_ee_target_pose(self):
        return np.concatenate([self.left_ee_target_position,
                               self.left_ee_target_rpy])

    @left_ee_target_pose.setter
    def left_ee_target_pose(self, pose):
        assert(len(pose) == 6), 'Pose should be a list of 6 elements (x, y, z, roll, pitch, yaw).'
        self.left_ee_target_position = pose[:3]
        self.left_ee_target_rpy = pose[3:]

    @property
    def left_ee_error(self):
        return self.left_ee_task.compute_error(self.configuration)

    '''
    right end effector target properties
    right_ee_target_transformation: transformation matrix of the right end effector target
    right_ee_target_position: position of the right end effector target
    right_ee_target_rotation: rotation matrix of the right end effector target
    right_ee_target_rpy: roll, pitch, yaw of the right end effector target
    right_ee_target_pose: pose of the right end effector target (x, y, z, roll, pitch, yaw)
    right_ee_error: error between the current right end effector and the target
    '''
    @property
    def right_ee_target_transformation(self):
        return self.right_ee_task.transform_target_to_world.np

    @right_ee_target_transformation.setter
    def right_ee_target_transformation(self, transformation):
        assert(transformation.shape == (4, 4)), 'Transformation should be a 4x4 matrix.'
        self.right_ee_task.transform_target_to_world = pin.SE3(transformation)

    @property
    def right_ee_target_position(self):
        return self.right_ee_task.transform_target_to_world.translation

    @right_ee_target_position.setter
    def right_ee_target_position(self, position):
        assert(len(position) == 3), 'Position should be a list of 3 elements (x, y, z).'
        self.right_ee_task.transform_target_to_world.translation = np.array(position)

    @property
    def right_ee_target_rotation(self):
        return self.right_ee_task.transform_target_to_world.rotation

    @right_ee_target_rotation.setter
    def right_ee_target_rotation(self, rotation):
        assert(rotation.shape == (3, 3)), 'Rotation should be a 3x3 matrix.'
        self.right_ee_task.transform_target_to_world.rotation = rotation

    @property
    def right_ee_target_rpy(self):
        return pin.rpy.matrixToRpy(self.right_ee_target_rotation)

    @right_ee_target_rpy.setter
    def right_ee_target_rpy(self, rpy):
        assert(len(rpy) == 3), 'Rpy should be a list of 3 elements (roll, pitch, yaw).'
        self.right_ee_target_rotation = pin.rpy.rpyToMatrix(np.array(rpy))

    @property
    def right_ee_target_pose(self):
        return np.concatenate([self.right_ee_target_position,
                               self.right_ee_target_rpy])

    @right_ee_target_pose.setter
    def right_ee_target_pose(self, pose):
        assert(len(pose) == 6), 'Pose should be a list of 6 elements (x, y, z, roll, pitch, yaw).'
        self.right_ee_target_position = pose[:3]
        self.right_ee_target_rpy = pose[3:]

    @property
    def right_ee_error(self):
        return self.right_ee_task.compute_error(self.configuration)

    def limit_joint_vel(self, vel):
        # compute end effector velocity
        v_left = self.robot_model.compute_frame_twist(self.left_ee_name, vel)[0:3]
        v_right = self.robot_model.compute_frame_twist(self.right_ee_name, vel)[0:3]
        # limit end effector velocity
        scaler = np.min([0.3,
                         self.vlim / (np.linalg.norm(v_left) + 1e-3),
                         self.vlim / (np.linalg.norm(v_right) + 1e-3)])

        return scaler * vel

    def apply_joint_vel(self, vel):
        # solve dynamics
        tau = pin.rnea(self.robot_model.model,
                       self.robot_model.data,
                       self.robot_model.q + vel * self.dt,
                       self.robot_model.dq,
                       np.zeros(self.robot_model.model.nv))

        # update joint action
        self._left_arm_action = vel[13:20] * self.dt
        self._right_arm_action = vel[32:39] * self.dt

        # send the velocity command to the robot
        self.command_publisher.q = (self.robot_model.q + vel * self.dt)[self.robot_model.body_q_ids]
        self.command_publisher.dq = vel[self.robot_model.body_q_ids]
        self.command_publisher.tau = tau[self.robot_model.body_q_ids]

    def lock_configuration(self, q):
        # sync robot model and compute forward kinematics
        self.robot_model.sync_subscriber()
        self.robot_model.update_kinematics()
        # update visualizer if needed
        if self.visualize:
            self.robot_model.update_visualizer()
            self.robot_model.visualize_wrench(self.left_ee_name)

        # update configuration
        self.configuration.update(self.robot_model.q)
        self.reduced_configuration.update(self.robot_model.q_reduced)

        # compute tau and enforce same q
        tau = pin.rnea(self.robot_model.model,
                       self.robot_model.data,
                       q,
                       np.zeros(self.robot_model.model.nv),
                       np.zeros(self.robot_model.model.nv))

        # send command to lock the robot in current configuration
        self.command_publisher.q = q
        self.command_publisher.dq = np.zeros(self.robot_model.model.nv)
        self.command_publisher.tau = tau

    def goto_configuration(self, q):
        # sync robot model and compute forward kinematics
        self.robot_model.sync_subscriber()
        self.robot_model.update_kinematics()
        # update visualizer if needed
        if self.visualize:
            self.robot_model.update_visualizer()
            self.robot_model.visualize_wrench(self.left_ee_name)

        # update configuration
        self.configuration.update(self.robot_model.q)
        self.reduced_configuration.update(self.robot_model.q_reduced)

        # use the joint task to solve joint velocity update
        self.joint_task.set_target(q)
        vel = pink.solve_ik(
            self.configuration,
            [self.joint_task],
            dt=self.dt,
            solver=self.solver,
            barriers=self.barriers,
            safety_break=False
        )

        # apply the control
        vel = self.limit_joint_vel(vel)
        self.apply_joint_vel(vel)

    def solve_ik(self):
        # update posture task
        self.posture_task.set_target_from_configuration(self.configuration)

        # solve IK
        vel = pink.solve_ik(
            self.configuration,
            self.tasks,
            dt=self.dt,
            solver=self.solver,
            barriers=self.barriers,
            safety_break=False
        )

        vel = self.limit_joint_vel(vel)

        return vel

    def solve_reduced_ik(self):
        # update posture task
        self.posture_task.set_target_from_configuration(self.reduced_configuration)

        # solve IK
        vel = pink.solve_ik(
            self.reduced_configuration,
            self.tasks,
            dt=self.dt,
            solver=self.solver,
            barriers=self.barriers,
            safety_break=False
        )

        vel_full = np.zeros(self.robot_model.model.nv)
        vel_full[self.robot_model.reduced_mask] = vel
        vel_full = self.limit_joint_vel(vel_full)

        return vel_full

    def control_full_body_step(self):
        # t = time.time()
        # sync robot model and compute forward kinematics
        self.robot_model.sync_subscriber()
        self.robot_model.update_kinematics()
        # update visualizer if needed
        if self.visualize:
            self.robot_model.update_visualizer()
            self.robot_model.visualize_wrench(self.left_ee_name)

        # update configuration
        self.configuration.update(self.robot_model.q)
        self.reduced_configuration.update(self.robot_model.q_reduced)

        # solve IK and apply the control
        vel = self.solve_ik()
        self.apply_joint_vel(vel)

        # print(f'Time: {time.time() - t:.4f}s')

    def control_dual_arm_step(self):
        # t = time.time()
        # sync robot model and compute forward kinematics
        self.robot_model.sync_subscriber()
        self.robot_model.update_kinematics()
        # update visualizer if needed
        if self.visualize:
            self.robot_model.update_visualizer()
            self.robot_model.visualize_wrench(self.left_ee_name)

        # update configuration
        self.configuration.update(self.robot_model.q)
        self.reduced_configuration.update(self.robot_model.q_reduced)

        # solve IK and apply the control
        vel = self.solve_reduced_ik()
        self.apply_joint_vel(vel)

        # print(f'Time: {time.time() - t:.4f}s')

    def sim_full_body_step(self):
        # t = time.time()
        # update visualizer if needed
        if self.visualize:
            self.robot_model.update_visualizer()
            self.robot_model.visualize_wrench(self.left_ee_name)

        # update configuration
        self.configuration.update(self.robot_model.q)
        self.reduced_configuration.update(self.robot_model.q_reduced)

        # solve IK and apply the control
        vel = self.solve_ik()
        self.robot_model._q = self.robot_model.q + vel * self.dt
        self.robot_model.update_kinematics()

        # print(f'Time: {time.time() - t:.4f}s')

    def sim_dual_arm_step(self):
        # t = time.time()
        # update visualizer if needed
        if self.visualize:
            self.robot_model.update_visualizer()
            self.robot_model.visualize_wrench(self.left_ee_name)

        # update configuration
        self.configuration.update(self.robot_model.q)
        self.reduced_configuration.update(self.robot_model.q_reduced)

        # solve IK and apply the control
        vel = self.solve_reduced_ik()
        self.robot_model._q = self.robot_model.q + vel * self.dt
        self.robot_model.update_kinematics()

        # print(f'Time: {time.time() - t:.4f}s')

    def estop(self):
        self.command_publisher.estop()

if __name__ == '__main__':
    # example usage
    arm_controller = ArmController('assets/h1_2/h1_2.urdf',
                                   dt=0.01,
                                   vlim=1.0,
                                   visualize=True)

    root = tk.Tk()
    root.title('Arm Controller')
    root.geometry('600x400')

    # pack sliders side by side
    left_frame = tk.Frame(root)
    right_frame = tk.Frame(root)  # Commented out for now
    left_frame.pack(side=tk.LEFT, padx=10, pady=10)
    right_frame.pack(side=tk.RIGHT, padx=10, pady=10)  # Commented out for now

    # left hand sliders
    slider_lx = tk.Scale(left_frame, label="Left X",
                         from_=-1.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_ly = tk.Scale(left_frame, label="Left Y",
                         from_=-1.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_lz = tk.Scale(left_frame, label="Left Z",
                         from_=-1.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_lr = tk.Scale(left_frame, label="Left Roll",
                         from_=-np.pi, to=np.pi, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_lp = tk.Scale(left_frame, label="Left Pitch",
                         from_=-np.pi, to=np.pi, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_lyaw = tk.Scale(left_frame, label="Left Yaw",
                           from_=-np.pi, to=np.pi, resolution=0.01,
                           orient=tk.HORIZONTAL, length=250)
    slider_lx.pack(in_=left_frame, pady=5)
    slider_ly.pack(in_=left_frame, pady=5)
    slider_lz.pack(in_=left_frame, pady=5)
    slider_lr.pack(in_=left_frame, pady=5)
    slider_lp.pack(in_=left_frame, pady=5)
    slider_lyaw.pack(in_=left_frame, pady=5)

    # right hand sliders
    slider_rx = tk.Scale(right_frame, label="Right X",
                         from_=-1.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_ry = tk.Scale(right_frame, label="Right Y",
                         from_=-1.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_rz = tk.Scale(right_frame, label="Right Z",
                         from_=-1.0, to=1.0, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_rr = tk.Scale(right_frame, label="Right Roll",
                         from_=-np.pi, to=np.pi, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_rp = tk.Scale(right_frame, label="Right Pitch",
                         from_=-np.pi, to=np.pi, resolution=0.01,
                         orient=tk.HORIZONTAL, length=250)
    slider_ryaw = tk.Scale(right_frame, label="Right Yaw",
                           from_=-np.pi, to=np.pi, resolution=0.01,
                           orient=tk.HORIZONTAL, length=250)
    slider_rx.pack(in_=right_frame, pady=5)
    slider_ry.pack(in_=right_frame, pady=5)
    slider_rz.pack(in_=right_frame, pady=5)
    slider_rr.pack(in_=right_frame, pady=5)
    slider_rp.pack(in_=right_frame, pady=5)
    slider_ryaw.pack(in_=right_frame, pady=5)

    # left hand target initialization
    left_ee_position = arm_controller.left_ee_target_pose[:3]
    slider_lx.set(left_ee_position[0])
    slider_ly.set(left_ee_position[1])
    slider_lz.set(left_ee_position[2])
    left_ee_rpy = arm_controller.left_ee_target_rpy
    slider_lr.set(left_ee_rpy[0])
    slider_lp.set(left_ee_rpy[1])
    slider_lyaw.set(left_ee_rpy[2])

    # Right hand target initialization
    right_ee_position = arm_controller.right_ee_target_pose[:3]
    slider_rx.set(right_ee_position[0])
    slider_ry.set(right_ee_position[1])
    slider_rz.set(right_ee_position[2])
    right_ee_rpy = arm_controller.right_ee_target_rpy
    slider_rr.set(right_ee_rpy[0])
    slider_rp.set(right_ee_rpy[1])
    slider_ryaw.set(right_ee_rpy[2])

    root.update()

    while True:
        root.update()
        # update left hand target
        lx = slider_lx.get()
        ly = slider_ly.get()
        lz = slider_lz.get()
        lr = slider_lr.get()
        lp = slider_lp.get()
        lyaw = slider_lyaw.get()
        arm_controller.left_ee_target_pose = [lx, ly, lz, lr, lp, lyaw]

        # update right hand target
        rx = slider_rx.get()
        ry = slider_ry.get()
        rz = slider_rz.get()
        rr = slider_rr.get()
        rp = slider_rp.get()
        ryaw = slider_ryaw.get()
        arm_controller.right_ee_target_pose = [rx, ry, rz, rr, rp, ryaw]

        # arm_controller.control_dual_arm_step()
        arm_controller.sim_dual_arm_step()
        time.sleep(arm_controller.dt)
