import os
import time
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

import meshcat
import meshcat_shapes
import meshcat.geometry as geo
import meshcat.transformations as tf

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from h12_ros2_controller.core.channel_interface import StateSubscriber
from h12_ros2_controller.utility.joint_definition import ALL_JOINTS, BODY_JOINTS

class RobotModel:
    def __init__(self, filename: str):
        # break file name
        ext = os.path.splitext(filename)[1]
        dirs = os.path.dirname(filename)
        # check file extension, load the model
        if ext == '.xml':
            self.model, self.collision_model, self.visual_model = pin.buildModelsFromMJCF(
                filename=filename
            )
        elif filename.endswith('.urdf'):
            self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
                filename=filename,
                package_dirs=dirs
            )
        else:
            raise ValueError('Unsupported file format. Please provide a .xml or .urdf file.')
        # intiialize data for the model
        self.data = self.model.createData()

        # field variabels tracking joint states
        self._q = np.zeros(self.model.nq)
        self._dq = np.zeros(self.model.nv)
        self._tau = np.zeros(self.model.nv)

        # initialize with zero joint positions
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        # placeholder mask for reduced model
        self.reduced_mask = np.ones(self.model.nq, dtype=bool)
        # create a map of joint names, joint ids, and q ids
        self.joint_ids = {}
        self.joint_q_ids = {}
        for joint_id in range(self.model.njoints):
            joint_name = self.model.names[joint_id]
            self.joint_ids[joint_name] = joint_id
            self.joint_q_ids[joint_name] = self.model.joints[joint_id].idx_q
        # create mask for main body joints
        self.body_q_ids = [self.joint_q_ids[joint_name] for joint_name in BODY_JOINTS]

    def init_reduced_model(self, enabled_joints):
        frozen_joints = set(ALL_JOINTS) - set(enabled_joints)
        frozen_ids = [self.joint_ids[joint_name] for joint_name in frozen_joints]
        frozen_q_ids = [self.joint_q_ids[joint_name] for joint_name in frozen_joints]
        # create a reduced model
        self.model_reduced, self.collision_model_reduced = pin.buildReducedModel(
            self.model,
            self.collision_model,
            frozen_ids,
            self.zero_q
        )
        self.data_reduced = self.model_reduced.createData()
        # set the reduced mask
        self.reduced_mask[frozen_q_ids] = False
        # update the reduced q ids
        self.reduced_q_ids = [self.joint_q_ids[joint_name] for joint_name in enabled_joints]
        self.frozen_ids = frozen_ids

    @property
    def q(self):
        return np.copy(self._q)

    @property
    def dq(self):
        return np.copy(self._dq)

    @property
    def tau(self):
        return np.copy(self._tau)

    @property
    def zero_q(self):
        return np.zeros(self.model.nq)

    @property
    def q_reduced(self):
        return np.copy(self._q[self.reduced_mask])

    @property
    def dq_reduced(self):
        return np.copy(self._dq[self.reduced_mask])

    @property
    def tau_reduced(self):
        return np.copy(self._tau[self.reduced_mask])

    @property
    def zero_q_reduced(self):
        return np.copy(self.zero_q[self.reduced_mask])

    def init_visualizer(self):
        try:
            self.viz = MeshcatVisualizer(self.model, self.collision_model, self.visual_model,
                                         copy_models=False, data=self.data)
            self.viz.initViewer(open=True)
            self.viz.loadViewerModel('unitree_h1_2')

            # show lidar frame
            meshcat_shapes.frame(self.viz.viewer['lidar_frame'], opacity=1.0)
            self.viz.viewer['lidar_frame'].set_transform(
                self.get_frame_transformation('lidar_link')
            )

            # show head camera frame
            meshcat_shapes.frame(self.viz.viewer['head_camera_frame'], opacity=1.0)
            self.viz.viewer['head_camera_frame'].set_transform(
                self.get_frame_transformation('head_camera_link')
            )

        except ImportError as err:
            print('ImportError: MeshcatVisualizer requires the meshcat package.')
            print(err)
            exit(0)

    def visualize_wrench(self, link_name):
        # get frame position and wrench
        origin = self.get_frame_position(link_name)
        wrench = self.get_frame_wrench(link_name)

        # create cyclinder to represent force
        force = wrench[0:3]
        force_magnitude = np.linalg.norm(force) + 1e-6
        force_direction = force / force_magnitude
        # print(f'force: {force}, force_magnitude: {force_magnitude}')
        # scale down for visualization
        force_magnitude *= 0.1
        force_transform = self._get_arrow_transformation(origin, force_direction, force_magnitude)
        # add to viewer
        self.viz.viewer[f'{link_name}/force_arrow'].set_object(
            geo.Cylinder(height=force_magnitude, radius=0.01)
        )
        self.viz.viewer[f'{link_name}/force_arrow'].set_transform(force_transform)
        self.viz.viewer[f'{link_name}/force_arrow'].set_property('color', (1.0, 0.0, 0.0, 0.8))

        # create cyclinder to represent torque
        torque = wrench[3:6]
        torque_magnitude = np.linalg.norm(torque) + 1e-6
        torque_direction = torque / torque_magnitude
        # scale down for visualization
        torque_magnitude *= 0.1
        torque_transform = self._get_arrow_transformation(origin, torque_direction, torque_magnitude)
        # add to viewer
        self.viz.viewer[f'{link_name}/torque_arrow'].set_object(
            geo.Cylinder(height=torque_magnitude, radius=0.01)
        )
        self.viz.viewer[f'{link_name}/torque_arrow'].set_transform(torque_transform)
        self.viz.viewer[f'{link_name}/torque_arrow'].set_property('color', (0.0, 0.0, 1.0, 0.8))

    def _get_arrow_transformation(self, origin, direction, magnitude):
        # create rotation matrix to align Y-axis with the direction vector
        y_axis = np.array([0, 1, 0])
        rotation_axis = np.cross(y_axis, direction)
        if np.linalg.norm(rotation_axis) < 1e-6:
            rotation_matrix = np.eye(3) if np.dot(y_axis, direction) > 0 else -np.eye(3)
        else:
            rotation_axis /= np.linalg.norm(rotation_axis)
            angle = np.arccos(np.clip(np.dot(y_axis, direction), -1.0, 1.0))
            rotation_matrix = tf.rotation_matrix(angle, rotation_axis)[:3, :3]

        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = origin + rotation_matrix @ np.array([0, 1, 0]) * magnitude / 2

        return transform

    def init_subscriber(self):
        self.state_subscriber = StateSubscriber()

    def sync_subscriber(self):
        # update the q, dq, tau
        self._q[self.body_q_ids] = self.state_subscriber.q
        self._dq[self.body_q_ids] = self.state_subscriber.dq
        self._tau[self.body_q_ids] = self.state_subscriber.tau

    def update_kinematics(self):
        # udpate data with the current joint positions
        pin.forwardKinematics(self.model, self.data, self.q, self.dq)
        pin.updateFramePlacements(self.model, self.data)

    def update_visualizer(self):
        self.viz.display()

    def get_frame_transformation(self, frame_name: str):
        frame_id = self.model.getFrameId(frame_name)
        transformation = self.data.oMf[frame_id]
        return transformation.np

    def get_frame_position(self, frame_name: str):
        frame_id = self.model.getFrameId(frame_name)
        transformation = self.data.oMf[frame_id]
        return transformation.translation

    def get_frame_rotation(self, frame_name: str):
        frame_id = self.model.getFrameId(frame_name)
        transformation = self.data.oMf[frame_id]
        return transformation.rotation

    def get_frame_jacobian(self, frame_name: str):
        '''
        Get the frame jacobian in the world frame
        '''
        frame_id = self.model.getFrameId(frame_name)
        jacobian = pin.computeFrameJacobian(
            self.model,
            self.data,
            self.q,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        return jacobian

    def get_joint_jacobian(self, joint_name: str):
        '''
        Get the joint jacobian in the local frame of the joint
        '''
        joint_id = self.model.getJointId(joint_name)
        jacobian = pin.computeJointJacobian(
            self.model,
            self.data,
            self.q,
            joint_id
        )
        return jacobian

    def get_frame_twist(self, frame_name: str):
        frame_id = self.model.getFrameId(frame_name)
        twist = pin.getFrameVelocity(
            self.model,
            self.data,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        return np.concatenate([twist.linear, twist.angular])

    def compute_frame_twist(self, frame_name: str, dq: np.ndarray):
        jac = self.get_frame_jacobian(frame_name)
        twist = jac @ dq
        return twist

    def get_frame_wrench(self, frame_name: str):
        jac = self.get_frame_jacobian(frame_name)
        tau_gravity = pin.rnea(self.model,
                               self.data,
                               self.q,
                               np.zeros(self.model.nv),
                               np.zeros(self.model.nv))
        wrench = np.linalg.inv(jac @ jac.T) @ jac @ (self.tau - tau_gravity)
        return wrench
