import os
import time
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

import meshcat
import meshcat_shapes
import meshcat.geometry as geo
import meshcat.transformations as tf

from h12_ros2_controller.core.channel_interface import StateSubscriber
from h12_ros2_controller.utility.joint_definition import ALL_JOINTS, BODY_JOINTS

class RobotModel:
    def __init__(self, filename: str):
        assert(os.path.splitext(filename)[1] == '.urdf'), 'Please provide a urdf file for the robot model.'
        self.model, _, self.visual_model = self._load_urdf(filename)
        # initialize data for the model
        self.data = self.model.createData()

        # placeholder for visualizer and state subscriber
        self.viz = None
        self.state_subscriber = None

        # field variabels tracking joint states
        self._q = np.zeros(self.model.nq)
        self._dq = np.zeros(self.model.nv)
        self._tau = np.zeros(self.model.nv)

        # initialize with zero joint positions
        pin.forwardKinematics(self.model, self.data, self.state['q'])
        pin.updateFramePlacements(self.model, self.data)

        # flags for additional initialization
        self.init_reduced = False
        self.init_collision = False
        # placeholder mask for reduced model
        self.reduced_mask = np.ones(self.model.nq, dtype=bool)

    def init_reduced_model(self, enabled_joints):
        self.init_reduced = True
        frozen_joints = set(BODY_JOINTS) - set(enabled_joints)
        frozen_ids = [self.model.getJointId(joint_name) for joint_name in frozen_joints]
        frozen_q_ids = [self.model.joints[joint_id].idx_q for joint_id in frozen_ids]
        # create a reduced model
        self.model_reduced = pin.buildReducedModel(
            self.model, frozen_ids, self.zero_q
        )
        self.data_reduced = self.model_reduced.createData()
        # set the reduced mask
        self.reduced_mask[frozen_q_ids] = False
        # update the reduced q ids
        self.frozen_ids = frozen_ids

    def init_collision_model(self, urdf_path, srdf_path):
        '''Initialize the collision model from another urdf'''
        self.init_collision = True
        model, collision_model, _ = self._load_urdf(urdf_path)
        self.collision_model, self.collision_data = self._process_srdf(
            model, collision_model, srdf_path
        )

        if self.init_reduced:
            model_reduced, collision_model_reduced = pin.buildReducedModel(
                model, collision_model, self.frozen_ids, self.zero_q
            )
            self.collision_model_reduced, self.collision_data_reduced = RobotModel._process_srdf(
                model_reduced, collision_model_reduced, srdf_path
            )

    @staticmethod
    def _load_urdf(urdf_path):
        model, collision_model, visual_model = pin.buildModelsFromUrdf(
            filename=urdf_path,
            package_dirs=os.path.dirname(urdf_path),
        )
        # process to keep only the body joints
        frozen_joints = set(ALL_JOINTS) - set(BODY_JOINTS)
        frozen_ids = [model.getJointId(joint_name) for joint_name in frozen_joints]
        model, [collision_model, visual_model] = pin.buildReducedModel(
            model, [collision_model, visual_model], frozen_ids, np.zeros(model.nq)
        )

        return model, collision_model, visual_model

    @staticmethod
    def _process_srdf(model, collision_model, srdf_path):
        collision_model.addAllCollisionPairs()
        pin.removeCollisionPairs(model, collision_model, srdf_path)
        collision_data = pin.GeometryData(collision_model)
        collision_data.enable_contact = True
        return collision_model, collision_data

    @property
    def state(self):
        # if state_subscriber exists, use its state dict
        if self.state_subscriber is not None:
            return self.state_subscriber.state
        # otherwise, fallback to local fields
        return {
            'q': np.copy(self._q),
            'dq': np.copy(self._dq),
            'tau': np.copy(self._tau),
        }

    @property
    def state_reduced(self):
        mask_keys = {'mode', 'q', 'dq', 'ddq', 'tau', 'vol', 'motor_state', 'temperature', 'sensor'}
        state_reduced = {}
        for k, v in self.state.items():
            if k in mask_keys:
                assert v.shape[0] == self.reduced_mask.size, 'mask length mismatch'
                state_reduced[k] = np.copy(v[self.reduced_mask])
            else:
                state_reduced[k] = v
        return state_reduced

    @property
    def zero_q(self):
        return np.zeros(self.model.nq)

    @property
    def zero_q_reduced(self):
        return np.zeros(self.model_reduced.nq)

    def shutdown(self):
        if self.state_subscriber is not None:
            self.state_subscriber.shutdown()
            self.state_subscriber = None
        print('RobotModel shutdown')

    def init_visualizer(self):
        try:
            self.viz = MeshcatVisualizer(self.model, visual_model=self.visual_model,
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

    def visualize_com(self):
        com_pos = self.get_com()
        transform = np.eye(4)
        transform[:3, 3] = com_pos
        viewer = self.viz.viewer
        viewer['com'].set_object(geo.Sphere(0.02))
        viewer['com'].set_transform(transform)
        viewer['com'].set_property('color', (1.0, 0.5, 0.0, 0.8))

    def visualize_zmp(self):
        zmp_pos = self.get_zmp()
        transform = np.eye(4)
        transform[:3, 3] = zmp_pos
        viewer = self.viz.viewer
        viewer['zmp'].set_object(geo.Sphere(0.02))
        viewer['zmp'].set_transform(transform)
        viewer['zmp'].set_property('color', (0.0, 1.0, 0.0, 0.8))

    def visualize_wrench(self, link_name):
        # get frame position and wrench
        wrench = self.get_frame_wrench(link_name)

        # create cyclinder to represent force
        force = wrench[0:3]
        force_magnitude = np.linalg.norm(force) + 1e-6
        force_direction = force / force_magnitude
        # print(f'force: {force}, force_magnitude: {force_magnitude}')
        # scale down for visualization
        force_magnitude *= 0.1
        force_transform = self._get_arrow_transformation(force_direction, force_magnitude)
        # add to viewer
        viewer = self.viz.viewer
        viewer[f'{link_name}/force_arrow'].set_object(
            geo.Cylinder(height=force_magnitude, radius=0.01)
        )
        viewer[f'{link_name}/force_arrow'].set_transform(force_transform)
        viewer[f'{link_name}/force_arrow'].set_property('color', (1.0, 0.0, 0.0, 0.8))

        # create cyclinder to represent torque
        torque = wrench[3:6]
        torque_magnitude = np.linalg.norm(torque) + 1e-6
        torque_direction = torque / torque_magnitude
        # scale down for visualization
        torque_magnitude *= 0.1
        torque_transform = self._get_arrow_transformation(torque_direction, torque_magnitude)
        # add to viewer
        self.viz.viewer[f'{link_name}/torque_arrow'].set_object(
            geo.Cylinder(height=torque_magnitude, radius=0.01)
        )
        self.viz.viewer[f'{link_name}/torque_arrow'].set_transform(torque_transform)
        self.viz.viewer[f'{link_name}/torque_arrow'].set_property('color', (0.0, 0.0, 1.0, 0.8))

    def _get_arrow_transformation(self, direction, magnitude):
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
        transform[:3, 3] = rotation_matrix @ np.array([0, 1, 0]) * magnitude / 2

        return transform

    def init_subscriber(self):
        self.state_subscriber = StateSubscriber()
        print('StateSubscriber initialized.')

    def update_kinematics(self):
        # udpate data with the current joint positions
        pin.forwardKinematics(self.model, self.data, self.state['q'], self.state['dq'])
        pin.updateFramePlacements(self.model, self.data)
        if self.init_reduced:
            pin.forwardKinematics(self.model_reduced, self.data_reduced, self.state_reduced['q'], self.state_reduced['dq'])
            pin.updateFramePlacements(self.model_reduced, self.data_reduced)

    def update_visualizer(self):
        if self.viz is not None:
            self.viz.display()

    def get_com(self, q: np.ndarray=None):
        q = self.state['q'] if q is None else q
        com = pin.centerOfMass(self.model, self.data, q)
        return com

    def get_com_reduced(self, q_reduced: np.ndarray=None):
        q_reduced = self.state_reduced['q'] if q_reduced is None else q_reduced
        com = pin.centerOfMass(self.model_reduced, self.data_reduced, q_reduced)
        return com

    def get_zmp(self, q: np.ndarray=None):
        q = self.state['q'] if q is None else q
        com = self.get_com(q)

        # centroidal momentum time derivative
        pin.computeCentroidalMomentumTimeVariation(
            self.model, self.data, q, self.state['dq'], self.state['ddq']
        )
        dhg = self.data.dhg

        F_ext = dhg.linear
        tau_ext = dhg.angular

        # gravity force
        g = np.array([0, 0, -9.81])
        m = pin.computeTotalMass(self.model)
        F_g = m * g

        tau_ext_contact = tau_ext - np.cross(com, F_ext + F_g)
        F_ext_contact = F_ext - F_g

        zmp_x = -tau_ext_contact[1] / F_ext_contact[2]
        zmp_y = tau_ext_contact[0] / F_ext_contact[2]
        zmp = np.array([zmp_x, zmp_y, 0.0])

        return zmp

    def get_gravity_compensation(self, q: np.ndarray=None):
        q = self.state['q'] if q is None else q
        tau_gravity = pin.rnea(self.model,
                               self.data,
                               q,
                               np.zeros(self.model.nv),
                               np.zeros(self.model.nv))
        return tau_gravity

    def _get_frame_transformation(self, frame_name, q: np.ndarray=None):
        frame_id = self.model.getFrameId(frame_name)
        if q is not None:
            data_temp = self.model.createData()
            pin.forwardKinematics(self.model, data_temp, q)
            pin.updateFramePlacements(self.model, data_temp)
            transformation = data_temp.oMf[frame_id]
        else:
            self.update_kinematics()
            transformation = self.data.oMf[frame_id]
        return transformation

    def get_frame_transformation(self, frame_name: str, q: np.ndarray=None):
        return self._get_frame_transformation(frame_name, q).np

    def get_frame_position(self, frame_name: str, q: np.ndarray=None):
        return self._get_frame_transformation(frame_name, q).translation

    def get_frame_rotation(self, frame_name: str, q: np.ndarray = None):
        return self._get_frame_transformation(frame_name, q).rotation

    def _get_frame_transformation_reduced(self, frame_name, q_reduced: np.ndarray=None):
        assert(self.init_reduced), 'Reduced model is not initialized.'
        frame_id = self.model_reduced.getFrameId(frame_name)
        if q_reduced is not None:
            data_reduced_temp = self.model_reduced.createData()
            pin.forwardKinematics(self.model_reduced, data_reduced_temp, q_reduced)
            pin.updateFramePlacements(self.model_reduced, data_reduced_temp)
            transformation = data_reduced_temp.oMf[frame_id]
        else:
            self.update_kinematics()
            transformation = self.data_reduced.oMf[frame_id]
        return transformation

    def get_frame_transformation_reduced(self, frame_name: str, q_reduced: np.ndarray=None):
        assert(self.init_reduced), 'Reduced model is not initialized.'
        return self._get_frame_transformation_reduced(frame_name, q_reduced).np

    def get_frame_position_reduced(self, frame_name: str, q_reduced: np.ndarray=None):
        assert(self.init_reduced), 'Reduced model is not initialized.'
        return self._get_frame_transformation_reduced(frame_name, q_reduced).translation

    def get_frame_rotation_reduced(self, frame_name: str, q_reduced: np.ndarray=None):
        assert(self.init_reduced), 'Reduced model is not initialized.'
        return self._get_frame_transformation_reduced(frame_name, q_reduced).rotation

    def get_frame_jacobian(self, frame_name: str, q: np.ndarray=None):
        '''
        Get the frame jacobian in the world frame
        '''
        if q is not None:
            data, q = self.model.createData(), q
        else:
            data, q = self.data, self.state['q']
        # update kinematics
        pin.forwardKinematics(self.model, data, q)
        pin.updateFramePlacements(self.model, data)
        # compute jacobian
        frame_id = self.model.getFrameId(frame_name)
        jacobian = pin.computeFrameJacobian(
            self.model,
            data,
            q,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        return jacobian

    def get_joint_jacobian(self, joint_name: str, q: np.ndarray=None):
        '''
        Get the joint jacobian in the local frame of the joint
        '''
        if q is not None:
            data, q = self.model.createData(), q
        else:
            data, q = self.data, self.state['q']
        # update kinematics
        pin.forwardKinematics(self.model, data, q)
        pin.updateFramePlacements(self.model, data)
        # compute jacobian
        joint_id = self.model.getJointId(joint_name)
        jacobian = pin.computeJointJacobian(
            self.model,
            data,
            q,
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

    def get_frame_wrench(self, frame_name: str, q: np.ndarray=None):
        q = self.state['q'] if q is None else q
        tau_gravity = self.get_gravity_compensation(q)
        jac = self.get_frame_jacobian(frame_name, q)
        wrench = np.linalg.inv(jac @ jac.T) @ jac @ (self.state['tau'] - tau_gravity)
        return wrench

    def compute_frame_twist(self, frame_name: str, dq: np.ndarray):
        jac = self.get_frame_jacobian(frame_name)
        twist = jac @ dq
        return twist

    def check_valid(self, q):
        '''Check if the given joint position is valid'''
        return self.check_within_limits(q) and self.check_collision_free(q)

    def check_within_limits(self, q):
        '''Check if the given joint position violates joint limits'''
        return (
            np.all(q <= self.model.upperPositionLimit) and
            np.all(q >= self.model.lowerPositionLimit)
        )

    def check_within_limits_reduced(self, q_reduced):
        '''Check if the given joint position violates joint limits for the reduced model'''
        assert(self.init_reduced), 'Reduced model is not initialized.'
        return (
            np.all(q_reduced <= self.model_reduced.upperPositionLimit) and
            np.all(q_reduced >= self.model_reduced.lowerPositionLimit)
        )

    def check_collision_free(self, q):
        '''Check if the given joint position violates collision constraints'''
        assert(self.init_collision), 'Collision model is not initialized.'
        return not pin.computeCollisions(
            self.model, self.data,
            self.collision_model, self.collision_data, q,
            stop_at_first_collision=True
        )

    def check_collision_free_reduced(self, q_reduced):
        assert(self.init_reduced), 'Reduced model is not initialized.'
        assert(self.init_collision), 'Collision model is not initialized.'
        return not pin.computeCollisions(
            self.model_reduced, self.data_reduced,
            self.collision_model, self.collision_data, q_reduced,
            stop_at_first_collision=True
        )
