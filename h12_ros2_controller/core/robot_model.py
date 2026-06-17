import os
import numpy as np
import pinocchio as pin
from pyquaternion import Quaternion
from pinocchio.visualize import MeshcatVisualizer

import meshcat
import meshcat_shapes
import meshcat.geometry as geo
import meshcat.transformations as tf

from h12_ros2_controller.core.channel_interface import StateSubscriber
from h12_ros2_controller.utility.joint_definition import (
    ALL_JOINTS, ALL_HANDLESS_JOINTS, BODY_JOINTS, NUM_MOTOR,
    FREEFLYER_NQ, FREEFLYER_NV,
    FREEFLYER_POS, FREEFLYER_QUAT
)

class RobotModel:
    def __init__(self, filename: str, handless: bool=False):
        assert(os.path.splitext(filename)[1] == '.urdf'), \
            'Please provide a urdf file for the robot model.'
        self.handless = handless
        # full model (free-flyer) for visualization and dynamics
        self.model, _, self.visual_model = self._load_urdf_freeflyer(filename, handless=self.handless)
        self.data = self.model.createData()

        # model without free-flyer for IK solver (motor only)
        self.model_body, self.collision_model_body, _ = self._load_urdf_body(filename, handless=self.handless)
        self.data_body = self.model_body.createData()

        # placeholder for visualizer and state subscriber
        self.viz = None
        self.state_subscriber = None

        # field variables tracking joint states (motor only, length NUM_MOTOR)
        self._q = np.zeros(NUM_MOTOR)
        self._dq = np.zeros(NUM_MOTOR)
        self._ddq = np.zeros(NUM_MOTOR)
        self._tau = np.zeros(NUM_MOTOR)

        # initialize with zero joint positions
        pin.forwardKinematics(self.model, self.data, self.zero_q)
        pin.updateFramePlacements(self.model, self.data)
        pin.forwardKinematics(self.model_body, self.data_body, self.zero_q_body)
        pin.updateFramePlacements(self.model_body, self.data_body)

        # flags for additional initialization
        self.init_reduced = False
        self.init_collision = False
        # placeholder mask for reduced model (motor only, length NUM_MOTOR)
        self.reduced_mask = np.ones(NUM_MOTOR, dtype=bool)
        # visualization configuration
        self.viz_config = {}

    def init_reduced_model(self, enabled_joints):
        '''
        create reduced model from model_body (no free-flyer) for IK
        reduced_mask applies to motor-only arrays (length NUM_MOTOR)
        '''
        self.init_reduced = True
        frozen_joints = set(BODY_JOINTS) - set(enabled_joints)
        # get joint ids in model_body (no free-flyer, so no offset)
        frozen_ids_body = [self.model_body.getJointId(joint_name) for joint_name in frozen_joints]
        frozen_motor_ids = [self.model_body.joints[joint_id].idx_q for joint_id in frozen_ids_body]
        # create reduced model from model_body
        self.model_body_reduced = pin.buildReducedModel(
            self.model_body, frozen_ids_body, self.zero_q_body
        )
        self.data_body_reduced = self.model_body_reduced.createData()
        # set the reduced mask (motor only)
        self.reduced_mask[frozen_motor_ids] = False
        # store frozen ids for collision model
        self.frozen_ids_body = frozen_ids_body

    def init_collision_model(self, urdf_path, srdf_path):
        '''
        Initialize collision model from another urdf (sphere collision)
        Uses model_body (no free-flyer) for collision checking
        '''
        self.init_collision = True
        # load collision model without free-flyer
        model_collision, collision_model, _ = self._load_urdf_body(urdf_path, handless=self.handless)
        self.collision_model_body, self.collision_data_body = self._process_srdf(
            model_collision, collision_model, srdf_path
        )

        if self.init_reduced:
            model_collision_reduced, collision_model_reduced = pin.buildReducedModel(
                model_collision, collision_model, self.frozen_ids_body, self.zero_q_body
            )
            self.collision_model_body_reduced, self.collision_data_body_reduced = RobotModel._process_srdf(
                model_collision_reduced, collision_model_reduced, srdf_path
            )

    @staticmethod
    def _load_urdf_freeflyer(urdf_path, handless=False):
        '''
        Load urdf with free-flyer root joint for visualization and dynamics
        Returns full model (free-flyer) with nq=34 (7 free-flyer + 27 motor), nv=33 (6 free-flyer + 27 motor)
        '''
        model, collision_model, visual_model = pin.buildModelsFromUrdf(
            filename=urdf_path,
            package_dirs=os.path.dirname(urdf_path),
            root_joint=pin.JointModelFreeFlyer(),
        )
        all_joints = ALL_HANDLESS_JOINTS if handless else ALL_JOINTS
        # process to keep only the body joints
        frozen_joints = set(all_joints) - set(BODY_JOINTS)
        frozen_ids = [model.getJointId(joint_name) for joint_name in frozen_joints]
        model, [collision_model, visual_model] = pin.buildReducedModel(
            model, [collision_model, visual_model], frozen_ids, np.zeros(model.nq)
        )
        return model, collision_model, visual_model

    @staticmethod
    def _load_urdf_body(urdf_path, handless=False):
        '''
        Load urdf without free-flyer for IK solver
        Returns model with nq=27 (motor only), nv=27 (motor only)
        '''
        model, collision_model, visual_model = pin.buildModelsFromUrdf(
            filename=urdf_path,
            package_dirs=os.path.dirname(urdf_path),
        )
        all_joints = ALL_HANDLESS_JOINTS if handless else ALL_JOINTS
        # process to keep only the body joints
        frozen_joints = set(all_joints) - set(BODY_JOINTS)
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
            'ddq': np.copy(self._ddq),
            'tau': np.copy(self._tau),
        }

    @property
    def state_reduced(self):
        '''
        Apply reduced_mask to motor-only state arrays
        Reduced_mask is length NUM_MOTOR (27)
        '''
        mask_keys = {'mode', 'q', 'dq', 'ddq', 'tau', 'vol', 'motor_state', 'temperature', 'sensor'}
        state_reduced = {}
        for k, v in self.state.items():
            if k in mask_keys:
                assert v.shape[0] == self.reduced_mask.size, 'mask length mismatch'
                state_reduced[k] = np.copy(v[self.reduced_mask])
            else:
                state_reduced[k] = v
        return state_reduced

    def full_q(self, q: np.ndarray, imu_quat=None) -> np.ndarray:
        '''
        convert motor q (27) to full pinocchio q (34) with pelvis orientation
        if imu_quat provided (unitree wxyz), use it; otherwise try state subscriber
        for use with full model with free-flyer base
        '''
        full_q = np.zeros(self.model.nq)
        # get imu quaternion
        if imu_quat is None and self.state_subscriber is not None and 'imu_state' in self.state:
            imu_quat = self.state['imu_state'].quaternion  # unitree wxyz

        if imu_quat is not None:
            imu_quat = np.asarray(imu_quat, dtype=float).reshape(4)
            # Validate IMU quaternion before normalization to avoid propagating NaN/Inf
            if not np.isfinite(imu_quat).all():
                # fallback to identity quaternion (wxyz) if IMU data is invalid
                imu_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
            else:
                imu_norm = np.linalg.norm(imu_quat)
                if (imu_norm < 1e-8) or (not np.isfinite(imu_norm)):
                    imu_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
                else:
                    imu_quat = imu_quat / imu_norm

            torso_quat = Quaternion(imu_quat[0], imu_quat[1], imu_quat[2], imu_quat[3])
            # get pelvis-to-torso transform from model_body
            pin.forwardKinematics(self.model_body, self.data_body, q)
            pin.updateFramePlacements(self.model_body, self.data_body)
            pelvis_to_torso = self.data_body.oMf[self.model_body.getFrameId('torso_link')]
            pelvis_to_torso_quat = Quaternion(matrix=pelvis_to_torso.rotation)
            # pelvis_quat = torso_quat * inv(pelvis_to_torso_quat)
            pelvis_quat = (torso_quat * pelvis_to_torso_quat.inverse).normalised
            # convert to pinocchio xyzw
            full_q[FREEFLYER_QUAT] = [pelvis_quat.x, pelvis_quat.y, pelvis_quat.z, pelvis_quat.w]
        else:
            full_q[FREEFLYER_QUAT] = [0, 0, 0, 1]  # identity quaternion xyzw
        full_q[FREEFLYER_NQ:] = q
        return full_q

    def full_v(self, v: np.ndarray) -> np.ndarray:
        '''
        Convert motor v (27) to full pinocchio v (33) with zero base velocity
        For use with full model (free-flyer)
        '''
        full_v = np.zeros(self.model.nv)
        full_v[FREEFLYER_NV:] = v
        return full_v

    @property
    def zero_q(self):
        '''Zero configuration for full model (free-flyer), length 34'''
        zero = np.zeros(self.model.nq)
        zero[FREEFLYER_QUAT] = [0, 0, 0, 1]  # identity quaternion xyzw
        return zero

    @property
    def zero_q_body(self):
        '''Zero configuration for model_body (no free-flyer), length 27'''
        return np.zeros(self.model_body.nq)

    @property
    def zero_q_body_reduced(self):
        '''Zero configuration for model_body_reduced'''
        return np.zeros(self.model_body_reduced.nq)

    def shutdown(self):
        if self.state_subscriber is not None:
            self.state_subscriber.shutdown()
            self.state_subscriber = None
        print('RobotModel shutdown', flush=True)

    def init_visualizer(self):
        '''Initialize meshcat visualizer'''
        try:
            self.viz = MeshcatVisualizer(self.model, visual_model=self.visual_model,
                                         copy_models=False, data=self.data)
            self.viz.initViewer()
            self.viz.loadViewerModel('unitree_h1_2')

            # store visualization configuration
            self.viz_config = {
                'show_com': False,
                'show_zmp': False,
                'show_sensors': False,
                'wrench_frames': [],
            }

        except ImportError as err:
            print('ImportError: MeshcatVisualizer requires the meshcat package.')
            print(err)
            exit(0)

    def config_visualizer(self,
                          show_com=None,
                          show_zmp=None,
                          show_sensors=None,
                          wrench_frames=None):
        '''
        Configure dynamic visualizations

        @param show_sensors: visualize sensor frames (lidar and head camera)
        @param show_com: visualize center of mass
        @param show_zmp: visualize zero moment point
        @param wrench_frames: list of frame names to visualize wrenches
        '''
        assert self.viz is not None, 'Visualizer must be initialized first.'

        # update config with provided values only
        config_updates = {
            'show_com': show_com,
            'show_zmp': show_zmp,
            'show_sensors': show_sensors,
            'wrench_frames': wrench_frames,
        }
        for key, value in config_updates.items():
            if value is not None:
                self.viz_config[key] = value

        # initialize frame visualizations
        if show_sensors:
            meshcat_shapes.frame(self.viz.viewer['livox_frame'], opacity=1.0)
            meshcat_shapes.frame(self.viz.viewer['camera_frame'], opacity=1.0)

    def _visualize_sensors(self):
        '''Update sensor frame visualizations (lidar and head camera)'''
        viewer = self.viz.viewer
        viewer['livox_frame'].set_transform(
            self.get_frame_transformation('livox_link')
        )
        viewer['camera_frame'].set_transform(
            self.get_frame_transformation('camera_link')
        )

    def _visualize_com(self):
        '''Update center of mass visualization'''
        com_pos = self.get_com()
        transform = np.eye(4)
        transform[:3, 3] = com_pos
        viewer = self.viz.viewer
        viewer['com'].set_object(geo.Sphere(0.02))
        viewer['com'].set_transform(transform)
        viewer['com'].set_property('color', (1.0, 0.5, 0.0, 0.8))

    def _visualize_zmp(self):
        '''Update zero moment point visualization'''
        zmp_pos = self.get_zmp()
        transform = np.eye(4)
        transform[:3, 3] = zmp_pos
        viewer = self.viz.viewer
        viewer['zmp'].set_object(geo.Sphere(0.02))
        viewer['zmp'].set_transform(transform)
        viewer['zmp'].set_property('color', (0.0, 1.0, 0.0, 0.8))

    def _visualize_wrench(self, link_name):
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

    def init_subscriber(self, low_state_topic='rt/lowstate'):
        self.state_subscriber = StateSubscriber(low_state_topic=low_state_topic)
        print('StateSubscriber initialized.', flush=True)

    def update_kinematics(self, imu_quat=None):
        '''update forward kinematics for all models'''
        # update full model (free-flyer) with current joint positions
        q_full = self.full_q(self.state['q'], imu_quat=imu_quat)
        dq_full = self.full_v(self.state['dq'])
        pin.forwardKinematics(self.model, self.data, q_full, dq_full)
        pin.updateFramePlacements(self.model, self.data)
        # update model_body (no free-flyer)
        pin.forwardKinematics(self.model_body, self.data_body, self.state['q'], self.state['dq'])
        pin.updateFramePlacements(self.model_body, self.data_body)
        # update reduced model if initialized
        if self.init_reduced:
            pin.forwardKinematics(self.model_body_reduced, self.data_body_reduced,
                                  self.state_reduced['q'], self.state_reduced['dq'])
            pin.updateFramePlacements(self.model_body_reduced, self.data_body_reduced)

    def update_visualizer(self):
        '''Update robot visualization and all enabled dynamic visualizations'''
        if self.viz is not None:
            self.viz.display()

            # update dynamic visualizations based on config
            if self.viz_config.get('show_sensors', False):
                self._visualize_sensors()
            if self.viz_config.get('show_com', False):
                self._visualize_com()
            if self.viz_config.get('show_zmp', False):
                self._visualize_zmp()
            for frame_name in self.viz_config.get('wrench_frames', []):
                self._visualize_wrench(frame_name)

    def get_com(self, q: np.ndarray=None):
        q = self.state['q'] if q is None else q
        q_full = self.full_q(q)
        com = pin.centerOfMass(self.model, self.data, q_full)
        return com

    def get_com_reduced(self, q_reduced: np.ndarray=None):
        '''Get center of mass for the reduced model (model_body_reduced)'''
        q_reduced = self.state_reduced['q'] if q_reduced is None else q_reduced
        com = pin.centerOfMass(self.model_body_reduced, self.data_body_reduced, q_reduced)
        return com

    def get_zmp(self, q: np.ndarray=None):
        q = self.state['q'] if q is None else q
        com = self.get_com(q)

        # find reference support plane
        left_foot_pos = self.get_frame_position('left_ankle_roll_link', q)
        right_foot_pos = self.get_frame_position('right_ankle_roll_link', q)
        ground_point = 0.5 * (left_foot_pos + right_foot_pos)
        ground_height = ground_point[2]

        # centroidal momentum time derivative
        q_full = self.full_q(q)
        dq_full = self.full_v(self.state['dq'])
        ddq_full = self.full_v(self.state['ddq'])
        pin.computeCentroidalMomentumTimeVariation(
            self.model, self.data, q_full, dq_full, ddq_full
        )
        dhg = self.data.dhg

        F_com = dhg.linear
        tau_com = dhg.angular

        # gravity force
        g = np.array([0, 0, -9.81])
        m = pin.computeTotalMass(self.model)
        F_g = m * g

        # isolate contact resultant force
        F_contact = F_com - F_g
        # shift torque from CoM -> support_plane_point on the plane
        tau_contact = tau_com + np.cross(com - ground_point, F_contact)

        eps = 1e-6
        # TODO gate ZMP when contact is unreliable (low Fz / contact loss)
        if abs(F_contact[2]) < eps:
            return np.zeros(3)

        # ZMP on support plane
        zmp_x = ground_point[0] - tau_contact[1] / F_contact[2]
        zmp_y = ground_point[1] + tau_contact[0] / F_contact[2]
        zmp = np.array([zmp_x, zmp_y, ground_height])

        return zmp

    def get_gravity_compensation(self, q: np.ndarray=None, imu_quat=None):
        q = self.state['q'] if q is None else q
        q_full = self.full_q(q, imu_quat)
        tau_full = pin.rnea(self.model,
                            self.data,
                            q_full,
                            np.zeros(self.model.nv),
                            np.zeros(self.model.nv))
        # return motor-only torques (skip free-flyer)
        return tau_full[FREEFLYER_NV:]

    def _get_frame_transformation(self, frame_name, q: np.ndarray=None):
        frame_id = self.model.getFrameId(frame_name)
        if q is not None:
            q_full = self.full_q(q)
            data_temp = self.model.createData()
            pin.forwardKinematics(self.model, data_temp, q_full)
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
        '''Get frame transformation for the reduced model (model_body_reduced)'''
        assert(self.init_reduced), 'Reduced model is not initialized.'
        frame_id = self.model_body_reduced.getFrameId(frame_name)
        if q_reduced is not None:
            data_body_reduced_temp = self.model_body_reduced.createData()
            pin.forwardKinematics(self.model_body_reduced, data_body_reduced_temp, q_reduced)
            pin.updateFramePlacements(self.model_body_reduced, data_body_reduced_temp)
            transformation = data_body_reduced_temp.oMf[frame_id]
        else:
            self.update_kinematics()
            transformation = self.data_body_reduced.oMf[frame_id]
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

    def get_frame_jacobian(self, frame_name: str, q: np.ndarray=None, imu_quat=None):
        '''
        Get the frame jacobian in the world frame
        q is motor-only (27), returns motor-only jacobian (6 x 27)
        '''
        if q is not None:
            q_full = self.full_q(q, imu_quat)
            data = self.model.createData()
        else:
            q_full = self.full_q(self.state['q'])
            data = self.data
        # update kinematics
        pin.forwardKinematics(self.model, data, q_full)
        pin.updateFramePlacements(self.model, data)
        # compute jacobian
        frame_id = self.model.getFrameId(frame_name)
        jacobian_full = pin.computeFrameJacobian(
            self.model,
            data,
            q_full,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        # return motor-only columns (skip free-flyer columns)
        return jacobian_full[:, FREEFLYER_NV:]

    def get_joint_jacobian(self, joint_name: str, q: np.ndarray=None):
        '''
        Get the joint jacobian in the local frame of the joint
        q is motor-only (27), returns motor-only jacobian (6 x 27)
        '''
        if q is not None:
            q_full = self.full_q(q)
            data = self.model.createData()
        else:
            q_full = self.full_q(self.state['q'])
            data = self.data
        # update kinematics
        pin.forwardKinematics(self.model, data, q_full)
        pin.updateFramePlacements(self.model, data)
        # compute jacobian
        joint_id = self.model.getJointId(joint_name)
        jacobian_full = pin.computeJointJacobian(
            self.model,
            data,
            q_full,
            joint_id
        )
        # return motor-only columns (skip free-flyer columns)
        return jacobian_full[:, FREEFLYER_NV:]

    def get_frame_twist(self, frame_name: str):
        frame_id = self.model.getFrameId(frame_name)
        twist = pin.getFrameVelocity(
            self.model,
            self.data,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        return np.concatenate([twist.linear, twist.angular])

    def get_frame_wrench(self, frame_name: str,
                         q: np.ndarray=None, tau: np.ndarray=None, imu_quat=None):
        '''
        Estimate wrench from joint torques using a condition-aware least-squares solve

        Uses adaptive damped least-squares when the Jacobian is ill-conditioned to
        reduce sensitivity to torque noise near singular configurations
        '''
        cond_threshold = 10
        base_damping = 1e-1
        max_damping = 10
        svd_tol = 1e-3

        q = self.state['q'] if q is None else q
        tau = self.state['tau'] if tau is None else tau
        tau_gravity = self.get_gravity_compensation(q, imu_quat)
        jac = self.get_frame_jacobian(frame_name, q, imu_quat)
        tau_residual = tau - tau_gravity

        singular_values = np.linalg.svd(jac.T, compute_uv=False)
        if singular_values.size == 0 or singular_values[0] < svd_tol:
            return np.zeros(6)

        cond_number = singular_values[0] / max(singular_values[-1], svd_tol)
        if cond_number <= cond_threshold:
            damping = 0.0
        else:
            # increase damping smoothly once condition number passes threshold
            damping = base_damping * (cond_number / cond_threshold)
            damping = np.clip(damping, base_damping, max_damping)

        jj_t = jac @ jac.T
        rhs = jac @ tau_residual
        damping_sq = damping * damping
        wrench = np.linalg.solve(jj_t + damping_sq * np.eye(6), rhs)
        return wrench

    def get_frame_wrench_raw(self, frame_name: str,
                             q: np.ndarray=None, tau: np.ndarray=None, imu_quat=None):
        '''
        Pseudo-inverse solution without damping for reference/debugging
        '''
        q = self.state['q'] if q is None else q
        tau = self.state['tau'] if tau is None else tau
        tau_gravity = self.get_gravity_compensation(q, imu_quat)
        jac = self.get_frame_jacobian(frame_name, q, imu_quat)
        tau_residual = tau - tau_gravity
        wrench = np.linalg.pinv(jac.T) @ tau_residual
        return wrench

    def compute_frame_twist(self, frame_name: str, dq: np.ndarray):
        jac = self.get_frame_jacobian(frame_name)
        twist = jac @ dq
        return twist

    def check_valid(self, q):
        '''
        Check if the given joint position is valid
        q is motor-only (27)
        '''
        return self.check_within_limits(q) and self.check_collision_free(q)

    def check_within_limits(self, q):
        '''
        Check if the given motor joint position violates joint limits
        q is motor-only (27)
        '''
        q_full = self.full_q(q)
        return (
            np.all(q_full <= self.model.upperPositionLimit) and
            np.all(q_full >= self.model.lowerPositionLimit)
        )

    def check_within_limits_reduced(self, q_reduced):
        '''Check if the given joint position violates joint limits for the reduced model'''
        assert(self.init_reduced), 'Reduced model is not initialized.'
        return (
            np.all(q_reduced <= self.model_body_reduced.upperPositionLimit) and
            np.all(q_reduced >= self.model_body_reduced.lowerPositionLimit)
        )

    def check_collision_free(self, q):
        '''
        check if the given joint position violates collision constraints
        q is motor-only (27), uses collision_model_body (no free-flyer)
        '''
        assert(self.init_collision), 'Collision model is not initialized.'
        return not pin.computeCollisions(
            self.model_body, self.data_body,
            self.collision_model_body, self.collision_data_body, q,
            stop_at_first_collision=True
        )

    def check_collision_free_reduced(self, q_reduced):
        '''Check collision for the reduced model (model_body_reduced)'''
        assert(self.init_reduced), 'Reduced model is not initialized.'
        assert(self.init_collision), 'Collision model is not initialized.'
        return not pin.computeCollisions(
            self.model_body_reduced, self.data_body_reduced,
            self.collision_model_body_reduced, self.collision_data_body_reduced, q_reduced,
            stop_at_first_collision=True
        )
