import numpy as np
import pinocchio as pin

from h12_ros2_controller.core.controller.upper_controller import UpperController

class ImpedanceController(UpperController):
    def __init__(self,
                 urdf_path: str,
                 urdf_sphere_path: str,
                 srdf_sphere_path: str,
                 handless: bool=False,
                 visualize: bool=False,
                 config: dict=None):
        # initialize base controller
        super().__init__(
            urdf_path=urdf_path,
            urdf_sphere_path=urdf_sphere_path,
            srdf_sphere_path=srdf_sphere_path,
            handless=handless,
            visualize=visualize,
            config=config,
        )

    def impedance_step(self, x_target):
        # get states in Cartesian space
        x = self.left_ee_position
        dx = self.robot_model.dynamics.get_frame_twist(self.left_ee_name)[0:3]

        # spring damper
        kp = np.array([100.0, 100.0, 100.0])
        kd = np.array([5.0, 5.0, 5.0])
        x_target = self.left_ee_target_position
        F = kp * (x_target - x) + kd * (-dx)

        # inverse dynamics using model_body
        J_left = self.robot_model.dynamics.get_frame_jacobian(self.left_ee_name)
        tau = J_left.T @ np.concatenate([F, np.zeros(3)])
        tau_gravity = pin.computeGeneralizedGravity(
            self.robot_model.model_body,
            self.robot_model.data_body,
            self.robot_model.state['q']
        )
        tau_cmd = tau + tau_gravity

        # apply the control
        self.command_publisher.tau = tau_cmd
        # update robot model
        self.update_robot_model()
