'''Shared planner data types.

Kept dependency-free (no ompl / no cuRobo / no torch) so every backend and the
subprocess client can import PlannerConfig / PlanResult without dragging in a
native planning library. reduced_joint_planner re-exports these for backward
compatibility.
'''

from dataclasses import dataclass, field

import numpy as np


@dataclass
class PlannerConfig:
    '''Configuration for reduced joint-space planning'''

    # backend selects the planning implementation:
    #   'ompl'   -> ReducedJointPlanner (CPU, always available)
    #   'curobo' -> CuroboJointPlanner (CUDA GPU required)
    #   'auto'   -> prefer curobo when a GPU is present, else fall back to ompl
    backend: str = 'auto'
    # backend-specific settings for cuRobo (ignored by the OMPL backend)
    curobo: dict = field(default_factory=dict)
    planner: str = 'RRTConnect'
    timeout: float = 1.0
    range: float = 0.0
    try_direct: bool = True
    simplify: bool = True
    simplify_time: float = 1.0
    shortcut: bool = True
    moving_speed: float = 0.25
    dt: float = 1.0 / 30.0
    min_interpolation_steps: int = 2
    max_interpolation_steps: int = 300
    validity_resolution: float = 0.0025
    constraint_check_steps: int = 10
    frame_names: tuple = ('left_grasp_frame', 'right_grasp_frame')
    frame_z_min: float = None
    frame_z_min_margin: float = 1e-3
    frame_z_corridor_margin: float = 0.05


@dataclass
class PlanResult:
    '''Result returned by reduced joint-space planning'''

    success: bool
    path: np.ndarray
    reason: str = ''
    planning_time: float = 0.0
    planner_name: str = ''
    metadata: dict = field(default_factory=dict)
