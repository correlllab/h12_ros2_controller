import json
import hashlib
from dataclasses import dataclass, asdict

import numpy as np


def command_context_digest(controller):
    '''Fingerprint configuration and joint topology, not frozen-joint angles'''
    model = controller.robot_model.model_body
    planner = controller.planner
    context = {
        'config': controller.config,
        'planner': asdict(planner.planner_config),
        'dt': controller.dt,
        'joints': list(model.names),
        'parents': np.asarray(model.parents).tolist(),
        'reduced_mask': controller.robot_model.reduced_mask.tolist(),
        'lower': model.lowerPositionLimit.tolist(),
        'upper': model.upperPositionLimit.tolist(),
        'velocity': model.velocityLimit.tolist(),
        'assets': [str(getattr(planner, name)) for name in (
            'urdf_path', 'urdf_sphere_path', 'srdf_sphere_path',
        )],
    }
    encoded = json.dumps(context, sort_keys=True, default=lambda value: value.tolist())
    return hashlib.sha256(encoded.encode()).hexdigest()


@dataclass(frozen=True)
class PlanReference:
    '''Immutable waypoint bytes with their original creation provenance'''

    path: np.ndarray
    created_at: float
    dt: float
    config_digest: str
    checksum: str

    @classmethod
    def capture(cls, controller, path):
        result = controller.last_plan_result
        if result is None or not result.success or not np.array_equal(path, result.path):
            raise ValueError('Reference must match the most recent validated plan')
        context = result.metadata['execution_context']
        path = np.ascontiguousarray(path, dtype=np.float64)
        # a bytes backing store prevents callers from re-enabling writes
        copied = np.frombuffer(path.tobytes(), dtype=np.float64).reshape(path.shape)
        return cls(copied, context['planned_at'], context['dt'],
                   command_context_digest(controller),
                   hashlib.sha256(copied.tobytes()).hexdigest())

    def check(self, controller, path):
        if (not np.array_equal(path, self.path) or
                hashlib.sha256(self.path.tobytes()).hexdigest() != self.checksum):
            raise ValueError('Cached reference path differs from validated waypoints')
        if command_context_digest(controller) != self.config_digest:
            raise ValueError('Cached reference model/configuration differs')


@dataclass(frozen=True)
class ValidatedPlan:
    '''Fresh checks of a saved reference without changing its creation time'''

    reference: PlanReference
    validated_at: float
