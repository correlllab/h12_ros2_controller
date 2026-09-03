import numpy as np

from h12_ros2_controller.core.controller.counter_balance.frozen_3c_planner import (
    Frozen3CNominalInput,
    Frozen3CPlannerConfig,
    Frozen3CVelocitySolve,
    plan_frozen_3c_velocity,
)


def _inputs():
    return Frozen3CNominalInput(
        moving_dq=np.array([0.2, -0.1]),
        counter_q=np.array([0.1, -0.2, 0.3, -0.4]),
        counter_q_ref=np.zeros(4),
        com_error=np.array([0.01, -0.02]),
        gyro=np.array([0.3, -0.4, 0.0]),
        com_moving=np.eye(2),
        com_counter=np.eye(2, 4),
        momentum_moving=np.array([[1.0, 2.0], [-1.0, 1.0]]),
        momentum_counter=np.eye(2, 4, k=2),
        balance_scale=0.5,
        lower=-np.ones(4),
        upper=np.ones(4),
        com_rhs_offset=np.zeros(2),
        momentum_rhs_offset=np.zeros(2),
    )


def test_frozen_3c_planner_is_nonpublishing_and_does_not_mutate_inputs():
    inputs = _inputs()
    arrays = {
        name: np.copy(value)
        for name, value in vars(inputs).items()
        if isinstance(value, np.ndarray)
    }
    calls = []

    def solve(*args):
        calls.append(tuple(np.copy(value) for value in args))
        return Frozen3CVelocitySolve(
            requested_counter_dq=np.array([0.1, 0.2, 0.3, 0.4]),
            accepted=True,
            diagnostics={'status': 'accepted'},
            objective_matrix=np.eye(4),
            objective_target=np.ones(4),
        )

    plan = plan_frozen_3c_velocity(
        inputs,
        Frozen3CPlannerConfig(
            com_gain=2.0,
            gyro_gain=0.1,
            posture_gain=0.5,
        ),
        solve,
    )

    assert len(calls) == 1
    assert np.allclose(plan.com_rhs, [-0.11, 0.07])
    assert np.allclose(plan.momentum_rhs, [0.015, 0.13])
    assert np.allclose(plan.posture_target, [-0.05, 0.1, -0.15, 0.2])
    assert plan.solve_diagnostics == {'status': 'accepted'}
    for name, expected in arrays.items():
        assert np.array_equal(getattr(inputs, name), expected)
    for value in (
        plan.requested_counter_dq,
        plan.com_rhs,
        plan.momentum_rhs,
        plan.posture_target,
        plan.lower,
        plan.upper,
        plan.objective_matrix,
        plan.objective_target,
    ):
        assert not value.flags.writeable
