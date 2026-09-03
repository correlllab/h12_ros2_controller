import numpy as np

from h12_ros2_controller.core.controller.counter_balance import (
    CounterDDPVelocityController,
)
from h12_ros2_controller.core.controller.counter_balance.counter_velocity_ocp import (
    CounterVelocityOCP,
)
from h12_ros2_controller.core.controller.counter_balance.frozen_3c_planner import (
    Frozen3CNominalInput,
)
from test_counter_balance_controller import _harness as _reactive_harness


def _controllers():
    reactive = _reactive_harness()
    reactive.com_weight = 1.0
    reactive.momentum_weight = 2.0
    reactive.posture_weight = 0.02
    reactive.damping = 0.0001
    reactive.com_velocity_scale = 0.1
    reactive.momentum_scale = 1.0
    reactive.posture_velocity_scale = 1.0
    ddp = CounterDDPVelocityController.__new__(CounterDDPVelocityController)
    ddp.__dict__.update(reactive.__dict__)
    ddp.velocity_ocp = CounterVelocityOCP(dt=reactive.dt)
    ddp.latest_velocity_ocp_result = None
    return reactive, ddp


def test_velocity_controller_matches_reactive_solver_on_random_problems():
    reactive, ddp = _controllers()
    rng = np.random.default_rng(12)
    differences = []
    for _ in range(100):
        com_counter = rng.normal(size=(2, 4)) * 0.1
        momentum_counter = rng.normal(size=(2, 4))
        com_rhs = rng.normal(size=2) * 0.1
        momentum_rhs = rng.normal(size=2)
        posture_target = rng.normal(size=4) * 0.2
        lower = rng.uniform(-2.0, -0.1, size=4)
        upper = rng.uniform(0.1, 2.0, size=4)
        balance_scale = rng.uniform(0.1, 1.0)
        expected = reactive._solve_bounded_velocity(
            com_counter,
            momentum_counter,
            com_rhs,
            momentum_rhs,
            posture_target,
            lower,
            upper,
            balance_scale,
        )
        actual = ddp._solve_bounded_velocity(
            com_counter,
            momentum_counter,
            com_rhs,
            momentum_rhs,
            posture_target,
            lower,
            upper,
            balance_scale,
        )
        differences.append(np.max(np.abs(actual - expected)))

    assert np.quantile(differences, 0.95) < 5e-4
    assert max(differences) < 3e-3


def test_velocity_controller_exposes_crocoddyl_diagnostics():
    _, controller = _controllers()
    controller._solve_bounded_velocity(
        np.zeros((2, 4)),
        np.eye(2, 4),
        np.zeros(2),
        np.ones(2),
        np.zeros(4),
        -np.ones(4),
        np.ones(4),
    )

    diagnostics = controller.diagnostics()

    assert diagnostics['velocity_ocp_available']
    assert diagnostics['velocity_ocp_solve_time'] > 0.0


def test_velocity_controller_reports_infeasible_excursion_as_safety_hold():
    _, controller = _controllers()
    controller.max_excursion = np.full(4, 0.01)
    q_target = np.zeros(14)
    dq_target = np.zeros(14)
    controller.control_configuration_step(q_target, dq_target)
    controller.robot_model.state['q'][controller.counter_ids] += 0.1

    controller.control_configuration_step(q_target, dq_target)

    assert controller.latest_status == 'counter_bounds_infeasible'


def test_nominal_planning_commits_diagnostics_only_after_explicit_commit():
    _, controller = _controllers()
    inputs = Frozen3CNominalInput(
        moving_dq=np.zeros(7),
        counter_q=np.zeros(4),
        counter_q_ref=np.zeros(4),
        com_error=np.zeros(2),
        gyro=np.zeros(3),
        com_moving=np.zeros((2, 7)),
        com_counter=np.eye(2, 4),
        momentum_moving=np.zeros((2, 7)),
        momentum_counter=np.eye(2, 4, k=2),
        balance_scale=1.0,
        lower=-np.ones(4),
        upper=np.ones(4),
        com_rhs_offset=np.zeros(2),
        momentum_rhs_offset=np.zeros(2),
    )

    plan = controller._plan_counter_velocity(inputs)

    assert controller.latest_velocity_ocp_result is None
    assert plan.solve_diagnostics is not None
    controller._commit_nominal_plan(plan)
    assert controller.latest_velocity_ocp_result is plan.solve_diagnostics
