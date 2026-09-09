from types import SimpleNamespace

import numpy as np
import pytest

import h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_controller as ddp
import h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_robust_controller as adapter
import h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_controller as h2
from h12_ros2_controller.core.controller.counter_balance.frozen_3c_planner import (
    Frozen3CVelocitySolve,
)
from test_counter_balance_controller import _harness


def _robust():
    controller = _harness()
    controller.__class__ = adapter.CounterDDPVelocityRobustController
    controller.velocity_ocp = None
    controller.latest_velocity_ocp_result = None
    controller.com_weight = 1.0
    controller.momentum_weight = 2.0
    controller.posture_weight = 0.02
    controller.damping = 0.0001
    controller.momentum_scale = 1.0
    controller.posture_velocity_scale = 1.0
    return controller


def _solve_result(accepted=True, retry=False):
    velocity = np.array([0.01, -0.02, 0.0, 0.005])
    diagnostics = SimpleNamespace(
        velocity=velocity, accepted=accepted, converged=accepted, cost=0.0,
        solve_time=0.01, iterations=1, stopping_criterion=0.0,
        kkt_violation=0.0, regularization=0.0, boxqp_polished=False,
        backend='trf' if retry else 'bvls', retry_used=retry,
        primary_accepted=accepted and not retry, status='test',
        attempts=(SimpleNamespace(accepted=accepted and not retry),),
    )
    return Frozen3CVelocitySolve(velocity, accepted, diagnostics)


def test_diagnostics_record_exact_nominal_problem_without_stale_hold_data():
    controller = _robust()
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    nominal = controller._latest_nominal_plan
    values = controller.diagnostics()
    for field, expected in (
        ('matrix', nominal.objective_matrix),
        ('target', nominal.objective_target),
        ('lower', nominal.lower),
        ('upper', nominal.upper),
        ('requested', nominal.requested_counter_dq),
    ):
        np.testing.assert_array_equal(values[f'nominal_problem_{field}'], expected)
    assert len(controller.low_cmd_handler.calls) == 1
    with pytest.raises(ValueError, match='moving-arm command must be finite'):
        controller.control_configuration_step(np.full(14, np.nan), np.zeros(14))
    assert 'nominal_problem_matrix' not in controller.diagnostics()


def test_total_time_covers_outer_robust_wrapper(monkeypatch):
    controller = _robust()
    clock = iter((10.0, 10.125))
    monkeypatch.setattr(adapter.time, 'perf_counter', lambda: next(clock))

    def inner(self, *args, **kwargs):
        self.latest_velocity_total_time = 0.01
        self._nominal_observation = {'q': np.zeros(27)}
        return np.zeros(14)

    monkeypatch.setattr(ddp.CounterDDPVelocityController,
                        'control_configuration_step', inner)
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    assert controller.latest_velocity_total_time == 0.125
    assert controller._nominal_observation is None


@pytest.mark.parametrize('with_h2', [False, True])
def test_robust_constructors_never_construct_nominal_ocp(monkeypatch, with_h2):
    def initialize(self, *args, **kwargs):
        self.dt = 0.02

    def forbidden(*args, **kwargs):
        pytest.fail('nominal Crocoddyl OCP or nominal-only H2 constructed')

    monkeypatch.setattr(ddp.CounterBalanceController, '__init__', initialize)
    monkeypatch.setattr(ddp, 'CounterVelocityOCP', forbidden)
    if with_h2:
        from h12_ros2_controller.core.controller.counter_balance import (
            CounterResidualH2RobustController,
        )
        cls = CounterResidualH2RobustController
    else:
        monkeypatch.setattr(h2, 'verified_h2_models', forbidden)
        monkeypatch.setattr(h2, 'ResidualH2OCP', forbidden)
        cls = adapter.CounterDDPVelocityRobustController
    controller = cls(config={})
    assert controller.velocity_ocp is None
    assert hasattr(controller, 'h2_models') == with_h2


def test_isolated_scipy_solve_only_forwards_explicit_inputs(monkeypatch):
    controller = _robust()
    args = (
        np.zeros((2, 4)), np.zeros((2, 4)), np.zeros(2), np.zeros(2),
        np.zeros(4), -np.ones(4), np.ones(4), 0.3,
    )
    expected = _solve_result()
    calls = []
    monkeypatch.setattr(
        adapter, 'solve_scipy_nominal', lambda *args: calls.append(args) or expected,
    )
    before = controller.__dict__.copy()
    assert controller._isolated_velocity_solve(*args) is expected
    assert controller.__dict__.keys() == before.keys()
    assert all(controller.__dict__[key] is value for key, value in before.items())
    assert len(calls) == 1
    assert all(actual is value for actual, value in zip(calls[0][:-1], args))
    config = calls[0][-1]
    for name in (
        'com_weight', 'momentum_weight', 'posture_weight', 'damping',
        'com_velocity_scale', 'momentum_scale', 'posture_velocity_scale',
    ):
        assert getattr(config, name) == getattr(controller, name)
    assert controller.low_cmd_handler.calls == []


@pytest.mark.parametrize('cls', [
    ddp.CounterDDPVelocityController, h2.CounterResidualH2Controller,
])
def test_frozen_constructors_keep_nominal_ocp_factory(monkeypatch, cls):
    calls = []
    sentinel = object()

    def initialize(self, *args, **kwargs):
        self.dt = 0.02

    monkeypatch.setattr(ddp.CounterBalanceController, '__init__', initialize)
    monkeypatch.setattr(
        ddp, 'CounterVelocityOCP',
        lambda **kwargs: calls.append(kwargs) or sentinel,
    )
    controller = cls(config={})
    assert controller.velocity_ocp is sentinel
    assert calls == [{'dt': 0.02}]


def test_rejected_nominal_without_backend_attempt_still_holds(monkeypatch):
    controller = _robust()
    result = _solve_result(accepted=False)
    result.diagnostics.attempts = ()
    monkeypatch.setattr(adapter, 'solve_scipy_nominal', lambda *args: result)
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    assert controller.latest_nominal_primary_result is None
    assert controller.latest_velocity_ocp_result is result.diagnostics
    assert len(controller.low_cmd_handler.calls) == 1
    assert controller.latest_status == 'solver_failure'


@pytest.mark.parametrize('failure', ['rejected', 'exception', 'bounds'])
@pytest.mark.parametrize('with_h2', [False, True])
def test_nominal_failure_publishes_one_hold_without_h2(monkeypatch, failure, with_h2):
    from test_counter_residual_h2_robust_controller import _robust_h2
    from h12_ros2_controller.core.controller.counter_balance.objective import (
        CounterVelocityBoundsError,
    )
    controller = _robust_h2() if with_h2 else _robust()

    def solve(*args):
        if failure == 'exception':
            raise RuntimeError('nominal failed')
        if failure == 'bounds':
            raise CounterVelocityBoundsError('empty bounds')
        return _solve_result(accepted=False)

    def forbidden(*args):
        pytest.fail('H2 or finalizer called after nominal failure')

    controller._select_requested_counter_velocity = forbidden
    controller._finalize_counter_velocity = forbidden
    monkeypatch.setattr(adapter, 'solve_scipy_nominal', solve)
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    assert len(controller.low_cmd_handler.calls) == 1
    assert controller.latest_status == (
        'counter_bounds_infeasible' if failure == 'bounds' else 'solver_failure'
    )
    assert np.array_equal(controller.latest_applied_counter_dq, np.zeros(4))


@pytest.mark.parametrize('failure', ['update', 'model', 'support', 'estop'])
@pytest.mark.parametrize('with_h2', [False, True])
def test_early_failures_never_solve_or_publish_twice(monkeypatch, failure, with_h2):
    from test_counter_residual_h2_robust_controller import _robust_h2
    controller = _robust_h2() if with_h2 else _robust()

    def fail(*args):
        raise RuntimeError('preparation failed')

    def forbidden(*args):
        pytest.fail('nominal solver called after preparation failure')

    monkeypatch.setattr(adapter, 'solve_scipy_nominal', forbidden)
    if failure == 'update':
        controller.update_robot_model = fail
    elif failure == 'model':
        controller._model_terms = fail
    elif failure == 'support':
        controller._model_terms(None)[0].valid = False
    else:
        controller.low_cmd_handler._estopped = True
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    assert len(controller.low_cmd_handler.calls) == (0 if failure == 'estop' else 1)
    assert controller._nominal_observation is None
    assert controller.latest_velocity_ocp_result is None
