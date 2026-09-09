from types import SimpleNamespace

import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_controller import (
    CounterResidualH2Controller,
)
from h12_ros2_controller.core.controller.counter_balance.counter_residual_h2_robust_controller import (
    CounterResidualH2RobustController,
)
from test_counter_residual_h2_controller import _h2_controller
from test_counter_ddp_velocity_robust_controller import _robust, _solve_result
import h12_ros2_controller.core.controller.counter_balance.counter_ddp_velocity_robust_controller as adapter


def _robust_h2():
    controller = _h2_controller()
    controller.__class__ = CounterResidualH2RobustController
    controller.velocity_ocp = None
    controller.h2_shadow = False
    controller._h2_pending_residual = np.zeros(4)
    return controller


def test_robust_h2_uses_shared_scipy_not_archived_crocoddyl(monkeypatch):
    # the old Crocoddyl-first robust baseline is archived at commit 9256ab6
    controller = _robust_h2()
    calls = []
    result = _solve_result()
    monkeypatch.setattr(
        adapter, 'solve_scipy_nominal',
        lambda *args: calls.append(args) or result,
    )
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    assert len(calls) == 1
    assert controller.latest_velocity_ocp_result is result.diagnostics
    assert not controller.latest_nominal_fallback_used
    assert len(controller.low_cmd_handler.calls) == 1


def test_robust_h2_fallback_alias_means_scipy_trf_retry(monkeypatch):
    controller = _robust_h2()
    result = _solve_result(retry=True)
    monkeypatch.setattr(adapter, 'solve_scipy_nominal', lambda *args: result)
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    diagnostics = controller.diagnostics()
    assert diagnostics['nominal_fallback_used']
    assert diagnostics['nominal_retry_used']
    assert not diagnostics['nominal_primary_accepted']
    assert diagnostics['nominal_backend'] == 'trf'


@pytest.mark.parametrize('scale', [1.0, 0.5, 0.0, None])
def test_zero_applied_residual_command_history_matches_nominal(monkeypatch, scale):
    nominal = _robust()
    residual = _robust_h2()
    residual._run_h2 = lambda *args: np.zeros(4)
    nominal.robot_model.state['imu_state'].gyroscope = [0.08, -0.05, 0.0]
    counts = []
    for controller in (nominal, residual):
        count = {}
        for name in (
            'update_robot_model', '_capture_control_observation',
            '_model_terms', '_capture_reference', '_plan_counter_velocity',
            '_commit_nominal_plan', '_finalize_counter_velocity',
        ):
            original = getattr(controller, name)

            def bind(original, name, count):
                def wrapped(*args, **kwargs):
                    count[name] = count.get(name, 0) + 1
                    return original(*args, **kwargs)
                return wrapped

            monkeypatch.setattr(controller, name, bind(original, name, count))
        counts.append(count)
        if scale is None:
            controller._backtrack_counter = lambda *args: (None, 'test_hold')
        else:
            controller.backtrack_scales = (scale,)

    for index in range(4):
        q = np.zeros(14)
        dq = np.zeros(14)
        dq[:4] = [0.1, -0.1, 0.02, -0.03]
        for controller in (nominal, residual):
            controller.robot_model.state['q'][controller.counter_ids] = index * 0.001
            controller.control_configuration_step(q, dq, balance_scale=0.7)
        for name in ('q', 'dq', 'tau'):
            assert np.array_equal(
                nominal.low_cmd_handler.calls[-1][name],
                residual.low_cmd_handler.calls[-1][name],
            )
        assert np.array_equal(residual._h2_pending_residual, np.zeros(4))
        assert nominal.latest_backtrack_scale == residual.latest_backtrack_scale
        assert nominal.latest_backtrack_scale == (0.0 if scale is None else scale)
        assert np.linalg.norm(nominal.latest_requested_counter_dq) > 0.0
        assert nominal.latest_status == residual.latest_status
    assert len(nominal.low_cmd_handler.calls) == 4
    assert len(residual.low_cmd_handler.calls) == 4
    assert counts[0] == counts[1]
    assert counts[0].pop('_capture_reference') == 1
    assert set(counts[0].values()) == {4}


@pytest.mark.parametrize('failure', ['shape', 'nan', 'bounds', 'yaw', 'rejected', 'exception'])
@pytest.mark.parametrize('reset_fails', [False, True])
def test_invalid_h2_retains_valid_nominal(monkeypatch, failure, reset_fails):
    nominal = _robust()
    controller = _robust_h2()
    monkeypatch.setattr(adapter, 'solve_scipy_nominal', lambda *args: _solve_result())

    def solve(*args, **kwargs):
        if failure == 'exception':
            raise RuntimeError('H2 failed')
        value = {
            'shape': np.zeros(3),
            'nan': np.array([np.nan, 0.0, 0.0, 0.0]),
            'bounds': np.ones(4),
            'yaw': np.array([0.0, 0.0, 0.01, 0.0]),
            'rejected': np.zeros(4),
        }[failure]
        return SimpleNamespace(
            residual=value, accepted=failure != 'rejected', status='test',
            solve_time=0.0, iterations=0, stopping_criterion=0.0,
            seed_cost=0.0, optimized_cost=0.0, warm_started=False, xs=[],
        )

    controller.h2_ocp.solve = solve
    if reset_fails:
        def reset():
            raise RuntimeError('reset failed')
        controller.h2_ocp.reset = reset
    for candidate in (nominal, controller):
        candidate.control_configuration_step(np.zeros(14), np.zeros(14))
    assert controller.latest_h2_status == 'model_failure'
    assert not controller.latest_h2_accepted
    assert np.array_equal(controller._h2_pending_residual, np.zeros(4))
    assert np.array_equal(controller.latest_h2_residual, np.zeros(4))
    assert len(controller.low_cmd_handler.calls) == 1
    for name in ('q', 'dq', 'tau'):
        assert np.array_equal(
            controller.low_cmd_handler.calls[-1][name],
            nominal.low_cmd_handler.calls[-1][name],
        )
    if reset_fails:
        assert 'reset failed' in controller.latest_h2_error


def test_h2_uses_prepared_observation_after_nominal_solve(monkeypatch):
    controller = _robust_h2()
    seen = []

    def solve(*args):
        controller.robot_model.state['dq'][:] = np.nan
        controller.robot_model.state['imu_state'].quaternion[:] = [np.nan] * 4
        return _solve_result()

    original = CounterResidualH2Controller._run_h2

    def run(self, context, nominal):
        state = self._control_observation_state()
        seen.append((state['dq'].copy(), list(state['imu_state'].quaternion)))
        return original(self, context, nominal)

    monkeypatch.setattr(adapter, 'solve_scipy_nominal', solve)
    monkeypatch.setattr(CounterResidualH2Controller, '_run_h2', run)
    controller.control_configuration_step(np.zeros(14), np.zeros(14))
    assert len(seen) == 1
    assert np.all(np.isfinite(seen[0][0]))
    assert seen[0][1] == [1.0, 0.0, 0.0, 0.0]
    assert controller._nominal_observation is None
