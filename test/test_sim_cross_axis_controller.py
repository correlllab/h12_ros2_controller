from unittest.mock import Mock

import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_balance.scipy_nominal_planner import (
    solve_bounded_least_squares,
)
from h12_ros2_controller.core.controller.counter_balance.sim_cross_axis_controller import (
    SimCrossAxisController,
)
from test_counter_balance_diagnostics import _controller


def _configured(side, candidate):
    ctrl = _controller(side, 'c0')
    if candidate:
        ctrl.__class__ = SimCrossAxisController
        ctrl.configure_diagnostic_trial(candidate)
    ctrl._reference_captured = True
    ctrl.q_counter_ref = np.zeros(4)
    ctrl.counter_wrist_ref = np.zeros(3)
    ctrl.robot_model.state['q'][ctrl.counter_ids] = [0.03, -0.04, 0.07, -0.08]
    ctrl.com_offset_ref = np.array([-0.02, 0.01])
    # deliberately disagree with the planned moving velocity, and supply nonzero gyro feedback
    ctrl.robot_model.state['dq'][ctrl.moving_ids] = [-0.11, 0.23, 0., 0., 0., 0., 0.]
    ctrl.robot_model.state['imu_state'].gyroscope = [0.2, -0.1, 0.3]
    return ctrl


def _step(ctrl, xy, scale):
    dq = np.zeros(14)
    dq[ctrl.moving_local] = [*xy, 0.25, -0.15, 0.02, 0.04, 0.06]
    return ctrl.control_configuration_step(np.zeros(14), dq, balance_scale=scale)


@pytest.mark.parametrize('candidate,xy,scale', [
    ('c0', (0.1, -0.2), 1.0),
    ('sim_cross_axis', (0.1, -0.2), 0.0),
    ('sim_cross_axis', (0.0, 0.0), 0.37),
])
def test_off_and_zero_effect_are_byte_exact(candidate, xy, scale):
    c0, cross = _configured('left', None), _configured('left', candidate)
    assert _step(c0, xy, scale).tobytes() == _step(cross, xy, scale).tobytes()
    for field in ('objective_matrix', 'objective_target', 'requested_counter_dq', 'lower', 'upper'):
        assert getattr(c0._latest_nominal_plan, field).tobytes() == getattr(
            cross._latest_nominal_plan, field,
        ).tobytes()
    for field in ('q_cmd', 'dq_cmd', 'tau_cmd'):
        assert getattr(c0.low_cmd_handler, field).tobytes() == getattr(
            cross.low_cmd_handler, field,
        ).tobytes()
    assert cross._nominal_prepared_context == c0._nominal_prepared_context
    diag = cross.diagnostics()['counter_balance_diagnostic']
    assert diag['variant'] == diag['actuation'] == candidate
    if candidate == 'sim_cross_axis':
        assert diag['cross_axis_row']['active'] is False
    else:
        assert 'cross_axis_row' not in diag


@pytest.mark.parametrize('side,xy,expected_perp', [
    ('left', (0.0, 0.2), (-1.0, 0.0)),
    ('right', (0.2, 0.0), (0.0, 1.0)),
    ('left', (0.0003, -0.0004), (0.4, 0.3)),
    ('right', (-0.3, 0.4), (-0.8, -0.6)),
])
@pytest.mark.parametrize('scale', [1.0, 0.37])
def test_exact_shadow_row_and_bounded_solve(side, xy, expected_perp, scale):
    c0, cross = _configured(side, None), _configured(side, 'sim_cross_axis')
    for ctrl in (c0, cross):
        ctrl.max_velocity = np.full(4, 0.04)
        _step(ctrl, xy, scale)
    base, alt = c0._latest_nominal_plan, cross._latest_nominal_plan
    assert base.accepted and alt.accepted
    np.testing.assert_array_equal(alt.objective_matrix[:-1], base.objective_matrix)
    np.testing.assert_array_equal(alt.objective_target[:-1], base.objective_target)
    moving = np.asarray(xy)
    direction = moving / max(0.001, np.linalg.norm(moving))
    perp = np.array([-direction[1], direction[0]])
    np.testing.assert_allclose(perp, expected_perp, atol=1e-15, rtol=0)
    assert perp @ moving == pytest.approx(0., abs=1e-16)
    np.testing.assert_array_equal(alt.objective_matrix[-1], perp @ base.objective_matrix[:2])
    assert alt.objective_target[-1] == 0.0
    for field in ('lower', 'upper'):
        np.testing.assert_array_equal(getattr(alt, field), getattr(base, field))
    expected = solve_bounded_least_squares(
        np.vstack((base.objective_matrix, perp @ base.objective_matrix[:2])),
        np.append(base.objective_target, 0.), base.lower, base.upper,
    )
    assert expected.accepted
    np.testing.assert_array_equal(alt.requested_counter_dq, expected.velocity)
    assert np.all((alt.lower <= alt.requested_counter_dq) & (alt.requested_counter_dq <= alt.upper))
    diag = cross.diagnostics()['counter_balance_diagnostic']['cross_axis_row']
    assert diag['frame'] == 'fixed_base_model_xy' and diag['active']
    np.testing.assert_array_equal(diag['weighted_row'], alt.objective_matrix[-1])


def test_finalizer_backtracks_and_identity_requires_publication():
    cross = _configured('right', 'sim_cross_axis')
    unchecked = _configured('right', 'sim_cross_axis')
    _step(unchecked, (0.17, -0.25), 0.55)
    measured = cross.robot_model.state['q'][cross.counter_ids]
    max_delta = cross.dt * np.max(np.abs(unchecked.latest_requested_counter_dq))
    assert max_delta > 0.0
    # accept the hold and half step, but not the full step
    cross.robot_model.check_collision_free = lambda q: (
        np.max(np.abs(q[cross.counter_ids] - measured)) < 0.75 * max_delta
    )
    _step(cross, (0.17, -0.25), 0.55)
    diag = cross.diagnostics()
    assert diag['status'] == 'collision_backtracked'
    assert diag['backtrack_scale'] == 0.5
    assert diag['published']
    assert diag['counter_balance_diagnostic']['actuation'] == 'sim_cross_axis'
    requested = np.asarray(diag['nominal_problem_requested'])
    assert np.any(requested != 0.0)
    np.testing.assert_array_equal(diag['applied_counter_dq'], 0.5 * requested)
    np.testing.assert_array_equal(cross.low_cmd_handler.dq_cmd[cross.counter_ids], 0.5 * requested)
    assert len(cross.low_cmd_handler.calls) == 1

    cross.robot_model.check_collision_free = Mock(return_value=False)
    _step(cross, (0.17, -0.25), 0.55)
    diag = cross.diagnostics()['counter_balance_diagnostic']
    assert diag['variant'] == 'sim_cross_axis'
    assert diag['actuation'] == 'not_delivered'
