"""Reproduce original 26 fixtures from benchmark root; not full safety coverage"""

import json
from pathlib import Path

import numpy as np
import pytest

from iter7b_recovery_fixture import ROOT, Harness, Recovery, PARAMETERS, make_model, sample


@pytest.fixture(scope='module', params=['F02_counter', 'F03_counter'])
def fixture_data(request):
    campaign = ROOT / 'runs/real_experiments/iter7b_fame_validation_20261110_184722'
    path = next((campaign / request.param).glob('*/arm_target.jsonl'))
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    meta = json.loads(path.with_name('metadata.json').read_text())
    config = json.loads(path.with_name('resolved_config.yaml').read_text())['controller']
    limits = rows[0]['diagnostics']['controller_diagnostics']['nominal_prepared_context']['publication']['limits']['q_clip_limits']
    transition = next(i for i, row in enumerate(rows) if row['diagnostics']['controller_diagnostics']['balance_scale'] == 0)
    return rows, meta, config, make_model(limits), transition


@pytest.fixture
def seam(fixture_data):
    rows, meta, config, model, i = fixture_data
    h = Harness(rows[0], config, model, meta['task']['moving_arm'])
    r = Recovery(h, 'original')
    _, _, q, dq, _, _, _ = sample(rows[i - 1])
    aq, av, tau = h.baseline(rows[i - 1])
    assert r.step(q, dq, aq, av, tau, False) == 'baseline'
    _, _, q, dq, _, _, _ = sample(rows[i])
    aq, av, tau = h.baseline(rows[i])
    return h, r, [q, dq, aq, av, tau, True]


def test_actual_seed_step_velocity_and_ownership(seam):
    h, r, args = seam
    c = h.c
    old_q = h.c.low_cmd_handler.q_cmd.copy()
    old_v = h.c.low_cmd_handler.dq_cmd.copy()
    assert r.step(*args) == 'accepted'
    assert np.allclose(r.reference, old_q[c.counter_ids] + c.dt * r.velocity, atol=1e-7)
    assert np.max(abs(r.velocity - old_v[c.counter_ids])) <= c.dt + 1e-7
    assert np.array_equal(h.c.low_cmd_handler.q_cmd[:13], old_q[:13])
    assert np.array_equal(h.c.low_cmd_handler.q_cmd[c.moving_ids], args[2][c.moving_local].astype(np.float32))
    assert np.max(abs(h.c.low_cmd_handler.q_cmd[c.counter_ids] - args[2][c.counter_active_local])) > 1e-5
    assert np.array_equal(h.proposals[-1][c.counter_ids].astype(np.float32), r.reference)
    assert h.c.low_cmd_handler.q_cmd.dtype == np.float32
    assert np.array_equal(h.kp, [240, 200, 150, 150])
    assert np.array_equal(h.kd, [12, 12, 12, 12])
    expected = h.kp * (r.reference - args[0][c.counter_ids]) + h.kd * (r.velocity - args[1][c.counter_ids]) + args[4][c.counter_active_local]
    assert np.allclose(r.effort(*args[:5]), h.kp * (args[2][c.counter_active_local] - args[0][c.counter_ids]) + h.kd * (args[3][c.counter_active_local] - args[1][c.counter_ids]) + args[4][c.counter_active_local])
    actual = r.effort(args[0], args[1], h.c.low_cmd_handler.q_cmd[13:], h.c.low_cmd_handler.dq_cmd[13:], args[4])
    assert np.allclose(actual, expected)


def test_skipped_updates_never_accumulate(seam):
    h, r, args = seam
    assert r.step(*args) == 'accepted'
    ref, vel = r.reference.copy(), r.velocity.copy()
    writes = h.writes
    for _ in range(50):
        assert r.step(*args, eligible=False) == 'skipped_no_write'
    assert h.writes == writes
    assert np.array_equal(r.reference, ref)
    assert np.array_equal(r.velocity, vel)
    assert r.step(*args) == 'accepted'
    assert np.allclose(r.reference - ref, h.c.dt * r.velocity, atol=1e-7)


@pytest.mark.parametrize('fault', ['stale', 'operator'])
def test_stale_and_stop_latch_no_untrusted_hold(seam, fault):
    h, r, args = seam
    assert r.step(*args) == 'accepted'
    writes, validations = h.writes, len(h.proposals)
    args[0] = np.full(27, np.nan)
    status = r.step(*args, fresh=fault != 'stale', stopped=fault == 'operator')
    assert status.startswith('stop_required:')
    assert h.writes == writes and len(h.proposals) == validations
    assert r.reference is None and r.latched and r.stop_latched
    assert h.stop_requests == 1
    assert r.step(*args) == 'stop_latched_no_write'
    assert h.writes == writes


def test_collision_checks_actual_proposal_and_validated_fallback(seam):
    h, r, args = seam
    original = h.check
    seen = []
    def reject_first(q, aq):
        seen.append(aq.copy())
        return 'recovery_candidate_collision' if len(seen) == 1 else original(q, aq)
    h.check = reject_first
    status = r.step(*args)
    assert status == 'validated_hold:recovery_candidate_collision'
    assert len(seen) == 2
    assert not np.array_equal(seen[0][h.c.counter_active_local], seen[1][h.c.counter_active_local])
    assert np.array_equal(seen[1][h.c.counter_active_local], args[0][h.c.counter_ids])
    assert np.array_equal(h.c.low_cmd_handler.dq_cmd[h.c.counter_arm_ids], np.zeros(7))
    assert r.reference is None and r.latched and r.accepted == 0
    assert r.step(*args) == 'validated_hold:latched'
    h.check = original
    r.rearm('original')
    assert r.step(*args) == 'accepted'
    assert np.max(abs(r.reference - args[0][h.c.counter_ids])) <= h.c.dt ** 2 + 1e-7


def test_invalid_fallback_stops_without_writing(seam):
    h, r, args = seam
    h.check = lambda q, aq: 'recovery_candidate_collision'
    writes = h.writes
    assert r.step(*args).startswith('stop_required:')
    assert h.writes == writes and h.stop_requests == 1
    assert r.reference is None and r.stop_latched


def test_new_target_cannot_reuse_old_reference(seam):
    h, r, args = seam
    assert r.step(*args) == 'accepted'
    assert r.step(*args, token='new_target') == 'validated_hold:new_target'
    assert r.reference is None and r.latched
    assert r.step(*args) == 'validated_hold:latched'
    r.rearm('new_target')
    assert r.step(*args, token='new_target') == 'accepted'


def test_new_moving_target_is_checked_not_only_counter(seam):
    h, r, args = seam
    args[2][h.c.moving_local[0]] = 100
    writes = h.writes
    assert r.step(*args).startswith('stop_required:recovery_candidate_out_of_limits')
    assert h.writes == writes and r.reference is None


@pytest.mark.parametrize('guard', ['lead', 'effort', 'excursion'])
def test_guard_rejection_does_not_commit(seam, guard):
    h, r, args = seam
    assert r.step(*args) == 'accepted'
    if guard == 'lead':
        r.reference += .08
    elif guard == 'effort':
        args[1][h.c.counter_ids] = -10
    else:
        h.c.max_excursion[:] = 1e-5
    count = r.accepted
    status = r.step(*args)
    assert guard in status
    assert r.reference is None and r.accepted == count and r.latched


def test_publication_failure_never_commits(seam):
    h, r, args = seam
    h.write = lambda *unused: False
    assert r.step(*args) == 'stop_required:publication'
    assert r.reference is None and r.accepted == 0 and h.stop_requests == 1


def test_guard_fixture_uses_real_limit_and_collision_checks(seam):
    h, r, args = seam
    assert h.c.robot_model.init_collision
    assert h.check(args[0], args[2]) is None
    bad = args[2].copy()
    bad[h.c.counter_active_local[0]] = 100
    assert h.check(args[0], bad) == 'recovery_candidate_out_of_limits'
