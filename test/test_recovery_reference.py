"""Offline fault contracts only; no SDK, hardware enable or commissioned limits"""

import sys
from pathlib import Path
from dataclasses import replace
from importlib.util import module_from_spec, spec_from_file_location

import numpy as np
import pytest

# bypass the existing package initializer, which eagerly imports controllers
MODULE_PATH = Path(__file__).resolve().parents[1] / (
    'h12_ros2_controller/core/controller/counter_balance/recovery_reference.py')
SPEC = spec_from_file_location('pure_recovery_reference', MODULE_PATH)
MODULE = module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
Command = MODULE.Command
BoundValues = MODULE.BoundValues
ReferencePlanner = MODULE.ReferencePlanner


def make_planner(*, validator=lambda old, new: True, seed=None, bounds=None):
    bounds = bounds or BoundValues.research_fixture()
    seed = seed or Command((.1,) * 4, (0.,) * 4, (0.,) * 4)
    planner = ReferencePlanner(bounds, validator)
    planner.reset_trial('one', (0.,) * 4, seed, accepted_at=0., mode_ack=True)
    return planner


def propose(planner, **changes):
    args = dict(q=(.1,) * 4, dq=(0.,) * 4, tau=(0.,) * 4,
                now=1 / 30, source_time=1 / 30, trial_id='one', postfade=True)
    args.update(changes)
    return planner.propose(**args)


def commit(planner, proposal, **changes):
    args = dict(accepted=True, actual_buffer=proposal.command,
                now=proposal.created_at)
    args.update(changes)
    return planner.commit(proposal, **args)


def test_two_phase_and_nominal_dt_after_missed_slots():
    planner = make_planner()
    old = planner.last_accepted
    for _ in range(20):
        assert propose(planner, postfade=False) is None
        assert propose(planner, eligible=False) is None
    assert planner.last_accepted is old and planner.state == 'ready'
    proposal = propose(planner, now=.08, source_time=.08)
    assert planner.last_accepted is old and planner.state == 'ready'
    assert np.allclose(proposal.command.dq, -1 / 30)
    assert np.allclose(proposal.command.q, .1 - .5 / 30 ** 2)
    assert commit(planner, proposal)
    assert planner.last_accepted is proposal.command
    assert propose(planner, now=.09, source_time=.09) is None


@pytest.mark.parametrize('revocation', [{'postfade': False}, {'eligible': False}])
def test_active_authorization_revocation_latches_stop_without_a_hold(revocation):
    planner = make_planner()
    proposal = propose(planner)
    assert commit(planner, proposal)
    accepted = planner.last_accepted
    assert propose(planner, now=100., source_time=100., **revocation) is None
    assert planner.stop_requested and planner.last_accepted is accepted
    assert planner.reason == 'recovery_authorization_revoked'


@pytest.mark.parametrize('clock', [None, [], [1.], {}, True, 'bad'])
def test_malformed_commit_clock_invalidates_pending_transaction(clock):
    planner = make_planner()
    proposal = propose(planner)
    assert not commit(planner, proposal, now=clock)
    assert planner.stop_requested and planner.reason == 'malformed_commit_clock'
    assert not commit(planner, proposal)


def test_callback_sees_exact_candidate_and_inherited_velocity_slew():
    seen = []
    def validate(old, new):
        seen.append((old, new))
        return True
    seed = Command((.1,) * 4, (.05,) * 4, (.2,) * 4)
    planner = make_planner(seed=seed, validator=validate)
    proposal = propose(planner, tau=(.3,) * 4)
    assert seen == [(seed, proposal.command)]
    delta_v = np.array(proposal.command.dq) - seed.dq
    assert np.all(abs(delta_v) <= 1 / 30 + 1e-15)
    expected = np.array(seed.q) + .5 / 30 * (
        np.array(seed.dq) + proposal.command.dq)
    assert np.allclose(proposal.command.q, expected)
    assert proposal.command.tau == (.3,) * 4
    assert commit(planner, proposal)


@pytest.mark.parametrize('field', ['q', 'dq', 'tau'])
@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf])
def test_nonfinite_observation_latches_without_hold(field, bad):
    planner = make_planner()
    old = planner.last_accepted
    assert propose(planner, **{field: (bad, .1, .1, .1)}) is None
    assert planner.stop_requested and planner.last_accepted is old
    assert propose(planner) is None


@pytest.mark.parametrize('field', ['q', 'dq', 'tau'])
def test_nonfinite_seed_rejected(field):
    args = dict(q=(0.,) * 4, dq=(0.,) * 4, tau=(0.,) * 4)
    args[field] = (np.nan,) * 4
    with pytest.raises(ValueError):
        Command(**args)


@pytest.mark.parametrize('field', ['dt', 'lead', 'kp', 'acceleration'])
def test_nonfinite_bounds_rejected(field):
    value = np.nan if field == 'dt' else (np.nan,) * 4
    with pytest.raises(ValueError):
        replace(BoundValues.research_fixture(), **{field: value})


def test_collision_rejection_never_snaps_reference_by_point_zero_four():
    seen = []
    def collision(old, candidate):
        seen.append(candidate)
        return False
    planner = make_planner(validator=collision)
    old = planner.last_accepted
    assert propose(planner, q=(.061,) * 4) is None
    assert len(seen) == 1
    assert planner.reason == 'external_validation'
    assert planner.last_accepted is old
    assert propose(planner, q=(.061,) * 4) is None
    assert len(seen) == 1


def test_validator_exception_fails_closed():
    def broken(old, new):
        raise RuntimeError('collision service failed')
    planner = make_planner(validator=broken)
    assert propose(planner) is None
    assert planner.reason == 'proposal_error:RuntimeError'


def test_inherited_velocity_must_fit_lead_stopping_distance():
    seed = Command((.1,) * 4, (.20,) * 4, (0.,) * 4)
    planner = make_planner(seed=seed)
    assert propose(planner, q=(.075,) * 4) is None
    assert planner.reason == 'inherited_stopping_margin'
    assert planner.last_accepted is seed


def test_candidate_reserves_lead_margin_not_only_endpoint():
    seed = Command((.1,) * 4, (-.1,) * 4, (0.,) * 4)
    planner = make_planner(seed=seed)
    # inherited stopping endpoint fits; next endpoint fits but its stop does not
    assert propose(planner, q=(.130,) * 4) is None
    assert planner.reason == 'stopping_margin'


@pytest.mark.parametrize('change,reason', [
    ({'dq': (-3.,) * 4}, 'inherited_total_pd_ff_effort'),
    ({'tau': (100.,) * 4}, 'total_pd_ff_effort'),
    ({'q': (.7,) * 4}, 'inherited_measured_position_or_excursion'),
    ({'now': .2, 'source_time': 0.}, 'stale_or_reversed_clock'),
    ({'now': .2, 'source_time': .2}, 'stale_buffer'),
    ({'now': np.nan}, 'proposal_error:ValueError'),
    ({'source_time': .1}, 'stale_or_reversed_clock'),
    ({'trial_id': 'other'}, 'trial_or_mode_changed'),
    ({'mode_ack': False}, 'trial_or_mode_changed'),
])
def test_guards_no_command(change, reason):
    planner = make_planner()
    assert propose(planner, **change) is None
    assert planner.reason == reason


def test_old_source_rejected_even_with_fresh_host_clock():
    bounds = replace(BoundValues.research_fixture(), max_source_age=.1)
    planner = make_planner(bounds=bounds)
    first = propose(planner)
    assert commit(planner, first)
    assert propose(planner, now=2 / 30, source_time=1 / 30) is None
    assert planner.reason == 'stale_source'


def test_source_expiring_between_propose_and_commit_rejected():
    planner = make_planner()
    proposal = propose(planner, now=.05, source_time=.001)
    assert proposal is not None
    assert not commit(planner, proposal, now=.06)
    assert planner.reason == 'stale_commit'


@pytest.mark.parametrize('field', ['q', 'dq', 'tau'])
def test_mismatched_vector_size_fails_closed(field):
    planner = make_planner()
    assert propose(planner, **{field: (0.,) * 3}) is None
    assert planner.stop_requested


def test_inherited_position_bound_violation_and_velocity_violation():
    for seed, reason in [
        (Command((.59,) * 4, (0.,) * 4, (0.,) * 4),
         'inherited_position_or_excursion'),
        (Command((.1,) * 4, (.3,) * 4, (0.,) * 4), 'inherited_velocity'),
    ]:
        planner = make_planner(seed=seed)
        assert propose(planner) is None
        assert planner.reason == reason


def test_measured_velocity_and_ff_add_in_total_effort_guard():
    planner = make_planner()
    # neither term alone exceeds the 10.8 Nm elbow bound, but their sum does
    assert propose(planner, dq=(0., 0., 0., -.5),
                   tau=(0., 0., 0., 6.)) is None
    assert planner.reason == 'total_pd_ff_effort'


@pytest.mark.parametrize('failure', ['false', 'exception', 'clipped', 'late'])
def test_write_failures_latch_without_commit_or_automatic_rearm(failure):
    planner = make_planner()
    old = planner.last_accepted
    proposal = propose(planner)
    changes = {
        'false': dict(accepted=False),
        'exception': dict(accepted=False, error=RuntimeError('writer failed')),
        'clipped': dict(actual_buffer=Command((.1,) * 4, (0.,) * 4, (0.,) * 4)),
        'late': dict(now=.1),
    }[failure]
    assert not commit(planner, proposal, **changes)
    assert planner.last_accepted is old and planner.stop_requested
    assert propose(planner) is None
    assert not commit(planner, proposal)


def test_pending_proposal_identity_and_stop_invalidation():
    planner = make_planner()
    proposal = propose(planner)
    assert not commit(planner, replace(proposal))
    assert planner.stop_requested
    planner.reset_trial('two', (0.,) * 4, planner.last_accepted,
                        accepted_at=.1, mode_ack=True)
    proposal = propose(planner, now=.14, source_time=.14, trial_id='two')
    planner.stop()
    assert not commit(planner, proposal)
    with pytest.raises(RuntimeError, match='external stop'):
        planner.reset_trial('three', (0.,) * 4, planner.last_accepted,
                            accepted_at=.2, mode_ack=True)


def test_local_fault_reset_requires_new_trial_and_mode_ack():
    planner = make_planner()
    assert propose(planner, tau=(np.nan,) * 4) is None
    for trial_id, ack in [('one', True), ('two', False)]:
        with pytest.raises(ValueError):
            planner.reset_trial(trial_id, (0.,) * 4, planner.last_accepted,
                                accepted_at=.1, mode_ack=ack)
    assert planner.stop_requested
    planner.reset_trial('two', (0.,) * 4, planner.last_accepted,
                        accepted_at=.1, mode_ack=True)
    assert propose(planner, now=.14, source_time=.14, trial_id='two') is not None


def test_inherited_target_overshoot_stops_instead_of_zero_velocity_jump():
    seed = Command((.0001,) * 4, (-.1,) * 4, (0.,) * 4)
    planner = make_planner(seed=seed)
    assert propose(planner, q=seed.q) is None
    assert planner.reason == 'target_overshoot'
    assert planner.last_accepted is seed


def test_uncommitted_second_proposal_requests_stop():
    planner = make_planner()
    first = propose(planner)
    assert propose(planner) is None
    assert planner.reason == 'uncommitted_proposal'
    assert not commit(planner, first)


@pytest.mark.parametrize('sign', [-1, 1])
def test_offline_four_joint_coupled_inertia_symmetric_load(sign):
    """Synthetic gravity-compensated fixed-base PD plant, not model validation"""
    bounds = replace(BoundValues.research_fixture(), velocity=(.1,) * 4)
    planner = make_planner(bounds=bounds)
    inertia = np.diag([2., 1.5, 1., .8]) + .04 * np.ones((4, 4))
    assert np.min(np.linalg.eigvalsh(inertia)) > 0
    q, dq = np.full(4, .1), np.zeros(4)
    command = planner.last_accepted
    next_control = bounds.dt
    peak_lead = 0.
    for tick in range(1, 6001):
        now = tick * .001
        if now + 1e-12 >= next_control:
            proposal = planner.propose(q, dq, np.zeros(4), now=now,
                                       source_time=now, trial_id='one', postfade=True)
            assert proposal is not None, planner.reason
            assert commit(planner, proposal)
            command = proposal.command
            next_control = now + bounds.dt
        lead = np.array(command.q) - q
        effort = np.array(bounds.kp) * lead + np.array(bounds.kd) * (
            np.array(command.dq) - dq) + command.tau
        peak_lead = max(peak_lead, np.max(abs(lead)))
        assert np.all(abs(effort) <= bounds.effort)
        ddq = np.linalg.solve(inertia, effort + sign * np.ones(4) - 1. * dq)
        dq += .001 * ddq
        q += .001 * dq
    assert peak_lead < .04
    assert max(abs(q)) < .01
    assert max(abs(np.array(command.q))) < 1e-8
