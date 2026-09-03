import numpy as np
import pytest

from h12_ros2_controller.core.controller.counter_balance \
    import residual_response_model as response_model


N5Model = response_model.N5Model
R5Model = response_model.R5Model
ContextualR5DiagonalModel = response_model.ContextualR5DiagonalModel
ContextualR5Model = response_model.ContextualR5Model
U5Model = response_model.U5Model
U5State = response_model.U5State
ContextualU5Model = response_model.ContextualU5Model
N5ValidityThresholds = response_model.N5ValidityThresholds
DistinguishabilityThresholds = response_model.DistinguishabilityThresholds
action_ranking_metrics = response_model.action_ranking_metrics
calibrate_n5_validity = response_model.calibrate_n5_validity
fit_r5 = response_model.fit_r5
fit_n5_momentum_rate = response_model.fit_n5_momentum_rate
fit_contextual_r5_diagonal = response_model.fit_contextual_r5_diagonal
fit_contextual_r5 = response_model.fit_contextual_r5
fit_u5 = response_model.fit_u5
fit_contextual_u5 = response_model.fit_contextual_u5
predict_residual_response = response_model.predict_residual_response
response_validation_metrics = response_model.response_validation_metrics
n5_no_crossing_confidence = response_model.n5_no_crossing_confidence


def test_u5_predicts_delay_gain_carryover_and_momentum():
    model = U5Model(
        delay=np.array([0, 1, 0, 1]),
        gain=np.array([0.5, 0.6, -0.4, 0.8]),
        carryover=np.array([0.2, 0.3, 0.0, -0.1]),
    )
    state = U5State(
        previous_realized_velocity=np.array([1.0, 2.0, 3.0, 4.0]),
        pending_request=np.array([5.0, 6.0, 7.0, 8.0]),
    )
    requested = np.array([
        [0.4, 0.5, 0.6, 0.7],
        [0.8, 0.9, 1.0, 1.1],
    ])
    momentum_map = np.array([
        [1.0, 2.0, 0.0, -1.0],
        [0.0, 0.5, 1.5, 0.0],
    ])

    prediction = model.predict(requested, momentum_map, state)

    expected_h1 = model.carryover * state.previous_realized_velocity
    expected_h1 += model.gain * np.array([0.4, 6.0, 0.6, 8.0])
    expected_h2 = model.carryover * expected_h1
    expected_h2 += model.gain * np.array([0.8, 0.5, 1.0, 0.7])
    expected = np.vstack([expected_h1, expected_h2])
    assert np.allclose(prediction.realized_velocity, expected)
    assert np.allclose(
        prediction.residual_momentum, expected @ momentum_map.T,
    )
    assert np.array_equal(prediction.final_state.pending_request, requested[1])
    assert np.allclose(
        prediction.final_state.previous_realized_velocity, expected_h2,
    )


def test_u5_zero_request_has_exact_zero_response_from_zero_state():
    model = U5Model(
        delay=np.array([0, 1, 1, 0]),
        gain=np.ones(4),
        carryover=np.full(4, 0.7),
    )

    prediction = model.predict(np.zeros((3, 4)), np.ones((2, 4)))

    assert np.array_equal(prediction.realized_velocity, np.zeros((3, 4)))
    assert np.array_equal(prediction.residual_momentum, np.zeros((3, 2)))


def test_fit_u5_recovers_per_axis_delay_gain_and_carryover():
    rng = np.random.default_rng(41)
    sample_count = 800
    delay = np.array([0, 1, 1, 0])
    gain = np.array([0.7, -0.4, 1.1, 0.25])
    carryover = np.array([0.15, 0.5, -0.2, 0.75])
    requested = rng.normal(size=(sample_count, 4))
    previous_request = rng.normal(size=(sample_count, 4))
    previous_realized = rng.normal(size=(sample_count, 4))
    effective = np.where(delay == 0, requested, previous_request)
    realized = gain * effective + carryover * previous_realized

    model = fit_u5(
        requested,
        previous_request,
        previous_realized,
        realized,
        ridge=1e-12,
    )

    assert np.array_equal(model.delay, delay)
    assert np.allclose(model.gain, gain, atol=1e-10)
    assert np.allclose(model.carryover, carryover, atol=1e-10)
    assert np.allclose(
        model.predict_aligned(
            requested, previous_request, previous_realized,
        ),
        realized,
    )


def test_fit_u5_honors_predeclared_delay():
    requested = np.arange(24, dtype=np.float64).reshape(6, 4) / 10.0
    previous_request = requested + 0.3
    previous_realized = requested - 0.2
    delay = np.ones(4, dtype=np.int64)
    realized = 0.5 * previous_request + 0.2 * previous_realized

    model = fit_u5(
        requested,
        previous_request,
        previous_realized,
        realized,
        delay=delay,
    )

    assert np.array_equal(model.delay, delay)
    assert np.allclose(
        model.predict_aligned(
            requested, previous_request, previous_realized,
        ),
        realized,
        atol=1e-7,
    )


def test_contextual_u5_recovers_joint_state_gains_and_masks_weak_axis():
    rng = np.random.default_rng(26)
    samples = 500
    requested = rng.normal(size=(samples, 4))
    q = rng.normal(scale=0.2, size=(samples, 4))
    dq = rng.normal(scale=0.3, size=(samples, 4))
    side = rng.choice([-1.0, 1.0], size=samples)
    gain = 0.5 + 0.1 * q - 0.05 * dq + 0.02 * side[:, None]
    realized = requested * gain

    model = fit_contextual_u5(
        requested,
        realized,
        q,
        dq,
        side,
        weak_direction_mask=np.array([False, False, True, False]),
        ridge=1e-10,
    )
    predicted = model.predict(requested, q, dq, side)

    assert isinstance(model, ContextualU5Model)
    assert np.allclose(predicted[:, [0, 1, 3]], realized[:, [0, 1, 3]])
    assert np.array_equal(predicted[:, 2], np.zeros(samples))


def test_r5_implements_documented_h1_h2_fir():
    model = R5Model(
        g0=np.array([[1.0, 2.0], [-0.5, 0.25]]),
        g1=np.array([[0.3, -0.1], [1.5, 0.4]]),
    )
    momentum = np.array([[0.5, -0.2], [0.7, 0.4]])

    predicted = model.predict(momentum)

    assert np.allclose(predicted[0], model.g0 @ momentum[0])
    assert np.allclose(
        predicted[1], model.g1 @ momentum[0] + model.g0 @ momentum[1],
    )
    batched = model.predict(np.stack([momentum, momentum]))
    assert np.allclose(batched, predicted)


def test_fit_r5_recovers_zero_intercept_gains():
    rng = np.random.default_rng(17)
    momentum = rng.normal(size=(700, 2, 2))
    expected = R5Model(
        g0=np.array([[0.8, -0.3], [0.2, 1.1]]),
        g1=np.array([[-0.4, 0.6], [0.9, 0.1]]),
    )
    rate = expected.predict(momentum)

    fitted = fit_r5(momentum, rate, ridge=1e-12)

    assert np.allclose(fitted.g0, expected.g0, atol=1e-10)
    assert np.allclose(fitted.g1, expected.g1, atol=1e-10)
    assert np.allclose(fitted.predict(momentum), rate)


def test_fit_r5_diagonal_ignores_unverified_cross_axis_terms():
    rng = np.random.default_rng(23)
    momentum = rng.normal(size=(400, 2, 2))
    expected = R5Model(
        g0=np.diag([-0.2, -0.5]),
        g1=np.diag([0.1, 0.3]),
    )
    rate = expected.predict(momentum)

    fitted = fit_r5(momentum, rate, ridge=1e-12, diagonal=True)

    assert np.allclose(fitted.g0, expected.g0, atol=1e-10)
    assert np.allclose(fitted.g1, expected.g1, atol=1e-10)


def test_contextual_r5_recovers_bilinear_diagonal_response():
    rng = np.random.default_rng(24)
    samples = 500
    tilt = rng.normal(scale=0.05, size=(samples, 2))
    rate = rng.normal(scale=0.1, size=(samples, 2))
    context = np.column_stack([tilt, rate])
    center = np.mean(context, axis=0)
    scale = np.std(context, axis=0)
    coefficients = np.array([
        [-0.2, 0.1, -0.05, 0.04, 0.03],
        [-0.5, -0.02, 0.08, 0.05, -0.04],
    ])
    gain = np.column_stack([
        np.ones(samples), (context - center) / scale,
    ]) @ coefficients.T
    momentum = rng.normal(scale=0.1, size=(samples, 2, 2))
    response = momentum * gain[:, None, :]

    fitted = fit_contextual_r5_diagonal(
        momentum, response, tilt, rate, ridge=1e-10,
    )

    assert isinstance(fitted, ContextualR5DiagonalModel)
    assert np.allclose(fitted.predict(momentum, tilt, rate), response)


def test_contextual_full_r5_recovers_cross_axis_response():
    rng = np.random.default_rng(25)
    samples = 600
    context = rng.normal(size=(samples, 3))
    center = np.mean(context, axis=0)
    scale = np.std(context, axis=0)
    coefficients = rng.normal(scale=0.2, size=(2, 2, 4))
    features = np.column_stack([
        np.ones(samples), (context - center) / scale,
    ])
    gains = np.einsum('nf,oif->noi', features, coefficients)
    momentum = rng.normal(scale=0.1, size=(samples, 2, 2))
    response = np.einsum('noi,nti->nto', gains, momentum)

    fitted = fit_contextual_r5(
        momentum, response, context, ridge=1e-10,
    )

    assert isinstance(fitted, ContextualR5Model)
    assert np.allclose(fitted.predict(momentum, context), response)


def test_composed_prediction_uses_structured_trapezoidal_integration():
    u5 = U5Model(np.zeros(4, dtype=np.int64), np.ones(4), np.zeros(4))
    r5 = R5Model(np.eye(2), np.zeros((2, 2)))
    state = U5State(
        previous_realized_velocity=np.array([0.2, -0.1, 0.3, 0.0]),
        pending_request=np.zeros(4),
    )
    requested = np.array([
        [1.0, 2.0, 3.0, 4.0],
        [2.0, 3.0, 4.0, 5.0],
    ])
    momentum_map = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
    ])
    dt = 0.02

    prediction = predict_residual_response(
        u5, r5, requested, momentum_map, dt, state,
    )

    expected_rate = requested[:, :2]
    expected_tilt_h1 = 0.5 * dt * expected_rate[0]
    expected_tilt_h2 = expected_tilt_h1 + 0.5 * dt * (
        expected_rate[0] + expected_rate[1]
    )
    expected_position_h1 = 0.5 * dt * (
        state.previous_realized_velocity + requested[0]
    )
    expected_position_h2 = expected_position_h1 + 0.5 * dt * (
        requested[0] + requested[1]
    )
    assert np.allclose(prediction.incremental_angular_rate, expected_rate)
    assert np.allclose(
        prediction.incremental_tilt,
        np.vstack([expected_tilt_h1, expected_tilt_h2]),
    )
    assert np.allclose(
        prediction.incremental_counter_position,
        np.vstack([expected_position_h1, expected_position_h2]),
    )


def test_composed_u5_r5_jacobians_match_finite_differences():
    u5 = U5Model(
        delay=np.array([0, 1, 0, 1]),
        gain=np.array([0.6, -0.4, 0.9, 0.3]),
        carryover=np.array([0.2, 0.5, -0.1, 0.7]),
    )
    r5 = R5Model(
        g0=np.array([[0.8, -0.25], [0.15, 0.6]]),
        g1=np.array([[-0.4, 0.3], [0.7, 0.1]]),
    )
    state = U5State(
        previous_realized_velocity=np.array([0.1, -0.2, 0.3, -0.4]),
        pending_request=np.array([-0.3, 0.2, 0.1, 0.5]),
    )
    momentum_map = np.array([
        [0.8, -0.2, 0.5, 0.1],
        [-0.1, 0.7, 0.3, -0.4],
    ])
    requested = np.array([
        [0.2, -0.1, 0.4, -0.3],
        [-0.5, 0.6, -0.2, 0.7],
    ])
    dt = 0.017
    prediction = predict_residual_response(
        u5, r5, requested, momentum_map, dt, state,
    )
    outputs = (
        ('realized_velocity', 'velocity_jacobian'),
        ('residual_momentum', 'momentum_jacobian'),
        ('incremental_angular_rate', 'angular_rate_jacobian'),
        ('incremental_tilt', 'tilt_jacobian'),
        ('incremental_counter_position', 'counter_position_jacobian'),
    )
    epsilon = 1e-7

    for request_step in range(2):
        for request_axis in range(4):
            plus = requested.copy()
            minus = requested.copy()
            plus[request_step, request_axis] += epsilon
            minus[request_step, request_axis] -= epsilon
            plus_prediction = predict_residual_response(
                u5, r5, plus, momentum_map, dt, state,
            )
            minus_prediction = predict_residual_response(
                u5, r5, minus, momentum_map, dt, state,
            )
            for value_name, jacobian_name in outputs:
                finite_difference = (
                    getattr(plus_prediction, value_name)
                    - getattr(minus_prediction, value_name)
                ) / (2.0 * epsilon)
                analytic = getattr(prediction, jacobian_name)[
                    :, :, request_step, request_axis
                ]
                assert np.allclose(
                    analytic, finite_difference, atol=2e-9, rtol=2e-7,
                ), value_name


def test_n5_predicts_phase_and_enforces_validity_thresholds():
    thresholds = N5ValidityThresholds(
        max_abs_tilt=np.array([0.5, 0.5]),
        max_abs_rate=np.array([2.0, 2.0]),
        max_tilt_step=np.array([0.1, 0.1]),
        max_rate_step=np.array([0.5, 0.5]),
        max_integration_error=np.array([1e-5, 1e-5]),
        min_dt=0.019,
        max_dt=0.021,
        sign_deadband=np.array([1e-6, 1e-6]),
    )
    model = N5Model(np.array([0.5, 0.25]), thresholds)
    dt = 0.02
    previous_rate = np.array([0.2, -0.4])
    current_rate = np.array([0.3, -0.2])
    current_tilt = np.array([0.1, -0.08])
    previous_tilt = current_tilt - 0.5 * dt * (
        previous_rate + current_rate
    )

    prediction = model.predict(
        current_tilt,
        previous_tilt,
        current_rate,
        previous_rate,
        dt,
    )

    expected_h1_rate = np.array([0.35, -0.15])
    expected_h2_rate = np.array([0.375, -0.1375])
    assert prediction.valid
    assert prediction.invalid_reasons == ()
    assert np.allclose(
        prediction.angular_rate,
        np.vstack([expected_h1_rate, expected_h2_rate]),
    )
    assert np.array_equal(prediction.diverging, np.ones((2, 2), dtype=bool))

    invalid = model.predict(
        current_tilt,
        previous_tilt,
        np.array([1.0, -0.2]),
        previous_rate,
        dt,
    )
    assert not invalid.valid
    assert 'rate_step' in invalid.invalid_reasons
    assert 'integration' in invalid.invalid_reasons


def test_n5_fits_and_uses_known_momentum_changes():
    rng = np.random.default_rng(12)
    samples = 200
    previous = rng.normal(scale=0.1, size=(samples, 2))
    current = previous + rng.normal(scale=0.02, size=(samples, 2))
    moving = rng.normal(scale=0.04, size=(samples, 2))
    nominal = rng.normal(scale=0.03, size=(samples, 2))
    trend = np.array([0.4, 0.7])
    moving_gain = np.array([[0.2, -0.1], [0.05, 0.3]])
    nominal_gain = np.array([[-0.15, 0.07], [0.1, -0.2]])
    following = (
        current
        + trend * (current - previous)
        + moving @ moving_gain.T
        + nominal @ nominal_gain.T
    )
    thresholds = N5ValidityThresholds(
        max_abs_tilt=np.ones(2),
        max_abs_rate=np.ones(2),
        max_tilt_step=np.ones(2),
        max_rate_step=np.ones(2),
        max_integration_error=np.ones(2),
        min_dt=0.01,
        max_dt=0.03,
        sign_deadband=np.zeros(2),
    )

    model = fit_n5_momentum_rate(
        previous,
        current,
        following,
        moving,
        nominal,
        thresholds,
        ridge=1e-12,
    )

    assert np.allclose(model.rate_trend_gain, trend, atol=1e-9)
    assert np.allclose(model.moving_momentum_gain, moving_gain, atol=1e-9)
    assert np.allclose(model.nominal_momentum_gain, nominal_gain, atol=1e-9)
    prediction = model.predict(
        np.zeros(2),
        np.zeros(2),
        current[0],
        previous[0],
        0.02,
        moving_momentum_change=np.vstack([moving[0], np.zeros(2)]),
        nominal_momentum_change=np.vstack([nominal[0], np.zeros(2)]),
    )
    assert np.allclose(prediction.angular_rate[0], following[0])


def test_calibrate_n5_validity_contains_calibration_samples():
    dt = np.array([0.019, 0.020, 0.021, 0.020])
    previous_rate = np.array([
        [0.1, -0.2], [0.2, -0.1], [0.3, 0.0], [0.1, 0.1],
    ])
    current_rate = previous_rate + np.array([
        [0.01, 0.02], [-0.01, 0.01], [0.02, -0.01], [0.0, 0.01],
    ])
    previous_tilt = np.array([
        [0.01, -0.02], [0.02, -0.01], [0.03, 0.0], [0.01, 0.01],
    ])
    current_tilt = previous_tilt + 0.5 * dt[:, None] * (
        previous_rate + current_rate
    )

    thresholds = calibrate_n5_validity(
        current_tilt,
        previous_tilt,
        current_rate,
        previous_rate,
        dt,
        quantile=1.0,
        margin=1.1,
        sign_deadband=np.array([1e-4, 1e-4]),
    )
    model = N5Model(np.array([0.5, 0.5]), thresholds)

    for sample in range(len(dt)):
        prediction = model.predict(
            current_tilt[sample],
            previous_tilt[sample],
            current_rate[sample],
            previous_rate[sample],
            dt[sample],
        )
        assert prediction.valid


def test_response_metrics_report_sign_and_baseline_improvement():
    expected = np.array([
        [1.0, -2.0], [2.0, -1.0], [-1.0, 2.0], [-2.0, 1.0],
    ])
    predicted = expected * 0.9
    reference = expected * -0.5

    metrics = response_validation_metrics(
        expected, predicted, reference, sign_threshold=0.1,
    )

    assert np.all(metrics.sign_accuracy == 1.0)
    assert np.all(metrics.zero_baseline_improvement > 0.98)
    assert np.all(metrics.reference_baseline_improvement > 0.99)

    zero_prediction = response_validation_metrics(
        expected, np.zeros_like(expected), sign_threshold=0.1,
    )
    assert np.all(zero_prediction.sign_accuracy == 0.0)


def test_n5_no_crossing_confidence_abstains_on_uncertain_phase():
    confidence = n5_no_crossing_confidence(
        current_rate=np.array([0.2, 0.03]),
        previous_rate=np.array([0.18, -0.02]),
        error_bound=np.array([0.02, 0.01]),
        horizon_steps=2,
    )

    assert np.array_equal(confidence, [True, False])


def test_action_ranking_uses_fixed_noise_effect_and_confidence_gates():
    measured_scores = np.array([[0.0, 1.0, 2.0]])
    predicted_scores = np.array([[0.0, 2.0, 1.5]])
    measured_responses = np.array([[
        [0.0, 0.0],
        [1.0, 0.0],
        [3.0, 0.0],
    ]])
    thresholds = DistinguishabilityThresholds(
        noise_bound=0.5,
        minimum_effect=0.75,
    )

    first = action_ranking_metrics(
        measured_scores,
        predicted_scores,
        measured_responses,
        thresholds,
        weights=np.array([1.0, 2.0]),
        confidence_radius=np.full((1, 3), 0.2),
    )
    second = action_ranking_metrics(
        measured_scores,
        predicted_scores,
        measured_responses,
        thresholds,
        weights=np.array([1.0, 2.0]),
        confidence_radius=np.full((1, 3), 0.2),
    )

    assert first.total_pairs == 3
    assert first.distinguishable_pairs == 2
    assert first.correct_pairs == 1
    assert first.coverage == pytest.approx(2.0 / 3.0)
    assert first.accuracy == 0.5
    assert np.array_equal(
        first.distinguishable_mask, second.distinguishable_mask,
    )
    assert np.array_equal(first.correct_mask, second.correct_mask)


@pytest.mark.parametrize(
    'operation',
    [
        lambda: U5Model(np.array([0, 1, 2, 0]), np.ones(4), np.zeros(4)),
        lambda: U5Model(np.zeros(4, dtype=int), np.ones(3), np.zeros(4)),
        lambda: R5Model(np.eye(3), np.eye(2)),
        lambda: R5Model(np.eye(2), np.array([[np.nan, 0.0], [0.0, 1.0]])),
        lambda: U5State(np.zeros(4), np.zeros(3)),
        lambda: DistinguishabilityThresholds(-1.0, 0.0),
    ],
)
def test_model_dataclasses_reject_invalid_values(operation):
    with pytest.raises(ValueError):
        operation()


def test_prediction_and_fitting_reject_invalid_shapes_and_nonfinite_values():
    u5 = U5Model(np.zeros(4, dtype=int), np.ones(4), np.zeros(4))
    r5 = R5Model(np.eye(2), np.eye(2))

    with pytest.raises(ValueError):
        u5.predict(np.zeros((2, 3)), np.zeros((2, 4)))
    with pytest.raises(ValueError):
        u5.predict(np.zeros((2, 4)), np.zeros((4, 2)))
    with pytest.raises(ValueError):
        r5.predict(np.array([[0.0, np.inf], [0.0, 0.0]]))
    with pytest.raises(ValueError):
        fit_r5(np.zeros((1, 2, 2)), np.zeros((1, 2, 2)))
    with pytest.raises(ValueError):
        predict_residual_response(
            u5, r5, np.zeros((3, 4)), np.zeros((2, 4)), 0.02,
        )
    with pytest.raises(ValueError):
        predict_residual_response(
            u5, r5, np.zeros((2, 4)), np.zeros((2, 4)), 0.0,
        )
