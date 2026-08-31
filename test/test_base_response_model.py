import numpy as np

from h12_ros2_controller.core.controller.counter_balance.base_response_model import (
    AffineResponseModel,
    fit_affine_response,
    response_metrics,
)


def test_affine_response_fit_recovers_linear_dynamics():
    rng = np.random.default_rng(4)
    states = rng.normal(size=(500, 2))
    inputs = rng.normal(size=(500, 1))
    next_states = states @ np.array([[0.9, 0.1], [-0.2, 0.8]]).T
    next_states += inputs @ np.array([[0.3, -0.1]])
    next_states += np.array([0.05, -0.02])

    model = fit_affine_response(states, inputs, next_states)
    predicted = model.predict(states, inputs)
    metrics = response_metrics(next_states, predicted)

    assert np.all(metrics['r2'] > 0.999)
    assert np.all(metrics['nrmse'] < 0.01)


def test_affine_response_model_round_trips_serialization():
    model = fit_affine_response(
        np.array([[0.0], [1.0], [2.0]]),
        np.zeros((3, 0)),
        np.array([[1.0], [2.0], [3.0]]),
    )

    loaded = AffineResponseModel.from_dict(model.to_dict())

    assert np.allclose(loaded.predict([3.0], []), [4.0], atol=1e-3)


def test_affine_response_rollout_uses_each_input():
    states = np.array([[0.0], [0.0], [1.0], [1.0]])
    inputs = np.array([[0.0], [1.0], [0.0], [1.0]])
    next_states = inputs.copy()
    model = fit_affine_response(states, inputs, next_states)

    rollout = model.rollout([0.0], [[1.0], [0.0]])

    assert rollout.shape == (3, 1)
    assert rollout[1, 0] > 0.9
    assert abs(rollout[2, 0]) < 0.1


def test_delta_response_model_predicts_next_state():
    states = np.array([[0.0], [1.0], [2.0], [3.0]])
    inputs = np.zeros((4, 0))
    next_states = states + 0.5

    model = fit_affine_response(
        states, inputs, next_states, predict_delta=True,
    )

    assert model.predicts_delta
    assert np.allclose(model.predict([[4.0]], np.zeros((1, 0))), [[4.5]])
