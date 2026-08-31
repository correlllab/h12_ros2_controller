from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class AffineResponseModel:
    '''Store one normalized affine closed-loop response model'''

    state_mean: np.ndarray
    state_scale: np.ndarray
    input_mean: np.ndarray
    input_scale: np.ndarray
    target_mean: np.ndarray
    target_scale: np.ndarray
    weights: np.ndarray
    bias: np.ndarray
    predicts_delta: bool = False

    def predict(self, state, model_input):
        '''Predict the next response state'''
        state = _matrix(state, self.state_mean.size, 'state')
        model_input = _matrix(
            model_input, self.input_mean.size, 'model_input',
        )
        state_norm = (state - self.state_mean) / self.state_scale
        input_norm = (
            (model_input - self.input_mean) / self.input_scale
            if self.input_mean.size else model_input
        )
        features = np.concatenate([state_norm, input_norm], axis=-1)
        target_norm = features @ self.weights.T + self.bias
        target = target_norm * self.target_scale + self.target_mean
        return state + target if self.predicts_delta else target

    def rollout(self, initial_state, model_inputs):
        '''Roll out one model-input sequence'''
        model_inputs = _matrix(
            model_inputs, self.input_mean.size, 'model_inputs',
        )
        state = _matrix(initial_state, self.state_mean.size, 'initial_state')
        if state.ndim != 1:
            raise ValueError('initial_state must be one-dimensional')
        values = [np.copy(state)]
        for model_input in model_inputs:
            state = self.predict(state, model_input)
            values.append(np.copy(state))
        return np.asarray(values)

    def to_dict(self):
        '''Return a JSON-compatible model mapping'''
        values = {
            name: getattr(self, name).tolist()
            for name in (
                'state_mean', 'state_scale', 'input_mean', 'input_scale',
                'target_mean', 'target_scale', 'weights', 'bias',
            )
        }
        values['predicts_delta'] = self.predicts_delta
        return values

    @classmethod
    def from_dict(cls, value):
        '''Load a model from a serialized mapping'''
        values = {
            name: np.asarray(value[name], dtype=np.float64)
            for name in (
                'state_mean', 'state_scale', 'input_mean', 'input_scale',
                'target_mean', 'target_scale', 'weights', 'bias',
            )
        }
        values['predicts_delta'] = bool(value.get('predicts_delta', False))
        return cls(**values)


def fit_affine_response(
        states, model_inputs, next_states, ridge=1e-4,
        predict_delta=False):
    '''Fit one normalized affine response model with ridge regularization'''
    states = _samples(states, 'states')
    next_states = _samples(next_states, 'next_states')
    model_inputs = np.asarray(model_inputs, dtype=np.float64)
    if model_inputs.ndim != 2 or model_inputs.shape[0] != states.shape[0]:
        raise ValueError('model_inputs must align with state samples')
    if next_states.shape != states.shape:
        raise ValueError('next_states must match states')
    if not np.all(np.isfinite(model_inputs)):
        raise ValueError('model_inputs must be finite')
    ridge = float(ridge)
    if not np.isfinite(ridge) or ridge < 0.0:
        raise ValueError('ridge must be finite and nonnegative')

    state_mean, state_scale = _normalization(states)
    input_mean, input_scale = _normalization(model_inputs)
    target_values = next_states - states if predict_delta else next_states
    target_mean, target_scale = _normalization(target_values)
    state_norm = (states - state_mean) / state_scale
    input_norm = (
        (model_inputs - input_mean) / input_scale
        if model_inputs.shape[1] else model_inputs
    )
    target_norm = (target_values - target_mean) / target_scale
    features = np.concatenate([
        state_norm,
        input_norm,
        np.ones((len(states), 1)),
    ], axis=1)
    regularization = ridge * np.eye(features.shape[1])
    regularization[-1, -1] = 0.0
    coefficients = np.linalg.solve(
        features.T @ features + regularization,
        features.T @ target_norm,
    ).T
    return AffineResponseModel(
        state_mean=state_mean,
        state_scale=state_scale,
        input_mean=input_mean,
        input_scale=input_scale,
        target_mean=target_mean,
        target_scale=target_scale,
        weights=coefficients[:, :-1],
        bias=coefficients[:, -1],
        predicts_delta=bool(predict_delta),
    )


def response_metrics(expected, predicted):
    '''Return per-axis R2 and normalized RMSE'''
    expected = _samples(expected, 'expected')
    predicted = _samples(predicted, 'predicted')
    if expected.shape != predicted.shape:
        raise ValueError('metric samples must have matching shapes')
    residual = predicted - expected
    variance = np.sum(
        (expected - np.mean(expected, axis=0)) ** 2,
        axis=0,
    )
    squared_error = np.sum(residual ** 2, axis=0)
    r2 = 1.0 - squared_error / np.maximum(variance, 1e-12)
    nrmse = np.sqrt(np.mean(residual ** 2, axis=0)) / np.maximum(
        np.std(expected, axis=0), 1e-6,
    )
    return {
        'r2': r2,
        'rmse': np.sqrt(np.mean(residual ** 2, axis=0)),
        'nrmse': nrmse,
    }


def _normalization(values):
    if values.shape[1] == 0:
        return np.zeros(0), np.ones(0)
    mean = np.mean(values, axis=0)
    scale = np.std(values, axis=0)
    scale[scale < 1e-6] = 1.0
    return mean, scale


def _samples(value, name):
    result = np.asarray(value, dtype=np.float64)
    if result.ndim != 2 or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must be a finite sample matrix')
    return result


def _matrix(value, width, name):
    result = np.asarray(value, dtype=np.float64)
    if (
        result.ndim not in (1, 2)
        or result.shape[-1] != width
        or not np.all(np.isfinite(result))
    ):
        raise ValueError(f'{name} must be finite with final width {width}')
    return result
