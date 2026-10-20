import numpy as np

from h12_ros2_controller.core.support_region import (
    signed_support_margin,
    support_rectangle,
)


def _transform(x=0.0, y=0.0, yaw=0.0):
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    transform = np.eye(4)
    transform[:2, :2] = [[cosine, -sine], [sine, cosine]]
    transform[:2, 3] = [x, y]
    return transform


def test_aligned_feet_produce_expected_enclosing_rectangle():
    rectangle = support_rectangle(
        _transform(y=0.16),
        _transform(y=-0.16),
    )

    assert rectangle.valid
    assert np.allclose(rectangle.center, [0.044, 0.0])
    assert np.allclose(rectangle.forward_axis, [1.0, 0.0])
    assert np.allclose(rectangle.lateral_axis, [0.0, 1.0])
    assert np.allclose(rectangle.half_extents, [0.13, 0.203])


def test_mirrored_frame_axis_is_aligned_before_averaging():
    rectangle = support_rectangle(
        _transform(y=0.16),
        _transform(y=-0.16, yaw=np.pi),
    )

    assert rectangle.valid
    assert np.allclose(rectangle.forward_axis, [1.0, 0.0])
    assert np.all(np.isfinite(rectangle.half_extents))


def test_excessive_yaw_divergence_is_invalid():
    rectangle = support_rectangle(
        _transform(y=0.16, yaw=-0.25),
        _transform(y=-0.16, yaw=0.25),
        max_yaw_divergence=0.349,
    )

    assert not rectangle.valid
    assert 'yaw divergence' in rectangle.invalid_reason


def test_missing_degenerate_and_nonfinite_frames_are_invalid():
    assert not support_rectangle(None, np.eye(4)).valid

    degenerate = np.eye(4)
    degenerate[:2, 0] = 0.0
    assert not support_rectangle(degenerate, np.eye(4)).valid

    nonfinite = np.eye(4)
    nonfinite[0, 0] = np.nan
    assert not support_rectangle(nonfinite, np.eye(4)).valid


def test_signed_margin_is_positive_inside_and_negative_outside():
    rectangle = support_rectangle(
        _transform(y=0.16),
        _transform(y=-0.16),
    )

    assert np.isclose(
        signed_support_margin(rectangle.center, rectangle),
        0.13,
    )
    outside = rectangle.center + 0.213 * rectangle.lateral_axis
    assert np.isclose(signed_support_margin(outside, rectangle), -0.01)
    assert np.isnan(signed_support_margin([np.nan, 0.0], rectangle))
