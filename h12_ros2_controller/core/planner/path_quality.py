'''Bounded whole-arm path preferences independent of planner backend'''

import numpy as np


def radial_excursions(landmarks, torso_positions, slack, torso_rotations=None):
    '''Measure torso-relative extension beyond each landmark's endpoints'''
    torso = np.asarray(torso_positions, dtype=float)
    rotations = (None if torso_rotations is None else
                 np.asarray(torso_rotations, dtype=float))
    result = {}
    for name, points in landmarks.items():
        points = np.asarray(points, dtype=float)
        relative = points - torso
        if rotations is not None:
            relative = np.einsum('nji,nj->ni', rotations, relative)
        radii = np.linalg.norm(relative[:, :2], axis=1)
        # requested endpoints define the necessary reach, not an absolute box
        comfort_limit = max(radii[0], radii[-1]) + slack
        result[name] = float(np.max(np.maximum(0.0, radii - comfort_limit)))
    return result


def candidate_path_score(metadata, ik_score, config):
    '''Rank already-valid paths without trading against hard constraints'''
    hand_deviation = max(
        metadata.get('hand_segment_deviation', {}).values(), default=0.0,
    )
    compact_excursion = max(
        metadata.get('compact_radial_excess', {}).values(), default=0.0,
    )
    clearance = metadata.get('minimum_self_clearance')
    clearance_shortfall = (
        max(0.0, config.preferred_clearance - clearance)
        if clearance is not None else 0.0
    )
    return (
        metadata['joint_path_length'] +
        config.candidate_ik_weight * (ik_score or 0.0) +
        config.candidate_hand_weight * hand_deviation +
        config.candidate_compact_weight * compact_excursion +
        config.candidate_clearance_weight * clearance_shortfall +
        config.candidate_wrist_path_weight * metadata.get('excess_wrist_travel', 0.0) +
        config.candidate_duration_weight * metadata.get('duration', 0.0)
    )
