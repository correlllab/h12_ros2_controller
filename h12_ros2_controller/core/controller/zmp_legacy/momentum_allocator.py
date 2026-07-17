from dataclasses import dataclass

import numpy as np


@dataclass
class ArmMomentumTarget:
    arm: str
    target_momentum: np.ndarray


class MomentumAllocator:
    def __init__(self, config):
        allocation = config.get('zmp', {}).get('allocation', {})
        self.weights = {
            'left': float(allocation.get('left_weight', 0.5)),
            'right': float(allocation.get('right_weight', 0.5)),
        }

    def allocate(self, target_momentum, active_arms):
        '''Split whole-body angular momentum across available arms'''
        target_momentum = np.asarray(target_momentum, dtype=np.float64)
        active_arms = list(active_arms)
        if not active_arms:
            return []
        total_weight = sum(
            max(0.0, self.weights.get(arm, 0.0))
            for arm in active_arms
        )
        if total_weight <= 0.0:
            total_weight = float(len(active_arms))
            weights = {arm: 1.0 for arm in active_arms}
        else:
            weights = self.weights

        return [
            ArmMomentumTarget(
                arm=arm,
                target_momentum=np.copy(
                    target_momentum * max(0.0, weights.get(arm, 0.0))
                    / total_weight
                ),
            )
            for arm in active_arms
        ]
