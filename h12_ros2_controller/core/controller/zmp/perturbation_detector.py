from dataclasses import dataclass, field

import numpy as np


@dataclass
class PerturbationState:
    active: bool
    severity: float
    reasons: list[str] = field(default_factory=list)
    zmp_bad: bool = False
    motion_bad: bool = False
    raw_perturbed: bool = False


class PerturbationDetector:
    def __init__(self, config):
        zmp_cfg = config.get('zmp', {})
        thresholds = zmp_cfg.get('thresholds', {})
        hysteresis = zmp_cfg.get('hysteresis', {})

        self.zmp_error_threshold = float(thresholds.get('zmp_error', 0.02))
        self.support_margin_threshold = float(
            thresholds.get('support_margin', -np.inf)
        )
        self.center_shift_threshold = float(
            thresholds.get('center_shift', 0.03)
        )
        self.com_velocity_threshold = float(
            thresholds.get('com_velocity', 0.08)
        )
        self.angular_velocity_threshold = float(
            thresholds.get('angular_velocity', 0.15)
        )
        self.angular_acceleration_threshold = float(
            thresholds.get('angular_acceleration', 1.0)
        )
        self.force_proxy_threshold = float(
            thresholds.get('force_proxy', 20.0)
        )
        self.enter_cycles = max(1, int(hysteresis.get('enter_cycles', 2)))
        self.exit_cycles = max(1, int(hysteresis.get('exit_cycles', 5)))

        self._active = False
        self._enter_count = 0
        self._exit_count = 0

    def update(self, state):
        '''Apply zmp-and-motion trigger logic with enter/exit hysteresis'''
        zmp_norm = float(np.linalg.norm(state.zmp_error))
        center_norm = float(np.linalg.norm(state.center_shift))
        velocity_norm = float(np.linalg.norm(state.com_velocity))
        angular_velocity_norm = float(np.linalg.norm(state.angular_velocity))
        angular_acceleration_norm = float(
            np.linalg.norm(state.angular_acceleration)
        )

        zmp_reasons = []
        if zmp_norm >= self.zmp_error_threshold:
            zmp_reasons.append('zmp_error')
        if state.support_margin < self.support_margin_threshold:
            zmp_reasons.append('support_margin')

        motion_reasons = []
        motion_reasons += self._threshold_reason(
            center_norm,
            self.center_shift_threshold,
            'center_shift',
        )
        motion_reasons += self._threshold_reason(
            velocity_norm,
            self.com_velocity_threshold,
            'com_velocity',
        )
        motion_reasons += self._threshold_reason(
            angular_velocity_norm,
            self.angular_velocity_threshold,
            'angular_velocity',
        )
        motion_reasons += self._threshold_reason(
            angular_acceleration_norm,
            self.angular_acceleration_threshold,
            'angular_acceleration',
        )
        motion_reasons += self._threshold_reason(
            state.force_proxy,
            self.force_proxy_threshold,
            'force_proxy',
        )

        zmp_bad = bool(zmp_reasons)
        motion_bad = bool(motion_reasons)
        raw_perturbed = zmp_bad and motion_bad
        self._update_hysteresis(raw_perturbed)

        return PerturbationState(
            active=self._active,
            severity=self._severity(
                state,
                zmp_norm,
                center_norm,
                velocity_norm,
                angular_velocity_norm,
                angular_acceleration_norm,
            ),
            reasons=zmp_reasons + motion_reasons,
            zmp_bad=zmp_bad,
            motion_bad=motion_bad,
            raw_perturbed=raw_perturbed,
        )

    @property
    def entering(self):
        return self._enter_count > 0

    def _update_hysteresis(self, raw_perturbed):
        if raw_perturbed:
            self._enter_count += 1
            self._exit_count = 0
        else:
            self._exit_count += 1
            self._enter_count = 0

        if not self._active and self._enter_count >= self.enter_cycles:
            self._active = True
            self._exit_count = 0
        elif self._active and self._exit_count >= self.exit_cycles:
            self._active = False
            self._enter_count = 0

    def _severity(self,
                  state,
                  zmp_norm,
                  center_norm,
                  velocity_norm,
                  angular_velocity_norm,
                  angular_acceleration_norm):
        ratios = [
            self._ratio(zmp_norm, self.zmp_error_threshold),
            self._ratio(center_norm, self.center_shift_threshold),
            self._ratio(velocity_norm, self.com_velocity_threshold),
            self._ratio(
                angular_velocity_norm,
                self.angular_velocity_threshold,
            ),
            self._ratio(
                angular_acceleration_norm,
                self.angular_acceleration_threshold,
            ),
            self._ratio(state.force_proxy, self.force_proxy_threshold),
        ]
        return float(max(ratios))

    @staticmethod
    def _threshold_reason(value, threshold, reason):
        if threshold <= 0.0:
            return []
        if value >= threshold:
            return [reason]
        return []

    @staticmethod
    def _ratio(value, threshold):
        if threshold <= 0.0 or not np.isfinite(threshold):
            return 0.0
        return float(value / threshold)
