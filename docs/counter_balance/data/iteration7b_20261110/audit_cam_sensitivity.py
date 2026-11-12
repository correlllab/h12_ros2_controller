"""Reproduce conditional angular CAM sensitivities after audit_mechanism.py

Run from the benchmark root with its .venv Python. Defaults consume the temporary
outputs of audit_mechanism.py; --verify-frozen compares every retained result.
No contact truth or physical counterfactual is available from this calculation.
"""

import sys
import json
import argparse
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation, Slerp

from audit_mechanism import vrms, digest


def main(args):
    root = args.root.resolve()
    ctrl = root / 'submodules/h12_ros2_controller'
    sys.path.insert(0, str(ctrl))
    from h12_ros2_controller.utility.joint_definition import BODY_JOINTS
    model = pin.buildModelFromUrdf(str(ctrl / 'submodules/CL_Assets/ros_assets/h1_2_magpie.urdf'), pin.JointModelFreeFlyer())
    frozen = [i for i, n in enumerate(model.names) if i > 1 and n not in BODY_JOINTS]
    model = pin.buildReducedModel(model, frozen, pin.neutral(model))
    data = model.createData()
    qi = np.array([model.joints[model.getJointId(n)].idx_q for n in BODY_JOINTS])
    vi = np.array([model.joints[model.getJointId(n)].idx_v for n in BODY_JOINTS])
    traces = json.loads(args.traces.read_text())
    audit = json.loads(args.audit.read_text())
    assert len(traces) == 12 and traces.keys() == audit['runs'].keys()
    out = {'definition': 'Sensitivity only: 30Hz uniform grids, SG polynomial2 windows5/9 (133/267ms endpoint spans). Alternative root from quaternion derivative and joint dq from q derivative are not truth. Exclude window half-width endpoints, no run has >.1s gaps. CAM SI, rates Nm, gyro rad/s.', 'runs': {}}
    for ident, trace in traces.items():
        x = {k: np.asarray(v) for k, v in trace.items()}
        t = x['t']
        grid = np.arange(t[0], t[-1], 1 / 30)
        assert np.max(np.diff(t)) < .1
        def interp(values):
            return np.column_stack([np.interp(grid, t, values[:, j]) for j in range(values.shape[1])])
        qg, dqg, wg = interp(x['q']), interp(x['dq']), interp(x['omega'])
        rg = Slerp(t, Rotation.from_matrix(x['rotation']))(grid)
        rmat = rg.as_matrix()
        wm = (rg[1:] * rg[:-1].inv()).as_rotvec() * 30
        wquat = np.column_stack([np.interp(grid, .5 * (grid[1:] + grid[:-1]), wm[:, j]) for j in range(3)])
        duration = audit['runs'][ident]['duration']
        masks = {'move': grid <= duration, 'fade': (grid > duration) & (grid <= duration + .5), 'late': (grid > duration + 7) & (grid <= duration + 9)}
        result = {}
        for width in (5, 9):
            velocities = {
                'sensor': (savgol_filter(dqg, width, 2, axis=0), savgol_filter(wg, width, 2, axis=0)),
                'quaternion_root': (savgol_filter(dqg, width, 2, axis=0), savgol_filter(wquat, width, 2, axis=0)),
                'position_derivatives': (savgol_filter(qg, width, 2, deriv=1, delta=1 / 30, axis=0), savgol_filter(wquat, width, 2, axis=0)),
            }
            wresult = {}
            for source, (velocities_joint, omega_world) in velocities.items():
                values = []
                for qj, dqj, r, w in zip(qg, velocities_joint, rmat, omega_world):
                    q = pin.neutral(model)
                    q[qi] = qj
                    q[3:7] = Rotation.from_matrix(r).as_quat()
                    v = np.zeros(model.nv)
                    v[vi], v[3:6] = dqj, r.T @ w
                    pin.computeCentroidalMap(model, data, q)
                    values.append(data.Ag[3:] @ v)
                values = np.array(values)
                rate = savgol_filter(values, width, 2, deriv=1, delta=1 / 30, axis=0)
                values_body = np.einsum('nji,nj->ni', rmat, values)
                body_rate = savgol_filter(values_body, width, 2, deriv=1, delta=1 / 30, axis=0)
                rotate_only = np.einsum('nij,nj->ni', rmat, body_rate)
                corrected_rate = rotate_only + np.cross(omega_world, values)
                phases = {}
                for name, mask0 in masks.items():
                    mask = mask0.copy()
                    mask[:width // 2] = mask[-(width // 2):] = False
                    phases[name] = {
                        'n': int(mask.sum()), 'total_h_xy': vrms(values[mask, :2]),
                        'total_h_xyz': vrms(values[mask]),
                        'total_rate_xy': vrms(rate[mask, :2]),
                        'root_omega_xy': vrms(omega_world[mask, :2]),
                        'frame_transport_term_rms': vrms(np.cross(omega_world, values)[mask]),
                        'frame_corrected_vs_direct_rate_error_rms': vrms((corrected_rate - rate)[mask]),
                    }
                wresult[source] = phases
            result[str(width)] = wresult
        out['runs'][ident] = result
        print(ident, 'Hxy move sensor/quat/qderivative width9',
              [result['9'][source]['move']['total_h_xy'] for source in velocities])
    if args.baseline:
        assert out == json.loads(args.baseline.read_text()), 'Original sensitivity audit differs'
    out['source_hashes'] = {
        'mechanism_summary_sha256': digest(args.audit),
        'traces_sha256': digest(args.traces),
        'urdf_sha256': audit['urdf_sha256'],
        'raw': {key: run['raw_sha256'] for key, run in audit['runs'].items()},
    }
    out['numeric_environment'] = audit['numeric_environment']
    out['pinocchio_version'] = audit['pinocchio_version']
    if args.verify_frozen:
        frozen_path = Path(__file__).with_name('cam_sensitivity_summary.json')
        assert out == json.loads(frozen_path.read_text()), 'Frozen sensitivity summary differs'
        print('Exact frozen sensitivity comparison passed')
    args.output.write_text(json.dumps(out, indent=2, allow_nan=False) + '\n')
    print('Summary:', args.output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--audit', type=Path, default=Path('/tmp/kilo/mechanism_summary.json'))
    parser.add_argument('--traces', type=Path, default=Path('/tmp/kilo/iter7b_retained_traces.json'))
    parser.add_argument('--output', type=Path, default=Path('/tmp/kilo/cam_sensitivity_summary.json'))
    parser.add_argument('--baseline', type=Path, help='Optional original sensitivity JSON for exact migration check')
    parser.add_argument('--verify-frozen', action='store_true')
    main(parser.parse_args())
