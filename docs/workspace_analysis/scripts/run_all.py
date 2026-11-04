'''Run the full workspace analysis: sweeps, explanation tables, report and figures'''
import argparse
from multiprocessing import Pool

import config as cfg
import sweep
import report
import figures


def main(args):
    if not args.report_only:
        with Pool(args.workers, maxtasksperchild=500) as pool:
            for pose in cfg.POSES:
                sweep.sweep('left', pose['pitch'], pose['yaw'], pose['roll'], 'fine', pool)
                sweep.mirror(pose, pool)
            if not args.skip_tables:
                for pose in cfg.POSES:
                    if pose['free'] == 'yaw':
                        # fine grid, needed to score each yaw inside the chosen box
                        for yaw in cfg.YAWS:
                            if yaw != pose['yaw']:
                                sweep.sweep('left', pose['pitch'], yaw, pose['roll'], 'yaw', pool,
                                            fill_rounds=0)
                    else:
                        for roll in cfg.ROLLS:
                            sweep.sweep('left', pose['pitch'], pose['yaw'], roll, 'scan', pool,
                                        step=cfg.COARSE_STEP, fill_rounds=0)
    report.main(argparse.Namespace(out=args.out))
    figures.main(argparse.Namespace(out=args.out))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run the full workspace analysis')
    parser.add_argument('--workers', type=int, default=12,
                        help='parallel IK workers (keep below the core count)')
    parser.add_argument('--out', default=cfg.REPORT_DIR)
    parser.add_argument('--skip-tables', action='store_true',
                        help='skip the yaw/roll sweeps behind the explanation tables')
    parser.add_argument('--report-only', action='store_true',
                        help='rebuild report and figures from existing data')
    main(parser.parse_args())
