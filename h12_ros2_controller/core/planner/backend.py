'''Selectable joint-space planning backend.

Both backends expose the same contract::

    backend = create_joint_planner(robot_model, config)
    result = backend.plan(start, goal, active_mask=None)  # -> PlanResult

so everything downstream (PlannerClient, upper_controller.execute_path, the
ROS action servers) is agnostic to which planner produced the path.

'ompl' is the CPU planner and is always usable. 'curobo' is GPU-only. 'auto'
prefers cuRobo when a CUDA GPU is available and otherwise falls back to OMPL,
which keeps the headless CPU sim (Mac/Colima) working with no config changes.

Backend classes are imported lazily so a host missing one library (no CUDA on
the Mac sim, or an ompl-free GPU host) can still import this module.
'''


class PlannerBackendError(RuntimeError):
    '''Raised when a requested planner backend cannot be constructed'''


def _make_ompl(robot_model, config):
    from h12_ros2_controller.core.planner.reduced_joint_planner import (
        ReducedJointPlanner,
    )
    return ReducedJointPlanner(robot_model, config)


def _make_curobo(robot_model, config):
    from h12_ros2_controller.core.planner.curobo_planner import (
        CuroboJointPlanner,
    )
    return CuroboJointPlanner(robot_model, config)


def curobo_available():
    '''Return True when cuRobo is importable and a CUDA device is present'''
    try:
        import torch
    except Exception:
        return False
    try:
        if not torch.cuda.is_available():
            return False
    except Exception:
        return False
    try:
        import curobo  # noqa: F401
    except Exception:
        return False
    return True


def create_joint_planner(robot_model, config):
    '''Construct the planning backend selected by config.backend'''
    backend = (getattr(config, 'backend', 'auto') or 'auto').lower()

    if backend == 'ompl':
        return _make_ompl(robot_model, config)

    if backend == 'curobo':
        # explicit request: fail loudly rather than silently degrade
        if not curobo_available():
            raise PlannerBackendError(
                'planner backend "curobo" requested but cuRobo/CUDA is '
                'unavailable on this host; install cuRobo on a GPU machine '
                "or set planner.backend to 'ompl' or 'auto'"
            )
        return _make_curobo(robot_model, config)

    if backend == 'auto':
        if curobo_available():
            try:
                return _make_curobo(robot_model, config)
            except Exception as exc:
                # a misconfigured cuRobo backend must never take down planning
                print(
                    f'[planner] cuRobo backend unavailable ({exc}); '
                    'falling back to OMPL',
                    flush=True,
                )
        return _make_ompl(robot_model, config)

    raise PlannerBackendError(f'Unsupported planner backend: {config.backend}')
