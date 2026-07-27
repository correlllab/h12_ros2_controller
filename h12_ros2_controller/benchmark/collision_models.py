'''Shared collision-model resolution and CLI model selection'''

from pathlib import Path

from h12_ros2_controller.utility.path_definition import (
    SRDF_HANDLESS_COLLISION_PATH,
    SRDF_HANDLESS_SPHERE_COLLISION_PATH,
    SRDF_MAGPIE_PATH,
    SRDF_MAGPIE_SPHERE_PATH,
    URDF_HANDLESS_PATH,
    URDF_HANDLESS_SPHERE_PATH,
    URDF_MAGPIE_PATH,
    URDF_MAGPIE_SPHERE_PATH,
)

_MODEL_VARIANTS = {
    'magpie': {
        'sphere': (URDF_MAGPIE_SPHERE_PATH, SRDF_MAGPIE_SPHERE_PATH),
        'mesh': (URDF_MAGPIE_PATH, SRDF_MAGPIE_PATH),
    },
    'handless': {
        'sphere': (
            URDF_HANDLESS_SPHERE_PATH,
            SRDF_HANDLESS_SPHERE_COLLISION_PATH,
        ),
        'mesh': (URDF_HANDLESS_PATH, SRDF_HANDLESS_COLLISION_PATH),
    },
}


def get_collision_models(variant):
    '''Return sphere and mesh (URDF, SRDF) tuples for one robot variant'''
    if variant not in _MODEL_VARIANTS:
        raise ValueError(
            f'Unknown model variant {variant!r}. '
            f'Choose from: {sorted(_MODEL_VARIANTS)}'
        )
    return _MODEL_VARIANTS[variant].copy()


def add_model_flag(parser):
    '''Add a required mutually exclusive --magpie / --handless flag group'''
    group = parser.add_argument_group('robot model')
    exclusive = group.add_mutually_exclusive_group(required=True)
    exclusive.add_argument('--magpie', action='store_true')
    exclusive.add_argument('--handless', action='store_true')
    return group


def resolve_model(args):
    '''Return the selected model variant name from parsed CLI arguments'''
    if args.magpie:
        return 'magpie'
    return 'handless'


def model_output_path(default, variant):
    '''Insert the model variant name before the file extension'''
    path = Path(default)
    return str(path.with_stem(f'{path.stem}_{variant}'))
