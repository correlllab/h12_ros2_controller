'''Generate a cuRobo robot_cfg.yml for the H1-2 arms.

Derives the cuRobo config from the existing collision assets so the cuRobo
backend and the OMPL/Pinocchio backend share identical self-collision geometry:

  - collision_spheres  <- fixed sphere links in h1_2_handless_sphere.urdf
  - self_collision_ignore <- disable_collisions in *_sphere_collision.srdf
  - cspace              <- the 14 arm joints (ENABLED_JOINTS), home = zeros
  - lock_joints         <- legs + torso, fixed at nominal stand

Pure stdlib (xml.etree) — no ROS / cuRobo / torch needed. Run:

    python3 -m h12_ros2_controller.utility.gen_curobo_cfg [urdf] [srdf] [out]

Defaults resolve the assets from the repo's CL_Assets submodule and write the
generated cuRobo YAML alongside them.
'''
import os
import re
import sys
import xml.etree.ElementTree as ET

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, '..', '..'))
_ROS_ASSETS = os.path.join(_REPO, 'submodules', 'CL_Assets', 'ros_assets')
_CUROBO_ASSETS = os.path.join(_REPO, 'submodules', 'CL_Assets', 'curobo_assets')

URDF = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    _ROS_ASSETS, 'h1_2_handless_sphere.urdf')
SRDF = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    _ROS_ASSETS, 'h1_2_handless_sphere_collision.srdf')
OUT = sys.argv[3] if len(sys.argv) > 3 else os.path.join(
    _CUROBO_ASSETS, 'h1_2_handless_curobo.yml')

# the 14 movable arm joints (must match ENABLED_JOINTS, left arm then right)
ARM_JOINTS = [
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint',
    'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint',
    'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
]
def main():
    # the URDF uses unprefixed drake: namespaced tags; neutralize for ET
    with open(URDF) as f:
        root = ET.fromstring(f.read().replace('drake:', 'drake_'))

    link_radius = {}
    for link in root.findall('link'):
        sph = link.find('collision/geometry/sphere')
        if sph is not None:
            link_radius[link.get('name')] = float(sph.get('radius'))

    spheres = {}       # parent_link -> [{center, radius}]
    parent_of = {}     # child_link -> parent_link
    for joint in root.findall('joint'):
        child = joint.find('child').get('link')
        parent = joint.find('parent').get('link')
        parent_of[child] = parent
        if re.search(r'_sphere\d+$', child):
            xyz = [float(v) for v in joint.find('origin').get('xyz').split()]
            spheres.setdefault(parent, []).append(
                {'center': xyz, 'radius': link_radius[child]})

    collision_links = list(spheres.keys())

    # self-collision ignore: SRDF pairs where both links carry spheres, plus
    # each link's nearest collision-link ancestor (chain neighbours touch)
    ignore = {L: set() for L in collision_links}
    srdf_pairs = 0
    for dc in ET.parse(SRDF).getroot().findall('disable_collisions'):
        a, b = dc.get('link1'), dc.get('link2')
        if a in ignore and b in ignore:
            ignore[a].add(b)
            ignore[b].add(a)
            srdf_pairs += 1

    def nearest_collision_ancestor(link):
        cur = parent_of.get(link)
        while cur is not None:
            if cur in ignore:
                return cur
            cur = parent_of.get(cur)
        return None

    for L in collision_links:
        anc = nearest_collision_ancestor(L)
        if anc is not None:
            ignore[L].add(anc)
            ignore[anc].add(L)

    write_yaml(OUT, collision_links, spheres, ignore)
    n_spheres = sum(len(v) for v in spheres.values())
    print(f'wrote {OUT}: {len(collision_links)} links, {n_spheres} spheres, '
          f'{srdf_pairs} SRDF ignore pairs')


def fmt(x):
    return f'{x:.6g}'


def write_yaml(out, collision_links, spheres, ignore):
    L14 = ', '.join(['0.0'] * 14)
    W14 = ', '.join(['1.0'] * 14)
    urdf_name = os.path.basename(URDF)
    srdf_name = os.path.basename(SRDF)
    lines = [
        '# cuRobo robot config for the Unitree H1-2 arms.',
        '# GENERATED from the matching sphere URDF and collision SRDF.',
        f'#   {urdf_name} + {srdf_name} by utility/gen_curobo_cfg.py.',
        '# Consumed directly by CuroboJointPlanner.',
        '#',
        '# Plans the 14 arm DoF with the torso locked. Collision geometry',
        '# is the same sphere set the OMPL/Pinocchio backend uses, so both',
        '# backends agree on self-collision.',
        'robot_cfg:',
        '  kinematics:',
        '    base_link: "pelvis"',
        '    ee_link: "left_wrist_yaw_link"',
        '    # extra frames tracked for future dual-arm Cartesian goals',
        '    link_names:',
        '      - "right_wrist_yaw_link"',
        '      - "left_grasp_frame"',
        '      - "right_grasp_frame"',
        '    collision_link_names:',
    ]
    lines += [f'      - "{L}"' for L in collision_links]
    lines.append('    collision_sphere_buffer: 0.005')
    lines.append('    self_collision_ignore:')
    for L in collision_links:
        others = ', '.join(f'"{o}"' for o in sorted(ignore[L]))
        lines.append(f'      "{L}": [{others}]')
    lines.append('    self_collision_buffer:')
    lines += [f'      "{L}": 0.0' for L in collision_links]
    lines += [
        '    # cuRobo builds only the pelvis-to-wrist tree, so leg joints must',
        '    # not be locked here. CuroboJointPlanner supplies torso_joint at',
        '    # its current measured position before constructing MotionGen.',
        '    lock_joints:',
    ]
    lines.append('      torso_joint: 0.0')
    lines.append('    collision_spheres:')
    for L in collision_links:
        lines.append(f'      {L}:')
        for s in spheres[L]:
            c = s['center']
            lines.append(
                f'        - center: [{fmt(c[0])}, {fmt(c[1])}, {fmt(c[2])}]')
            lines.append(f'          radius: {fmt(s["radius"])}')
    lines += [
        '    # cspace lists the 14 movable arm joints. The torso is locked at',
        '    # construction and inactive arm joints are locked per plan.',
        '    cspace:',
        '      joint_names:',
    ]
    lines += [f'        - "{j}"' for j in ARM_JOINTS]
    lines += [
        f'      retract_config: [{L14}]  # home',
        f'      null_space_weight: [{W14}]',
        f'      cspace_distance_weight: [{W14}]',
        '      max_acceleration: 15.0',
        '      max_jerk: 500.0',
        '',
    ]
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        f.write('\n'.join(lines))


if __name__ == '__main__':
    main()
