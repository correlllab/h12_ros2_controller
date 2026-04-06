# number of spheres per link
N = 4

# disable lower body links and hand links
disabled_links = [
    'pelvis', 'imu_link',
    'left_hip_yaw_link','left_hip_pitch_link','left_hip_roll_link',
    'left_knee_link','left_ankle_pitch_link','left_ankle_roll_link',
    'right_hip_yaw_link','right_hip_pitch_link','right_hip_roll_link',
    'right_knee_link','right_ankle_pitch_link','right_ankle_roll_link',
    'logo_link',
]

# append N copies with names 'link_sphere1' to 'link_sphereN' for disabled_links
disabled_link_full = []
for link in disabled_links:
    disabled_link_full.append(link)
    for i in range(1, N + 1):
        disabled_link_full.append(f'{link}_sphere{i}')
disabled_links = disabled_link_full

# complete list of all links
all_links = [
    'pelvis', 'imu_link',
    'left_hip_yaw_link', 'left_hip_pitch_link', 'left_hip_roll_link',
    'left_knee_link', 'left_ankle_pitch_link', 'left_ankle_roll_link',
    'right_hip_yaw_link', 'right_hip_pitch_link', 'right_hip_roll_link',
    'right_knee_link', 'right_ankle_pitch_link', 'right_ankle_roll_link',
    'torso_link',
    'left_shoulder_pitch_link', 'left_shoulder_roll_link', 'left_shoulder_yaw_link',
    'left_elbow_link',
    'left_wrist_roll_link', 'left_wrist_pitch_link', 'left_wrist_yaw_link',
    'right_shoulder_pitch_link', 'right_shoulder_roll_link', 'right_shoulder_yaw_link',
    'right_elbow_link',
    'right_wrist_roll_link', 'right_wrist_pitch_link', 'right_wrist_yaw_link',
    'logo_link',
]

# append N copies with names 'link_sphere1' to 'link_sphereN' for all_links
all_links_full = []
for link in all_links:
    all_links_full.append(link)
    for i in range(1, N + 1):
        all_links_full.append(f'{link}_sphere{i}')
all_links = all_links_full

# enabled links
left_arm_links = [
    'left_shoulder_pitch_link',
    'left_shoulder_roll_link',
    'left_shoulder_yaw_link',
    'left_elbow_link',
    'left_wrist_roll_link',
    'left_wrist_pitch_link',
    'left_wrist_yaw_link',
]

right_arm_links = [
    'right_shoulder_pitch_link',
    'right_shoulder_roll_link',
    'right_shoulder_yaw_link',
    'right_elbow_link',
    'right_wrist_roll_link',
    'right_wrist_pitch_link',
    'right_wrist_yaw_link',
]

output_file = './assets/h1_2/h1_2_handless_sphere_collision.srdf'

with open(output_file, "w") as f:
    f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
    f.write('<!-- SRDF fragment: disable collisions for lower body and hands -->\n')
    f.write('<robot name="h1_2_sphere">\n\n')
    f.write('  <!-- disable_collisions entries -->\n')
    # disable collisions between disabled links and all other links
    for dl in disabled_links:
        for other in all_links:
            if dl == other:
                continue
            f.write(f'  <disable_collisions link1="{dl}" link2="{other}" reason="Never"/>\n')

    # disable collisions in single enabled links
    # torso
    for i in range(1, N + 1):
        f.write(f'  <disable_collisions link1="torso_link" link2="torso_link_sphere{i}" reason="Never"/>\n')
        for j in range(i + 1, N + 1):
            f.write(f'  <disable_collisions link1="torso_link_sphere{i}" link2="torso_link_sphere{j}" reason="Never"/>\n')
    # left arm
    for i in range(len(left_arm_links)):
        for j in range(1, N + 1):
            f.write(f'  <disable_collisions link1="{left_arm_links[i]}" link2="{left_arm_links[i]}_sphere{j}" reason="Never"/>\n')
            for k in range(j + 1, N + 1):
                f.write(f'  <disable_collisions link1="{left_arm_links[i]}_sphere{j}" link2="{left_arm_links[i]}_sphere{k}" reason="Never"/>\n')
    # right arm
    for i in range(len(right_arm_links)):
        for j in range(1, N + 1):
            f.write(f'  <disable_collisions link1="{right_arm_links[i]}" link2="{right_arm_links[i]}_sphere{j}" reason="Never"/>\n')
            for k in range(j + 1, N + 1):
                f.write(f'  <disable_collisions link1="{right_arm_links[i]}_sphere{j}" link2="{right_arm_links[i]}_sphere{k}" reason="Never"/>\n')

    # disable collisions between consecutive enabled links
    # torso
    f.write(f'  <disable_collisions link1="torso_link" link2="{left_arm_links[0]}" reason="Never"/>\n')
    f.write(f'  <disable_collisions link1="torso_link" link2="{right_arm_links[0]}" reason="Never"/>\n')
    for i in range(1, N + 1):
        f.write(f'  <disable_collisions link1="torso_link_sphere{i}" link2="{left_arm_links[0]}" reason="Never"/>\n')
        f.write(f'  <disable_collisions link1="torso_link_sphere{i}" link2="{right_arm_links[0]}" reason="Never"/>\n')
        for j in range(1, N + 1):
            f.write(f'  <disable_collisions link1="torso_link_sphere{i}" link2="{left_arm_links[0]}_sphere{j}" reason="Never"/>\n')
            f.write(f'  <disable_collisions link1="torso_link_sphere{i}" link2="{right_arm_links[0]}_sphere{j}" reason="Never"/>\n')
    # left arm
    for i in range(len(left_arm_links) - 1):
        f.write(f'  <disable_collisions link1="{left_arm_links[i]}" link2="{left_arm_links[i + 1]}" reason="Never"/>\n')
        for j in range(1, N + 1):
            f.write(f'  <disable_collisions link1="{left_arm_links[i]}" link2="{left_arm_links[i + 1]}_sphere{j}" reason="Never"/>\n')
            f.write(f'  <disable_collisions link1="{left_arm_links[i]}_sphere{j}" link2="{left_arm_links[i + 1]}" reason="Never"/>\n')
            for k in range(1, N + 1):
                f.write(f'  <disable_collisions link1="{left_arm_links[i]}_sphere{j}" link2="{left_arm_links[i + 1]}_sphere{k}" reason="Never"/>\n')
    # right arm
    for i in range(len(right_arm_links) - 1):
        f.write(f'  <disable_collisions link1="{right_arm_links[i]}" link2="{right_arm_links[i + 1]}" reason="Never"/>\n')
        for j in range(1, N + 1):
            f.write(f'  <disable_collisions link1="{right_arm_links[i]}" link2="{right_arm_links[i + 1]}_sphere{j}" reason="Never"/>\n')
            f.write(f'  <disable_collisions link1="{right_arm_links[i]}_sphere{j}" link2="{right_arm_links[i + 1]}" reason="Never"/>\n')
            for k in range(1, N + 1):
                f.write(f'  <disable_collisions link1="{right_arm_links[i]}_sphere{j}" link2="{right_arm_links[i + 1]}_sphere{k}" reason="Never"/>\n')

    # disable collisions between second consecutive links
    # left arm
    for i in range(len(left_arm_links) - 2):
        f.write(f'  <disable_collisions link1="{left_arm_links[i]}" link2="{left_arm_links[i + 2]}" reason="Never"/>\n')
        for j in range(1, N + 1):
            f.write(f'  <disable_collisions link1="{left_arm_links[i]}" link2="{left_arm_links[i + 2]}_sphere{j}" reason="Never"/>\n')
            f.write(f'  <disable_collisions link1="{left_arm_links[i]}_sphere{j}" link2="{left_arm_links[i + 2]}" reason="Never"/>\n')
            for k in range(1, N + 1):
                f.write(f'  <disable_collisions link1="{left_arm_links[i]}_sphere{j}" link2="{left_arm_links[i + 2]}_sphere{k}" reason="Never"/>\n')
    # right arm
    for i in range(len(right_arm_links) - 2):
        f.write(f'  <disable_collisions link1="{right_arm_links[i]}" link2="{right_arm_links[i + 2]}" reason="Never"/>\n')
        for j in range(1, N + 1):
            f.write(f'  <disable_collisions link1="{right_arm_links[i]}" link2="{right_arm_links[i + 2]}_sphere{j}" reason="Never"/>\n')
            f.write(f'  <disable_collisions link1="{right_arm_links[i]}_sphere{j}" link2="{right_arm_links[i + 2]}" reason="Never"/>\n')
            for k in range(1, N + 1):
                f.write(f'  <disable_collisions link1="{right_arm_links[i]}_sphere{j}" link2="{right_arm_links[i + 2]}_sphere{k}" reason="Never"/>\n')

    f.write('\n</robot>\n')

print(f'SRDF file generated: {output_file}')
