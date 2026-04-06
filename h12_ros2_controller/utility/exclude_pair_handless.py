# disable lower body links and hand links
disabled_links = [
    'pelvis', 'imu_link',
    'left_hip_yaw_link','left_hip_pitch_link','left_hip_roll_link',
    'left_knee_link','left_ankle_pitch_link','left_ankle_roll_link',
    'right_hip_yaw_link','right_hip_pitch_link','right_hip_roll_link',
    'right_knee_link','right_ankle_pitch_link','right_ankle_roll_link',
    'logo_link',
]

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

# enabled links
enabled_links = [
    'torso_link',
    'left_shoulder_pitch_link', 'left_shoulder_roll_link', 'left_shoulder_yaw_link',
    'left_elbow_link',
    'left_wrist_roll_link', 'left_wrist_pitch_link', 'left_wrist_yaw_link',
    'torso_link',
    'right_shoulder_pitch_link', 'right_shoulder_roll_link', 'right_shoulder_yaw_link',
    'right_elbow_link',
    'right_wrist_roll_link', 'right_wrist_pitch_link', 'right_wrist_yaw_link'
]

output_file = './assets/h1_2/h1_2_handless_collision.srdf'

with open(output_file, "w") as f:
    f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
    f.write('<!-- SRDF fragment: disable collisions for lower body and hands -->\n')
    f.write('<robot name="h1_2">\n\n')
    f.write('  <!-- disable_collisions entries -->\n')
    # disable collisions between disabled links and all other links
    for dl in disabled_links:
        for other in all_links:
            if dl == other:
                continue
            f.write(f'  <disable_collisions link1="{dl}" link2="{other}" reason="Never"/>\n')

    # disable collisions between consecutive enabled links
    for i in range(len(enabled_links) - 1):
        f.write(f'  <disable_collisions link1="{enabled_links[i]}" link2="{enabled_links[i+1]}" reason="Never"/>\n')
    f.write('\n</robot>\n')

print(f"Wrote {len(disabled_links)*(len(all_links)-1)} entries to '{output_file}'.")
