# disable lower body links and hand links
disabled_links = [
    "pelvis","imu_link",
    "left_hip_yaw_link","left_hip_pitch_link","left_hip_roll_link",
    "left_knee_link","left_ankle_pitch_link","left_ankle_roll_link",
    "right_hip_yaw_link","right_hip_pitch_link","right_hip_roll_link",
    "right_knee_link","right_ankle_pitch_link","right_ankle_roll_link",
    "L_hand_base_link","L_index_proximal","L_index_intermediate",
    "L_middle_proximal","L_middle_intermediate","L_pinky_proximal",
    "L_pinky_intermediate","L_ring_proximal","L_ring_intermediate",
    "L_thumb_proximal_base","L_thumb_proximal","L_thumb_intermediate",
    "L_thumb_distal",
    "logo_link",
    "R_hand_base_link","R_index_proximal","R_index_intermediate",
    "R_middle_proximal","R_middle_intermediate","R_pinky_proximal",
    "R_pinky_intermediate","R_ring_proximal","R_ring_intermediate",
    "R_thumb_proximal_base","R_thumb_proximal","R_thumb_intermediate",
    "R_thumb_distal"
]

# complete list of all links
all_links = [
    "pelvis","imu_link",
    "left_hip_yaw_link","left_hip_pitch_link","left_hip_roll_link",
    "left_knee_link","left_ankle_pitch_link","left_ankle_roll_link",
    "right_hip_yaw_link","right_hip_pitch_link","right_hip_roll_link",
    "right_knee_link","right_ankle_pitch_link","right_ankle_roll_link",
    "torso_link",
    "left_shoulder_pitch_link","left_shoulder_roll_link","left_shoulder_yaw_link",
    "left_elbow_pitch_link","left_elbow_roll_link",
    "left_wrist_pitch_link","left_wrist_yaw_link",
    "L_hand_base_link","L_index_proximal","L_index_intermediate",
    "L_middle_proximal","L_middle_intermediate","L_pinky_proximal",
    "L_pinky_intermediate","L_ring_proximal","L_ring_intermediate",
    "L_thumb_proximal_base","L_thumb_proximal","L_thumb_intermediate","L_thumb_distal",
    "right_shoulder_pitch_link","right_shoulder_roll_link","right_shoulder_yaw_link",
    "right_elbow_pitch_link","right_elbow_roll_link",
    "right_wrist_pitch_link","right_wrist_yaw_link",
    "logo_link",
    "R_hand_base_link","R_index_proximal","R_index_intermediate",
    "R_middle_proximal","R_middle_intermediate","R_pinky_proximal",
    "R_pinky_intermediate","R_ring_proximal","R_ring_intermediate",
    "R_thumb_proximal_base","R_thumb_proximal","R_thumb_intermediate","R_thumb_distal"
]

# enabled links
enabled_links = [
    "torso_link",
    "left_shoulder_pitch_link","left_shoulder_roll_link","left_shoulder_yaw_link",
    "left_elbow_pitch_link","left_elbow_roll_link",
    "left_wrist_pitch_link","left_wrist_yaw_link",
    "torso_link",
    "right_shoulder_pitch_link","right_shoulder_roll_link","right_shoulder_yaw_link",
    "right_elbow_pitch_link","right_elbow_roll_link",
    "right_wrist_pitch_link","right_wrist_yaw_link"
]

output_file = "./assets/h1_2/h1_2_collision.srdf"

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
