'''
debug script to understand free-flyer base DoF structure
'''
import numpy as np
import pinocchio as pin

# load model without free-flyer
print('Without Free-Flyer (fixed base)')
print('================================')
model_fixed, _, _ = pin.buildModelsFromUrdf(
    filename='./assets/h1_2/h1_2.urdf',
    package_dirs='./assets/h1_2'
)
print(f'nq (configuration dim) = {model_fixed.nq}')
print(f'nv (velocity dim)      = {model_fixed.nv}')
print(f'njoints                = {model_fixed.njoints}')
print()

# load model with free-flyer
print('With Free-Flyer (floating base)')
print('================================')
model, _, _ = pin.buildModelsFromUrdf(
    filename='./assets/h1_2/h1_2.urdf',
    package_dirs='./assets/h1_2',
    root_joint=pin.JointModelFreeFlyer()
)
data = model.createData()
print(f'nq (configuration dim) = {model.nq}')
print(f'nv (velocity dim)      = {model.nv}')
print(f'njoints                = {model.njoints}')
print()

# iterate through first few joints
print('H1_2 Robot Joints (first 5)')
print('===========================')
for j_idx in range(min(5, model.njoints)):
    joint = model.joints[j_idx]
    print(f'Joint {j_idx}: {model.names[j_idx]}')
    print(f'    idx_q: {joint.idx_q}, nq: {joint.nq}')
    print(f'    idx_v: {joint.idx_v}, nv: {joint.nv}')
print()

# q vector layout
print('Configuration Vector (q) Layout')
print('================================')
print('q[0:3]  = base position (x, y, z)')
print('q[3:7]  = base quaternion (x, y, z, w)  <-- xyzw order!')
print('q[7:]   = motor joint positions')
print()

# v vector layout
print('Velocity Vector (v) Layout')
print('==========================')
print('v[0:3]  = base linear velocity (vx, vy, vz)')
print('v[3:6]  = base angular velocity (wx, wy, wz)')
print('v[6:]   = motor joint velocities')
print()

# test forward kinematics with base configuration
print('Forward Kinematics Test')
print('=======================')
q = np.zeros(model.nq)
q[3:7] = [0, 0, 0, 1]  # identity quaternion (xyzw)

pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)
pelvis_id = model.getFrameId('pelvis')
pelvis_pose = data.oMf[pelvis_id]
print(f'Identity base (q[0:7] = {q[0:7]}):')
print(f'  pelvis position: {pelvis_pose.translation}')
print()

# translate base
q[0:3] = [1.0, 2.0, 0.5]
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)
pelvis_pose = data.oMf[pelvis_id]
print(f'Translated base (q[0:7] = {q[0:7]}):')
print(f'  pelvis position: {pelvis_pose.translation}')
print()

# rotate base 45 deg around z
angle = np.pi / 4
q[0:3] = [0, 0, 0]
q[3:7] = [0, 0, np.sin(angle/2), np.cos(angle/2)]
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)
pelvis_pose = data.oMf[pelvis_id]
print(f'Rotated base 45 deg z (q[0:7] = {np.round(q[0:7], 3)}):')
print(f'  pelvis position: {pelvis_pose.translation}')
print(f'  pelvis rotation:\n{np.round(pelvis_pose.rotation, 3)}')

