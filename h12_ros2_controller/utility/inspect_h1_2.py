import pinocchio as pin

# load model
model, _, _ = pin.buildModelsFromUrdf(filename='./assets/h1_2/h1_2.urdf',
                                      package_dirs='./assets/h1_2')
# model, _, _ = pin.buildModelsFromUrdf(filename='./assets/h1_2/h1_2_sphere.urdf')
# model, _, _ = pin.buildModelsFromMJCF('./assets/h1_2/h1_2.xml')
data  = model.createData()

print(f'nq (configuration dim) = {model.nq}')
print(f'nv (velocity dim)      = {model.nv}')
print(f'nbodies                = {model.nbodies}')
print(f'njoints                = {model.njoints}')

# iterate through bodies
print('H1_2 Robot Bodies')
print('=================')
for frame in model.frames:
    if frame.type == pin.FrameType.BODY:
        print(f'Body {frame.name}')
        print(f'Parent Joint: {model.names[frame.parentJoint]}')
        print()

# iterate through joints
print('H1_2 Robot Joints')
print('=================')
for j_idx, joint in enumerate(model.joints):
    # Note: joint[0] is the universe/root; skip if you like
    print(f'''Joint {j_idx}: {model.names[j_idx]}
          idx_q: {joint.idx_q}, idx_v: {joint.idx_v}
          ''')
