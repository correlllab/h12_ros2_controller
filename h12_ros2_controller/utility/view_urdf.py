import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# URDF path
urdf_path = 'assets/h1_2/h1_2_sphere.urdf'
model_path = 'assets/h1_2'

# load model
model, visual, collision = pin.buildModelsFromUrdf(urdf_path,
                                      package_dirs=model_path)
viz = MeshcatVisualizer(model, visual, collision)
viz.initViewer(open=True)
viz.loadViewerModel()

while True:
    # display robot at neutral configuration
    viz.display(pin.neutral(model))
