import numpy as np
import pinocchio as pin
from scipy.spatial import cKDTree


class PointCloudChecker:
    '''Check reduced-model collision spheres against an obstacle point cloud'''

    def __init__(self, robot_model, margin=0.02):
        assert robot_model.init_reduced, 'Reduced model is not initialized'
        assert robot_model.init_collision, 'Collision model is not initialized'
        self.margin = float(margin)
        # reuse the reduced sphere collision model as the robot body geometry
        self.model = robot_model.model_body_reduced
        self.geom_model = robot_model.collision_model_body_reduced
        self.data = self.model.createData()
        self.geom_data = self.geom_model.createData()
        # sphere radii never change, so cache them once
        self.radii = np.asarray(
            [geom.geometry.radius for geom in self.geom_model.geometryObjects],
            dtype=float,
        )
        # point cloud stays empty until a cloud is provided
        self._points = None
        self._tree = None

    @property
    def has_point_cloud(self):
        '''Whether a non-empty obstacle point cloud is currently loaded'''
        return self._tree is not None

    def update_point_cloud(self, points):
        '''Replace the obstacle point cloud; pass None or empty to clear it'''
        if points is None:
            self._points = None
            self._tree = None
            return
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        if points.shape[0] == 0:
            self._points = None
            self._tree = None
            return
        # cache points and a kd-tree for fast nearest-neighbour queries
        self._points = points
        self._tree = cKDTree(points)

    def check_collision_free_reduced(self, q_reduced):
        '''Return True when no collision sphere overlaps the point cloud'''
        if self._tree is None:
            return True
        centers = self._sphere_centers(q_reduced)
        # nearest cloud point to every sphere center in one batched query
        distances, _ = self._tree.query(centers)
        return bool(np.all(distances > self.radii + self.margin))

    def _sphere_centers(self, q_reduced):
        '''World-frame centers of every reduced collision sphere for q'''
        q_reduced = np.asarray(q_reduced, dtype=float)
        pin.forwardKinematics(self.model, self.data, q_reduced)
        pin.updateGeometryPlacements(
            self.model, self.data, self.geom_model, self.geom_data
        )
        return np.asarray(
            [placement.translation for placement in self.geom_data.oMg],
            dtype=float,
        )
