import numpy as np
import open3d as o3d

mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(0.5)
mesh2 = o3d.geometry.TriangleMesh.create_coordinate_frame(0.4)
mesh_eu = o3d.geometry.TriangleMesh.create_coordinate_frame(0.5)
mesh_fx = o3d.geometry.TriangleMesh.create_coordinate_frame(0.4)
tr = np.array([[ 1,  0, 0, 0],
               [ 0,  np.math.cos(np.math.pi/4), -np.math.sin(np.math.pi/4), 0],
               [ 0,  np.math.sin(np.math.pi/4), np.math.cos(np.math.pi/4), 0],
               [ 0,  0 ,0, 1]])
tr3 = np.array([[ np.math.cos(np.math.pi/4),  0, -np.math.sin(np.math.pi/4), 0],
               [ 0,                           1,  0,                         0],
               [ np.math.sin(np.math.pi/4),   0,  np.math.cos(np.math.pi/4), 0],
               [ 0,                           0,  0,                         1]])
tr_fx = np.dot(tr3, tr)
tr_eu = np.dot(tr, tr3)
mesh_eu.transform(tr_eu)
mesh_fx.transform(tr_fx)
o3d.visualization.draw_geometries([mesh, mesh_eu, mesh_fx])