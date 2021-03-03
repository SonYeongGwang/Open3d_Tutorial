import numpy as np
import open3d as o3d
import math

mesh_org = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
o3d.visualization.draw_geometries([mesh_org])

roll = 30

roll = roll * math.pi/180.0

transformation1 = np.array([[1, 0, 0, 0],
                           [0, math.cos(roll), math.sin(roll), 0],
                           [0, -math.sin(roll), math.cos(roll), 0],
                           [0, 0, 0, 1]])

transformation2 = np.array([[1, 0, 0, 2],
                            [0, 1, 0, 0],
                            [0, 0, 1, 2],
                            [0, 0, 0, 10]])

mesh_rot_roll = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.7)
mesh_trans = o3d.geometry.TriangleMesh.create_coordinate_frame(size=7)
mesh_rot_roll.transform(transformation1)
mesh_trans.transform(transformation2)

o3d.visualization.draw_geometries([mesh_org, mesh_rot_roll, mesh_trans])