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

transformation3 = np.array([
    [ 2.90040405e-03,  5.49825709e-02, -3.71633300e-01, -6.06459842e-04],
    [-2.97180570e-01,  2.27689755e-01,  3.13670072e-02, -4.54455377e-02],
    [ 2.29821890e-01,  2.93729633e-01,  4.52504841e-02, -5.97049404e-01],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
     ])

transformation4 = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1/100]])

mesh_rot_roll = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.7)
mesh_trans = o3d.geometry.TriangleMesh.create_coordinate_frame(size=7)
mesh_trans2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=7)
mesh_rot_roll.transform(transformation1)
# mesh_trans.transform(transformation2)
mesh_trans2.transform(transformation4)

o3d.visualization.draw_geometries([mesh_trans,  mesh_trans2])