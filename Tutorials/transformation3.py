import open3d as o3d
import numpy as np

mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)

rotation_mat_x = o3d.geometry.get_rotation_matrix_from_xyz(np.array([-np.math.pi/2, 0, 0]))
rotation_mat_y = o3d.geometry.get_rotation_matrix_from_xyz(np.array([0, np.math.pi/2, 0]))
# print(rotation_mat)

rotated = o3d.geometry.TriangleMesh.create_coordinate_frame(0.05)
rotated_xy = o3d.geometry.TriangleMesh.create_coordinate_frame(0.03)
rotated.rotate(rotation_mat_x)

rotation_mat_xy = np.dot(rotation_mat_y, rotation_mat_x, )

o3d.visualization.draw_geometries([mesh, rotated])
rotated.rotate(rotation_mat_y)
o3d.visualization.draw_geometries([mesh, rotated])

rotated_xy.rotate(rotation_mat_xy)
o3d.visualization.draw_geometries([mesh, rotated, rotated_xy])